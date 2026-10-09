// The browser half of 2_session_viewer.py. It is read as text and pasted into
// the generated page, so it must be a standalone module with no imports beyond
// the 3D library the page embeds.
//
// Everything it draws comes from the json blob in #scene-data: bars where mocap
// found them, their 8 markers, the authored pose beside each one, the cell's
// solids, and one footprint per robot (the clicked bar's arm is drawn in full).

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const DATA = JSON.parse(document.getElementById('scene-data').textContent);

// * Green for good through to red for bad -- the reading everyone already has.
// * Every stop is dark enough to hold its own against the white page.
// ! Red and green are the classic pair that colour-blind readers cannot
// ! separate; the brightness still climbs then falls across the ramp, and the
// ! legend counts the bars in each band, so the numbers never depend on hue
// ! alone.
const RAMP = [[26,138,62],[86,163,54],[140,185,46],[190,200,40],[232,199,36],
              [242,170,32],[237,134,30],[226,92,34],[206,36,38]];
const OVER = [124, 20, 110];      // past the top of the scale: clearly off-ramp
const NODATA = [168, 176, 188];   // nothing measurable for this bar

const BAR_SIDE = 0.0225;          // how thick to draw a bar, metres
const MARKER_R = 0.019;
const FOOTPRINT = 0x1d3f72;       // dark blue, for the robot outlines + arrows

// * Two independent choices: which measurement drives the COLOUR, and which
// * one each bar is labelled with. They start the same but need not stay so --
// * colouring by placement while reading the load, say.
let metric = DATA.metrics[0];        // drives the gradient and the legend
let dataMetric = DATA.metrics[0];    // drives the number written on each bar
let selected = null;

/** One measurement's definition, found by name rather than by position. */
const metricByKey = key => DATA.metrics.find(m => m.key === key);

// ---------------------------------------------------------------------------
// * Colour
// ---------------------------------------------------------------------------

/** Colour for one value on the current metric's scale. */
function colourFor(value) {
  if (value === null || value === undefined) return NODATA;
  if (value > metric.high) return OVER;
  const t = Math.max(0, value / metric.high) * (RAMP.length - 1);
  const i = Math.min(RAMP.length - 2, Math.floor(t));
  const f = t - i;
  return [0, 1, 2].map(c => Math.round(RAMP[i][c] + (RAMP[i + 1][c] - RAMP[i][c]) * f));
}

const hex = c => (c[0] << 16) | (c[1] << 8) | c[2];
const css = c => `rgb(${c[0]},${c[1]},${c[2]})`;

// ! A colour written straight into a vertex buffer is taken as-is, while
// ! `material.color.setHex()` quietly converts from screen colours to the ones
// ! the renderer works in. Shading a bar per vertex therefore has to do that
// ! conversion itself, or the bars come out washed out next to everything else.
// ? The ramp has only a handful of distinct colours, so each one is converted
// ? once and kept.
const _converted = new Map();
function vertexColour(value) {
  const shade = colourFor(value);
  const code = hex(shade);
  if (!_converted.has(code)) {
    const colour = new THREE.Color().setRGB(
      shade[0] / 255, shade[1] / 255, shade[2] / 255, THREE.SRGBColorSpace);
    _converted.set(code, [colour.r, colour.g, colour.b]);
  }
  return _converted.get(code);
}

/** One bar's value for a given measurement, or null when it has none. */
function valueFor(bar, which) {
  if (which.key === 'load' && !bar.load_valid) return null;
  const v = bar[which.key];
  return (v === null || v === undefined) ? null : v;
}

/** The value a bar contributes to the COLOUR metric. */
function valueOf(bar) { return valueFor(bar, metric); }

// ---------------------------------------------------------------------------
// * Scene
// ---------------------------------------------------------------------------

const host = document.getElementById('scene');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
host.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xffffff);

// The cell is Z-up, so the camera has to be told that before anything else.
const camera = new THREE.PerspectiveCamera(42, 1, 0.05, 400);
camera.up.set(0, 0, 1);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.screenSpacePanning = true;

scene.add(new THREE.HemisphereLight(0xffffff, 0xc2cad6, 2.0));
const key = new THREE.DirectionalLight(0xffffff, 1.1); key.position.set(4, -5, 6);
const fill = new THREE.DirectionalLight(0xffffff, 0.45); fill.position.set(-5, 4, 3);
scene.add(key, fill);
// GridHelper lies in XZ by default; rotate it into the floor plane.
const grid = new THREE.GridHelper(24, 24, 0xc4ccd6, 0xe4e8ee);
grid.rotateX(Math.PI / 2);
scene.add(grid);

const groups = {};
for (const name of ['env', 'bars', 'authored', 'markers', 'robots', 'arm',
                    'cameras', 'labels', 'notes']) {
  groups[name] = new THREE.Group();
  scene.add(groups[name]);
}
groups.cameras.visible = false;   // off until asked for; the rig is busy

// ---------------------------------------------------------------------------
// * Building the pieces
// ---------------------------------------------------------------------------

/** Turn the packed base64 back into numbers. */
function unpack(text, Type) {
  const raw = atob(text);
  const bytes = new Uint8Array(raw.length);
  for (let i = 0; i < raw.length; i++) bytes[i] = raw.charCodeAt(i);
  return new Type(bytes.buffer);
}

/** A box stretched between two points, used for every bar. */
function barMesh(a, b, side, material) {
  const start = new THREE.Vector3().fromArray(a);
  const end = new THREE.Vector3().fromArray(b);
  const axis = new THREE.Vector3().subVectors(end, start);
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(side, side, axis.length()), material);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), axis.clone().normalize());
  mesh.position.copy(start).addScaledVector(axis, 0.5);
  return mesh;
}

if (DATA.env) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position',
    new THREE.BufferAttribute(unpack(DATA.env.positions, Float32Array), 3));
  geometry.setIndex(new THREE.BufferAttribute(unpack(DATA.env.indices, Uint32Array), 1));
  geometry.computeVertexNormals();
  groups.env.add(new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({
    color: 0x8fb4d9, transparent: true, opacity: 0.22,
    roughness: 0.9, metalness: 0.0, side: THREE.DoubleSide, depthWrite: false,
  })));
  groups.env.add(new THREE.LineSegments(new THREE.EdgesGeometry(geometry, 25),
    new THREE.LineBasicMaterial({ color: 0x4a7fb5, transparent: true, opacity: 0.6 })));
}

// * Label text is drawn into a canvas slightly larger than it appears, so that
// * shrinking it down leaves clean edges. How much larger depends on the
// * screen: on a retina display one CSS pixel is already two real ones, so the
// * canvas has to grow with it. Aim for ~1.4 texture pixels per real pixel --
// * enough to antialias, not so much that the downscale softens the glyphs.
const LABEL_TEXT_PX = 11;                       // matches #bands in the legend
const LABEL_FONT_PX = Math.round(LABEL_TEXT_PX * renderer.getPixelRatio() * 1.4);
const LABEL_CANVAS_H = Math.ceil(LABEL_FONT_PX * 1.6);   // room for the halo
const LABEL_SS = LABEL_FONT_PX / LABEL_TEXT_PX;          // halo/padding scale

/** A floating label that can be redrawn when the shown measurement changes. */
function labelSprite() {
  const canvas = document.createElement('canvas');
  canvas.height = LABEL_CANVAS_H;
  canvas.width = 16;
  const texture = new THREE.CanvasTexture(canvas);
  // ! Without this the texture gets a mipmap chain, and at the size these are
  // ! drawn the GPU samples a blurrier level of it -- which is what made the
  // ! text look soft. Sampling the full-resolution image keeps it crisp.
  texture.generateMipmaps = false;
  texture.minFilter = THREE.LinearFilter;
  texture.magFilter = THREE.LinearFilter;
  texture.anisotropy = renderer.capabilities.getMaxAnisotropy();

  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({
    map: texture, transparent: true, depthTest: false }));
  sprite.userData.aspect = 1;
  sprite.userData.draw = text => {
    const ctx = canvas.getContext('2d');
    const font = `600 ${LABEL_FONT_PX}px ui-sans-serif,system-ui,sans-serif`;
    // * Size the canvas to the text, so a short label does not carry a wide
    // * transparent margin that crowds its neighbours.
    ctx.font = font;
    const width = Math.ceil(ctx.measureText(text).width) + LABEL_SS * 6;
    // ! A texture's room on the graphics card is claimed ONCE, at the size of
    // ! the first picture it is given, and every later picture is copied into
    // ! that same room -- a copy that fails quietly when the sizes disagree.
    // ! So a label that changes width, which is exactly what happens when the
    // ! measurement being shown changes, would keep showing the OLD text (or a
    // ! squashed mix of the two) unless its texture is thrown away first.
    if (width !== canvas.width) {
      canvas.width = width;
      texture.dispose();
    }
    sprite.userData.aspect = canvas.width / canvas.height;
    // Resizing the canvas clears it, and so does this when it kept its width.
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    // Resizing the canvas also resets the context, so set it up again.
    ctx.font = font;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    // Dark text with a white halo, so a label stays legible over the pale
    // environment as well as over a dark bar.
    ctx.lineWidth = LABEL_SS * 1.6;
    ctx.strokeStyle = 'rgba(255,255,255,.92)';
    ctx.fillStyle = '#17202e';
    ctx.strokeText(text, canvas.width / 2, canvas.height / 2);
    ctx.fillText(text, canvas.width / 2, canvas.height / 2);
    texture.needsUpdate = true;
  };
  return sprite;
}

/** A thick round rod between two points.
 *
 * ! Real geometry rather than a line: WebGL ignores `linewidth`, so a
 * ! LineBasicMaterial is always one pixel however thick you ask for.
 *
 * `steps` cuts the rod into rings along its length. One ring is enough for a
 * rod of a single colour; a rod that shades from one end to the other needs
 * enough of them for the change to look smooth.
 */
function rodBetween(a, b, radius, material, steps = 1) {
  const start = new THREE.Vector3().fromArray(a);
  const end = new THREE.Vector3().fromArray(b);
  const axis = new THREE.Vector3().subVectors(end, start);
  const rod = new THREE.Mesh(
    new THREE.CylinderGeometry(radius, radius, axis.length(), 10, steps), material);
  // CylinderGeometry stands along +Y; turn it onto the segment.
  rod.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0),
                                    axis.clone().normalize());
  rod.position.copy(start).addScaledVector(axis, 0.5);
  return rod;
}

/** The same rod, broken into dashes. */
function dashedRod(a, b, radius, material, dash, gap) {
  const start = new THREE.Vector3().fromArray(a);
  const end = new THREE.Vector3().fromArray(b);
  const axis = new THREE.Vector3().subVectors(end, start);
  const total = axis.length();
  const step = axis.clone().normalize();
  const group = new THREE.Group();
  for (let at = 0; at < total; at += dash + gap) {
    const run = Math.min(dash, total - at);
    group.add(rodBetween(
      start.clone().addScaledVector(step, at).toArray(),
      start.clone().addScaledVector(step, at + run).toArray(), radius, material));
  }
  return group;
}

const BAR_RADIUS = 0.011;          // the drawn centre line, metres
// * How solid the detected bar is drawn. It is see-through so the dashed
// * authored rod behind it stays readable -- at a few millimetres of error the
// * two sit almost on top of each other. Turn this down towards 0.5 if the
// * authored rod matters more than the colour; up towards 1 for the reverse.
const BAR_OPACITY = 0.75;
// * Rings along a bar, for the placement shading. Enough that the change from
// * one end to the other reads as a gradient rather than as steps.
const BAR_STEPS = 24;

/** How far apart the detected and authored centre lines are at one point.
 *
 * Both bars are straight, so the gap between them changes smoothly from one
 * end to the other: the same measurement the detail card reports at the start,
 * the middle and the end, but available at every point in between.
 *
 * @param {object} bar one bar's record
 * @param {number} t 0 at the start tip, 1 at the end tip
 * @returns {number} the distance in millimetres
 */
function gapAt(bar, t) {
  return bar._gapStart.clone().lerp(bar._gapEnd, t).length() * 1000;
}

/** Shade one bar's rod from its own measurements.
 *
 * ? Only the placement error exists at every point ALONG a bar; every other
 * ? measurement is one number for the whole thing, so the rod is flat for them.
 */
function paintRod(bar) {
  const position = bar._rod.geometry.getAttribute('position');
  const colour = bar._rod.geometry.getAttribute('color');
  const flat = metric.key === 'placement' ? null : vertexColour(valueOf(bar));
  for (let i = 0; i < position.count; i++) {
    // The cylinder is built along its own +Y and turned onto the bar
    // afterwards, so the untransformed Y says how far along the bar a vertex is.
    const along = (position.getY(i) + bar._length / 2) / bar._length;
    const shade = flat || vertexColour(gapAt(bar, along));
    colour.setXYZ(i, shade[0], shade[1], shade[2]);
  }
  colour.needsUpdate = true;
}

const pickable = [];
for (const bar of DATA.bars) {
  // * Each bar is its CENTRE LINE: a solid rod where mocap found it, a dashed
  // * one where it was authored. Centre lines rather than full-thickness bars
  // * because at a few millimetres of error the two would overlap into one shape.
  // * The colour is per vertex rather than per rod, so the placement error can
  // * be shown shading along the bar instead of as one flat number.
  const material = new THREE.MeshStandardMaterial({
    roughness: 0.5, metalness: 0.1, vertexColors: true,
    transparent: true, opacity: BAR_OPACITY, depthWrite: false });
  const rod = rodBetween(bar.fitted[0], bar.fitted[1], BAR_RADIUS, material,
                         BAR_STEPS);
  rod.geometry.setAttribute('color', new THREE.BufferAttribute(
    new Float32Array(rod.geometry.getAttribute('position').count * 3), 3));
  groups.bars.add(rod);
  bar._rod = rod;
  bar._gapStart = new THREE.Vector3().fromArray(bar.fitted[0])
    .sub(new THREE.Vector3().fromArray(bar.authored[0]));
  bar._gapEnd = new THREE.Vector3().fromArray(bar.fitted[1])
    .sub(new THREE.Vector3().fromArray(bar.authored[1]));
  bar._length = new THREE.Vector3().fromArray(bar.fitted[0])
    .distanceTo(new THREE.Vector3().fromArray(bar.fitted[1]));

  groups.authored.add(dashedRod(bar.authored[0], bar.authored[1],
    BAR_RADIUS * 0.72,
    new THREE.MeshStandardMaterial({ color: 0x55606f, roughness: 0.7 }),
    0.055, 0.04));

  // ! A rod is still fiddly to hit, so an invisible sleeve does the picking.
  const sleeve = rodBetween(bar.fitted[0], bar.fitted[1], BAR_RADIUS * 3,
    new THREE.MeshBasicMaterial({ visible: false }));
  sleeve.userData.bar = bar;
  groups.bars.add(sleeve);
  pickable.push(sleeve);

  // The markers pick the same bar, so a click near one does the obvious thing.
  const markerGeometry = new THREE.SphereGeometry(MARKER_R, 10, 8);
  const markerMaterial = new THREE.MeshStandardMaterial({ roughness: 0.55 });
  for (const point of bar.markers) {
    const dot = new THREE.Mesh(markerGeometry, markerMaterial);
    dot.position.fromArray(point);
    dot.userData.bar = bar;
    groups.markers.add(dot);
    pickable.push(dot);
  }

  const label = labelSprite();
  label.position.fromArray(bar.fitted[1]).z += 0.12;
  groups.labels.add(label);
  bar._marker = markerMaterial;
  bar._label = label;
}

// Every robot's footprint, so all 20 parking spots read at once.
if (DATA.robot) {
  const [length, width] = DATA.robot.footprint;
  for (const bar of DATA.bars) {
    if (!bar.robot) continue;
    const base = new THREE.Matrix4().fromArray(bar.robot.base);
    const half = [length / 2, width / 2];
    const outline = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(-half[0], -half[1], 0), new THREE.Vector3(half[0], -half[1], 0),
      new THREE.Vector3(half[0], half[1], 0), new THREE.Vector3(-half[0], half[1], 0),
      new THREE.Vector3(-half[0], -half[1], 0),
    ]);
    const rectangle = new THREE.Line(outline,
      // Short, closely spaced dashes: a finer outline reads as a footprint
      // rather than as a line someone drew around the robot.
      new THREE.LineDashedMaterial({ color: FOOTPRINT,
        dashSize: 0.022, gapSize: 0.018 }));
    // ! A dashed line stays solid until the per-vertex distances are worked out.
    rectangle.computeLineDistances();
    // A flat triangle just past the front edge says which way the robot faced.
    const nose = new THREE.Mesh(
      new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(half[0] + 0.11, 0, 0),
        new THREE.Vector3(half[0] + 0.02, 0.05, 0),
        new THREE.Vector3(half[0] + 0.02, -0.05, 0),
      ]),
      new THREE.MeshBasicMaterial({ color: FOOTPRINT, side: THREE.DoubleSide }));
    const holder = new THREE.Group();
    holder.add(rectangle, nose);
    holder.applyMatrix4(base);
    holder.userData.bar = bar;
    groups.robots.add(holder);
  }
}

// The mocap camera rig: a small body at each camera, and a cone showing where
// it looks. Motive records no field of view, so the cone's width is a readable
// stand-in -- it says where a camera points, not exactly what it can see.
const CAMERA_COLOUR = 0x0f8c7e;
if (DATA.cameras && DATA.cameras.length) {
  const reach = DATA.cone.length;
  const radius = reach * Math.tan(DATA.cone.half_angle_deg * Math.PI / 180);

  const coneGeometry = new THREE.ConeGeometry(radius, reach, 20, 1, true);
  // ! ConeGeometry puts its apex at +Y and its wide end at -Y, which is the
  // ! wrong way round for a lens: flip it, then slide it so the APEX sits on
  // ! the camera and the opening widens out along the line of sight.
  coneGeometry.rotateX(Math.PI);
  coneGeometry.translate(0, reach / 2, 0);
  const coneMaterial = new THREE.MeshBasicMaterial({ color: CAMERA_COLOUR,
    transparent: true, opacity: 0.12, side: THREE.DoubleSide, depthWrite: false });

  // * The outline is what actually reads on a white page -- four whiskers from
  // * the lens out to the rim, plus the rim itself. A faint translucent cone on
  // * its own all but disappears.
  const outline = [];
  for (let i = 0; i < 20; i++) {
    const a = (i / 20) * Math.PI * 2;
    const b = ((i + 1) / 20) * Math.PI * 2;
    outline.push(new THREE.Vector3(radius * Math.cos(a), reach, radius * Math.sin(a)),
                 new THREE.Vector3(radius * Math.cos(b), reach, radius * Math.sin(b)));
    if (i % 5 === 0) {
      outline.push(new THREE.Vector3(0, 0, 0),
                   new THREE.Vector3(radius * Math.cos(a), reach, radius * Math.sin(a)));
    }
  }
  const outlineGeometry = new THREE.BufferGeometry().setFromPoints(outline);
  const outlineMaterial = new THREE.LineBasicMaterial({ color: CAMERA_COLOUR });

  const bodyGeometry = new THREE.BoxGeometry(0.16, 0.22, 0.16);
  const bodyMaterial = new THREE.MeshStandardMaterial({ color: CAMERA_COLOUR,
    roughness: 0.55 });

  for (const camera of DATA.cameras) {
    const holder = new THREE.Group();
    holder.add(new THREE.Mesh(coneGeometry, coneMaterial));
    holder.add(new THREE.LineSegments(outlineGeometry, outlineMaterial));
    holder.add(new THREE.Mesh(bodyGeometry, bodyMaterial));
    holder.applyMatrix4(new THREE.Matrix4().fromArray(camera.matrix));
    groups.cameras.add(holder);
  }
}

// The robot's link shapes, built once and reused by whichever bar is selected.
const linkGeometries = {};
if (DATA.robot && DATA.robot.meshes) {
  for (const [name, packed] of Object.entries(DATA.robot.meshes)) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(
      unpack(packed.positions, Float32Array), 3));
    geometry.setIndex(new THREE.BufferAttribute(
      unpack(packed.indices, packed.bits === 16 ? Uint16Array : Uint32Array), 1));
    geometry.computeVertexNormals();
    linkGeometries[name] = geometry;
  }
}

/** Draw the selected bar's robot with its real link geometry. */
function showArm(bar) {
  groups.arm.clear();
  if (!bar || !bar.robot) return;
  const material = new THREE.MeshStandardMaterial({ color: 0xb9c2cf,
    transparent: true, opacity: 0.62, roughness: 0.65, metalness: 0.1 });
  for (const [name, matrix] of Object.entries(bar.robot.links)) {
    const geometry = linkGeometries[name];
    if (!geometry) continue;
    // Each link's shape is stored in its own frame, so placing it is exactly
    // one matrix -- which is why every robot costs only 23 matrices.
    const mesh = new THREE.Mesh(geometry, material);
    mesh.matrixAutoUpdate = false;
    mesh.matrix.fromArray(matrix);
    groups.arm.add(mesh);
  }
}

// ---------------------------------------------------------------------------
// * Colour, legend and the side panel
// ---------------------------------------------------------------------------

/** How many decimals a metric deserves, given how fine its scale is. */
function decimalsFor(metric) {
  return metric.high <= 0.5 ? 3 : 2;
}

function recolour() {
  for (const bar of DATA.bars) {
    paintRod(bar);
    bar._marker.color.setHex(hex(colourFor(valueOf(bar))));
  }
  relabel();
  drawLegend();
  if (selected) showDetail(selected);
}

// * Measurements that are one reading PER GRIPPER rather than one for the whole
// * bar. These are never written as a single label: the pair is the interesting
// * part -- the left flange is driven exact while the right takes up the grasp
// * mismatch -- and "L 0.24 · R 0.43" squeezed together at one end of the bar
// * says nothing about which wrist is which. They go at the two grip points
// * instead, whether or not the bar has been clicked.
const PER_GRIPPER = new Set(['servo']);

/** True while the chosen measurement is read once per gripper. */
const perGripper = () => PER_GRIPPER.has(dataMetric.key);

/** True when this bar actually has those per-gripper numbers. */
function hasGripperNumbers(bar) {
  return perGripper()
    && bar.servo_left !== null && bar.servo_left !== undefined;
}

/** The number one measurement writes on a bar.
 *
 * @param {object} bar one bar's record
 * @param {object} which the measurement to read
 * @returns {string} the text, or "--" when that bar has no such reading
 */
function labelText(bar, which) {
  const value = valueFor(bar, which);
  if (value === null) return '--';
  return `${value.toFixed(decimalsFor(which))} ${which.unit}`;
}

/** Write the "data by" measurement onto every bar. */
function relabel() {
  for (const bar of DATA.bars) {
    // * A bar whose numbers are written along the bar itself keeps only its
    // * name here -- the selected bar, and every bar at once while the
    // * measurement is one per gripper.
    const nameOnly = bar === selected || hasGripperNumbers(bar);
    bar._label.userData.draw(nameOnly
      ? bar.bar : `${bar.bar} · ${labelText(bar, dataMetric)}`);
  }
}

function drawLegend() {
  const values = DATA.bars.map(valueOf);
  const inside = values.filter(v => v !== null && v <= metric.high).length;
  const above = values.filter(v => v !== null && v > metric.high).length;
  const none = values.filter(v => v === null).length;
  const stops = RAMP.map((c, i) =>
    `${css(c)} ${(100 * i / (RAMP.length - 1)).toFixed(0)}%`).join(',');

  let bands = `<div><span class="swatch" style="background:${css(RAMP[4])}"></span>`
    + `0 – ${metric.high} ${metric.unit} · ${inside} bar${inside === 1 ? '' : 's'}</div>`;
  if (above) bands += `<div><span class="swatch" style="background:${css(OVER)}"></span>`
    + `above ${metric.high} ${metric.unit} · ${above} bar${above === 1 ? '' : 's'}`
    + ` — ${DATA.bars.filter(b => { const v = valueOf(b); return v !== null && v > metric.high; })
        .map(b => b.bar).join(', ')}</div>`;
  if (none) bands += `<div><span class="swatch" style="background:${css(NODATA)}"></span>`
    + `not measured · ${none} bar${none === 1 ? '' : 's'}</div>`;

  // ! Only the body is rebuilt -- the header holds the picker, which would lose
  // ! its handler if it were replaced on every redraw.
  document.getElementById('legend-body').innerHTML =
    `<div id="ramp" style="background:linear-gradient(90deg,${stops})"></div>`
    + `<div id="ticks"><span>0 ${metric.unit}</span>`
    + `<span>${metric.high / 2}</span>`
    + `<span>${metric.high}</span></div><div id="bands">${bands}</div>`;
}

const fmt = (v, d, u) => (v === null || v === undefined)
  ? '<span class="muted">--</span>' : `${Number(v).toFixed(d)} ${u}`;
const row = (k, v) => `<div class="row"><span class="muted">${k}</span><span>${v}</span></div>`;
/** The same value, in bold when it is the one being asked for. */
const strong = (text, yes) => yes ? `<b>${text}</b>` : text;

/** One titled block of the card.
 *
 * The block holding the "data by" measurement is marked and its own numbers
 * are set in bold, so clicking a bar answers the question the picker asked
 * rather than leaving the reader to find it among the others.
 *
 * @param {string} key which measurement this block is about
 * @param {string} title the heading
 * @param {string} sub a line of explanation, or ''
 * @param {string} body the rows
 * @returns {string} the block's html
 */
function block(key, title, sub, body) {
  const chosen = dataMetric.key === key;
  return `<div class="block${chosen ? ' focus' : ''}"><h2>${title}`
    + (chosen ? '<span class="pill">data by</span>' : '') + '</h2>'
    + (sub ? `<div class="sub">${sub}</div>` : '') + body + '</div>';
}

const detailBox = document.getElementById('detail');

function showDetail(bar) {
  const asked = key => dataMetric.key === key;
  let html = '<button id="detail-close" title="unpin (Esc)">&times;</button>'
    + `<h2>bar ${bar.bar}</h2>`
    + `<div class="sub">${bar.file}${bar.run ? '<br>' + bar.run : ''}</div>`
    + row('bar length', fmt(bar.bar_length, 4, 'm'))
    + row('takes', `${bar.n_takes} (start spread `
          + `${bar.placement_spread.toFixed(2)} mm)`);

  // * What the placement number is made of. It is the WORST of three places,
  // * so all three are shown: alike means the bar is shifted bodily, different
  // * means it is tilted.
  const worst = Math.max(bar.tip_start, bar.tip_middle, bar.tip_end);
  const place = (name, value) => row(
    name, strong(fmt(value, 2, 'mm'), asked('placement') && value === worst));
  html += block('placement', 'placement error',
    'distance between the detected and authored bar at three places',
    place('at the start', bar.tip_start)
    + place('at the middle', bar.tip_middle)
    + place('at the end', bar.tip_end)
    + row('<b>worst of the three</b> <span class="muted">= placement error'
          + '</span>', `<b>${fmt(bar.placement, 2, 'mm')}</b>`)
    + row('average of the three', fmt(bar.tip_mean, 2, 'mm')));

  html += block('rotation', 'rotation error',
    'angle between the detected and authored bar axes',
    row('rotation error',
        strong(fmt(bar.rotation, 3, 'deg'), asked('rotation'))));

  let pairs = row('worst pair, worst take',
                  strong(fmt(bar.fit_residual, 2, 'mm'), asked('fit_residual')))
            + row('typical take <span class="muted">worst pair</span>',
                  fmt(bar.fit_residual_mean, 2, 'mm'));
  if (bar.pair_residuals) {
    const worstPair = Math.max(...bar.pair_residuals);
    bar.pair_residuals.forEach((value, i) => {
      const ids = (bar.pair_ids && bar.pair_ids[i])
        ? bar.pair_ids[i].join(' + ') : `pair ${i + 1}`;
      pairs += row(`markers ${ids}`, strong(fmt(value, 2, 'mm'),
        asked('fit_residual') && value === worstPair));
    });
  }
  html += block('fit_residual', 'fit residual',
    'how far each marker pair sits off the fitted axis, from the start end',
    pairs);

  if (bar.robot_holds_bar !== null && bar.robot_holds_bar !== undefined
      && bar.robot_holds_bar > 330) {
    html += `<div class="flag"><b>Arms not drawn: the joint values in this `
      + `bar's file do not hold the bar.</b> Their worse wrist sits `
      + `${bar.robot_holds_bar.toFixed(0)} mm from it, where a grasp sits at `
      + `about 80 mm -- the file stores a different pose (for B52 on 20261001, `
      + `the bar carried level at chest height) in the slot where the `
      + `assembled pose should be. During the session the monitor ignored those `
      + `joint values and solved IK from the flange targets, which are correct, `
      + `so the robot itself was fine. Only the parking footprint is drawn. `
      + `This is a fault in the export, not in the measurement; re-export the `
      + `BarAction.</div>`;
  }

  html += block('servo', 'servo, last iteration', '', bar.iterations
    ? row('tool0 left', strong(fmt(bar.servo_left, 2, 'mm'), asked('servo')))
      + row('tool0 right', strong(fmt(bar.servo_right, 2, 'mm'), asked('servo')))
      + row('rotation left', fmt(bar.rot_left, 3, 'deg'))
      + row('rotation right', fmt(bar.rot_right, 3, 'deg'))
      + row('iterations', bar.iterations)
    : '<div class="sub">no servo run matched to this bar</div>');

  let load;
  if (!bar.load_valid) {
    load = '<div class="sub warn">load not measured. The force sensor was '
      + 'zeroed while the bar was already gripped, so the bar\'s own load was '
      + 'tared away and this reading cannot say anything about bending.</div>'
      + row('left', fmt(bar.force_left, 2, 'N'))
      + row('right', fmt(bar.force_right, 2, 'N'));
  } else {
    const bending = bar.load > metricByKey('load').high;
    load = `<div class="sub">The gap between the two grippers is what the bar `
      + `absorbs sideways.</div>`
      + row('<b>imbalance</b>', `<b${bending ? ' class="warn"' : ''}>`
        + `${bar.load.toFixed(2)} N</b>`)
      + row('left', strong(fmt(bar.force_left, 2, 'N'), asked('load')))
      + row('right', strong(fmt(bar.force_right, 2, 'N'), asked('load')));
  }
  html += block('load', 'load — is the bar being bent?', '', load);

  // * Every number's definition, one click away. Measuring something is only
  // * useful if the reader knows exactly what was measured.
  html += '<h2>how these are measured</h2>'
    + '<button id="formula-toggle">show the formulas</button>'
    + '<div id="formulas" style="display:none">'
    + DATA.metrics.concat(DATA.extra_formulas || []).map(m =>
        `<div class="formula"><b>${m.label}</b> <span class="muted">`
        + `(${m.unit})</span><code>${m.formula}</code>`
        + `<div class="sub">${m.detail || m.help || ''}</div></div>`).join('')
    + '</div>';

  detailBox.innerHTML = html;
  detailBox.classList.add('open');
  // The card sits directly on top of the colour key, however tall it ends up.
  const legend = document.getElementById('legend');
  detailBox.style.bottom = (legend.offsetHeight + 24) + 'px';
  document.getElementById('detail-close').onclick = () => select(null);
  const toggle = document.getElementById('formula-toggle');
  const box = document.getElementById('formulas');
  toggle.onclick = () => {
    const open = box.style.display === 'none';
    box.style.display = open ? 'block' : 'none';
    toggle.textContent = open ? 'hide the formulas' : 'show the formulas';
    detailBox.style.bottom = (document.getElementById('legend').offsetHeight + 24) + 'px';
  };
}

/** Which numbers to write along the selected bar, and where each one goes.
 *
 * The "data by" picker decides, so the numbers floating beside the bar are
 * always the measurement that was asked for -- each one written at the place
 * on the bar it was actually measured at.
 *
 * @param {object} bar one bar's record
 * @returns {Array} ``[{point: Vector3, text: string}, ...]``
 */
function annotationsFor(bar) {
  const along = t => new THREE.Vector3().fromArray(bar.fitted[0])
    .lerp(new THREE.Vector3().fromArray(bar.fitted[1]), t);
  // Where each gripper holds the bar, so a per-wrist number sits by its wrist.
  const atGrip = side => (bar.grasp && bar.grasp[side])
    ? new THREE.Vector3().fromArray(bar.grasp[side]) : null;

  const notes = [];
  const add = (point, text) => {
    if (point && text !== null && text !== undefined) notes.push({ point, text });
  };
  const mm = value => (value === null || value === undefined)
    ? null : `${Number(value).toFixed(2)} mm`;

  if (dataMetric.key === 'placement') {
    add(along(0), mm(bar.tip_start));
    add(along(0.5), mm(bar.tip_middle));
    add(along(1), mm(bar.tip_end));
  } else if (dataMetric.key === 'rotation') {
    add(along(0.5), `${bar.rotation.toFixed(3)} deg`);
  } else if (dataMetric.key === 'fit_residual' && bar.pair_residuals) {
    // Each marker pair's own distance from the fitted axis, at that pair.
    bar.pair_points.forEach((point, i) => {
      add(new THREE.Vector3().fromArray(point), mm(bar.pair_residuals[i]));
    });
  } else if (dataMetric.key === 'servo') {
    add(atGrip('left'), bar.servo_left === null || bar.servo_left === undefined
      ? null : `L ${Number(bar.servo_left).toFixed(2)} mm`);
    add(atGrip('right'), bar.servo_right === null || bar.servo_right === undefined
      ? null : `R ${Number(bar.servo_right).toFixed(2)} mm`);
  } else if (dataMetric.key === 'load') {
    const newtons = (prefix, value) => (value === null || value === undefined)
      ? null : `${prefix}${Number(value).toFixed(2)} N`;
    add(atGrip('left'), newtons('L ', bar.force_left));
    add(atGrip('right'), newtons('R ', bar.force_right));
    // ? The gap is left off a record whose sensor was zeroed with the bar
    // ? already gripped: it reads near nothing there and would look like a
    // ? perfect result. The card says why in full.
    add(along(0.5), bar.load_valid ? newtons('gap ', bar.load)
                                   : 'gap not measured');
  }
  return notes;
}

/** Write those numbers into the scene.
 *
 * Normally only for the bar that was clicked -- all twenty bars' numbers at
 * once is not something anyone can read. A per-gripper measurement is the
 * exception: its two numbers have to sit at the two grip points to mean
 * anything, so they are drawn for every bar even with nothing selected.
 */
function showAnnotations() {
  groups.notes.clear();
  const bars = selected ? [selected] : (perGripper() ? DATA.bars : []);
  for (const bar of bars) {
    for (const note of annotationsFor(bar)) {
      const label = labelSprite();
      label.userData.draw(note.text);
      // Hung UNDER the bar: its name label sits above it, and a number at the
      // end of the bar would otherwise land on top of that name.
      label.position.copy(note.point).z -= 0.07;
      groups.notes.add(label);
    }
  }
}

function select(bar) {
  selected = bar;
  // The chosen bar glows so you can tell at a glance which one the panel is
  // describing, even after you have rotated away from it.
  // * With a bar pinned, only ITS label stays on screen -- 20 labels at once
  // * is what you want for an overview, not while reading one bar.
  for (const other of DATA.bars) {
    other._marker.emissive.setHex(other === bar ? 0x33425c : 0x000000);
    other._label.userData.muted = Boolean(bar) && other !== bar;
  }
  showArm(bar);
  showAnnotations();
  relabel();
  if (bar) {
    showDetail(bar);
  } else {
    detailBox.classList.remove('open');
    detailBox.innerHTML = '';
  }
}

// ---------------------------------------------------------------------------
// * Interaction
// ---------------------------------------------------------------------------

const raycaster = new THREE.Raycaster();
let pressedAt = null;

/** Which bar, if any, sits under the pointer. */
function barUnderPointer(event) {
  const rect = renderer.domElement.getBoundingClientRect();
  const pointer = new THREE.Vector2(
    ((event.clientX - rect.left) / rect.width) * 2 - 1,
    -((event.clientY - rect.top) / rect.height) * 2 + 1);
  raycaster.setFromCamera(pointer, camera);
  const hit = raycaster.intersectObjects(pickable, false)[0];
  return hit ? hit.object.userData.bar : null;
}

/** True when the pointer has barely moved, i.e. this was a click not a drag. */
function wasAClick(event) {
  return pressedAt && Math.hypot(event.clientX - pressedAt.x,
                                 event.clientY - pressedAt.y) < 5;
}

renderer.domElement.addEventListener('pointerdown', e => {
  pressedAt = { x: e.clientX, y: e.clientY, button: e.button };
});

// ! Selecting happens on pointerUP, and only for a click that did not drag --
// ! otherwise rotating the view would keep changing the selection under you.
renderer.domElement.addEventListener('pointerup', e => {
  if (e.button !== 0 || !wasAClick(e)) return;
  const bar = barUnderPointer(e);
  // Clicking empty space leaves the selection alone on purpose: the panel and
  // the arms stay put until they are explicitly dismissed.
  if (bar) select(bar);
});

// ? Right-click deliberately does NOT clear the selection: the right button is
// ? how the view is moved, and a pan that ends without quite enough movement
// ? would throw the card away. Esc and the card's x are the only two ways out.
// ? (OrbitControls already stops the browser menu appearing on a right drag.)

addEventListener('keydown', e => {
  if (e.key === 'Escape') select(null);
  // M cycles the colour metric without reaching for the dropdown.
  if (e.key === 'm' || e.key === 'M') {
    setMetric((DATA.metrics.indexOf(metric) + 1) % DATA.metrics.length);
  }
});

// * Two pickers, each driving one thing: the gradient, and the number written
// * on the bars. "M" still cycles the colour.
/** Fill a picker with the metric list and run `onPick` when it changes. */
function wirePicker(id, chosen, onPick) {
  const picker = document.getElementById(id);
  if (!picker) return;
  picker.innerHTML = DATA.metrics.map((m, i) =>
    `<option value="${i}"${m === chosen ? ' selected' : ''}>${m.label} (${m.unit})</option>`).join('');
  picker.onchange = () => onPick(Number(picker.value));
}

/** Switch which measurement the colours show. */
function setMetric(index) {
  metric = DATA.metrics[index];
  const sidebar = document.getElementById('metric');
  const legend = document.getElementById('metric-legend');
  for (const picker of [sidebar, legend]) if (picker) picker.value = String(index);
  const help = document.getElementById('metric-help');
  if (help) help.textContent = metric.help;
  recolour();
}

/** Switch which measurement is written on each bar, and on the selected one. */
function setDataMetric(index) {
  dataMetric = DATA.metrics[index];
  const picker = document.getElementById('data-legend');
  if (picker) picker.value = String(index);
  relabel();
  // ! The numbers floating beside the bars belong to this picker too. Leaving
  // ! them alone here is what used to show the measurement chosen BEFORE --
  // ! the fit residual's four numbers staying put while the picker said
  // ! "rotation error".
  showAnnotations();
  if (selected) showDetail(selected);
}

wirePicker('metric', metric, setMetric);
wirePicker('metric-legend', metric, setMetric);
wirePicker('data-legend', dataMetric, setDataMetric);

// Each tick box shows or hides one group of things in the scene.
for (const [id, group] of [['show-env', 'env'], ['show-authored', 'authored'],
    ['show-markers', 'markers'], ['show-robots', 'robots'],
    ['show-cameras', 'cameras'], ['show-labels', 'labels']]) {
  const box = document.getElementById(id);
  if (!box) continue;
  box.onchange = () => {
    groups[group].visible = box.checked;
    // The numbers written along the selected bar are labels too, so they come
    // and go with the rest of them.
    if (group === 'labels') groups.notes.visible = box.checked;
    // Showing the rig for the first time also pulls the view out far enough to
    // see it, since it sits well outside the bars.
    if (group === 'cameras' && box.checked) resetView();
  };
}
document.getElementById('reset').onclick = resetView;

/** Frame the view on the bars, widening to take in the mocap rig when shown.
 *
 * ! The cameras hang 2-6 m up and as much as 10 m out, so a view framed on the
 * ! bars alone leaves them off screen -- ticking them on would look like
 * ! nothing happened.
 */
function resetView() {
  const bounds = new THREE.Box3().setFromObject(groups.bars);
  if (groups.cameras.visible && groups.cameras.children.length) {
    bounds.union(new THREE.Box3().setFromObject(groups.cameras));
  }
  const centre = bounds.getCenter(new THREE.Vector3());
  const radius = Math.max(bounds.getSize(new THREE.Vector3()).length() / 2, 1);
  controls.target.copy(centre);
  camera.position.copy(centre).add(
    new THREE.Vector3(radius * 1.1, -radius * 1.3, radius * 0.95));
  camera.near = radius / 100;
  camera.far = radius * 60;
  camera.updateProjectionMatrix();
  controls.update();
}
function resize() {
  const w = host.clientWidth, h = host.clientHeight;
  if (!w || !h) return;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(host);

// ---------------------------------------------------------------------------
// * Go
// ---------------------------------------------------------------------------

setMetric(0);
const withRobot = DATA.bars.filter(b => b.robot).length;
document.getElementById('subtitle').textContent =
  `${DATA.bars.length} bars · generated ${DATA.generated}`;
document.getElementById('status').innerHTML =
  `${DATA.bars.length} bars · ${withRobot} robot poses · `
  + `${DATA.env ? DATA.env.count : 0} environment solids · `
  + `${(DATA.cameras || []).length} mocap cameras<br>`
  + `Distances are millimetres unless marked otherwise.`;

select(null);
resize();
resetView();
window.__viewerReady = true;
// The whole canvas is taller than the glyphs, so the sprite is scaled up by
// that ratio to land the TEXT at LABEL_TEXT_PX on screen.
const LABEL_SPRITE_PX = LABEL_TEXT_PX * (LABEL_CANVAS_H / LABEL_FONT_PX);

renderer.setAnimationLoop(() => {
  // How many world metres one screen pixel covers, one metre from the camera.
  // Derived from the camera and the canvas rather than guessed, so the labels
  // keep their size when the window is resized too.
  const viewportPx = renderer.domElement.clientHeight || 1;
  const spread = 2 * Math.tan((camera.fov * Math.PI / 180) / 2) / viewportPx;
  const size = sprite => {
    const height = LABEL_SPRITE_PX * spread
      * camera.position.distanceTo(sprite.position);
    sprite.scale.set(height * sprite.userData.aspect, height, 1);
  };
  for (const bar of DATA.bars) {
    size(bar._label);
    bar._label.visible = groups.labels.visible && !bar._label.userData.muted;
  }
  for (const label of groups.notes.children) size(label);
  controls.update();
  renderer.render(scene, camera);
});
