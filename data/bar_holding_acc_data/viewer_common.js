// The browser half every 3D viewer page shares: the renderer, scene, camera
// and controls, the drawing primitives (rods, markers, labels, footprints, the
// camera rig, the robot's links), the colour ramp, the legend, the detail card
// and the picking. Each page's own .js imports from here and keeps only what
// is specific to its data -- see 2_session_viewer.js.
//
// ! How this file reaches the page. viewer_common.py's viewer_script() pastes
// ! this file and the page's file into ONE module by dropping the `export`
// ! keywords here and the `import` lines there, so every name below is simply
// ! in scope for the page. That only works under three rules, for both files:
// !   1. imports are ONE line each (`import * as THREE from 'three';`);
// !   2. `export` is written on the declaration itself (`export function`,
// !      `export const`) -- never an `export { ... }` list, never a default;
// !   3. no top-level name declared here is declared again by a page.
// ! Shared mutable state is not allowed either: a function that needs the
// ! page's chosen metric, its selection or its data takes it as a parameter.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ---------------------------------------------------------------------------
// * Colour
// ---------------------------------------------------------------------------

// * Green for good through to red for bad -- the reading everyone already has.
// * Every stop is dark enough to hold its own against the white page.
// ! Red and green are the classic pair that colour-blind readers cannot
// ! separate; the brightness still climbs then falls across the ramp, and the
// ! legend counts the bars in each band, so the numbers never depend on hue
// ! alone.
export const RAMP = [[26,138,62],[86,163,54],[140,185,46],[190,200,40],[232,199,36],
                     [242,170,32],[237,134,30],[226,92,34],[206,36,38]];
export const OVER = [124, 20, 110];      // past the top of the scale: clearly off-ramp
export const NODATA = [168, 176, 188];   // nothing measurable for this bar

export const MARKER_R = 0.019;
export const FOOTPRINT = 0x1d3f72;       // dark blue, for the robot outlines + arrows

/** Colour for one value on a scale that turns to the warning colour past `high`. */
export function colourFor(value, high) {
  if (value === null || value === undefined) return NODATA;
  if (value > high) return OVER;
  const t = Math.max(0, value / high) * (RAMP.length - 1);
  const i = Math.min(RAMP.length - 2, Math.floor(t));
  const f = t - i;
  return [0, 1, 2].map(c => Math.round(RAMP[i][c] + (RAMP[i + 1][c] - RAMP[i][c]) * f));
}

export const hex = c => (c[0] << 16) | (c[1] << 8) | c[2];
export const css = c => `rgb(${c[0]},${c[1]},${c[2]})`;

// ! A colour written straight into a vertex buffer is taken as-is, while
// ! `material.color.setHex()` quietly converts from screen colours to the ones
// ! the renderer works in. Shading a bar per vertex therefore has to do that
// ! conversion itself, or the bars come out washed out next to everything else.
// ? The ramp has only a handful of distinct colours, so each one is converted
// ? once and kept.
const _converted = new Map();
export function vertexColour(value, high) {
  const shade = colourFor(value, high);
  const code = hex(shade);
  if (!_converted.has(code)) {
    const colour = new THREE.Color().setRGB(
      shade[0] / 255, shade[1] / 255, shade[2] / 255, THREE.SRGBColorSpace);
    _converted.set(code, [colour.r, colour.g, colour.b]);
  }
  return _converted.get(code);
}

// ---------------------------------------------------------------------------
// * Scene
// ---------------------------------------------------------------------------

export const host = document.getElementById('scene');
export const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
host.appendChild(renderer.domElement);

export const scene = new THREE.Scene();
scene.background = new THREE.Color(0xffffff);

// The cell is Z-up, so the camera has to be told that before anything else.
export const camera = new THREE.PerspectiveCamera(42, 1, 0.05, 400);
camera.up.set(0, 0, 1);

export const controls = new OrbitControls(camera, renderer.domElement);
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

/** Lay the grid on the measured lab floor. `matrix` is floor frame -> world, 16
 *  numbers column by column (DATA.floor); null leaves the grid at z = 0. */
export function placeFloor(matrix) {
  if (matrix) grid.applyMatrix4(new THREE.Matrix4().fromArray(matrix));
}

// * One group per kind of thing, so a tick box can show or hide a whole kind
// * at once and a page can clear one kind without touching the others.
export const groups = {};
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
export function unpack(text, Type) {
  const raw = atob(text);
  const bytes = new Uint8Array(raw.length);
  for (let i = 0; i < raw.length; i++) bytes[i] = raw.charCodeAt(i);
  return new Type(bytes.buffer);
}

/** The cell's solids, as one translucent mesh plus its edges. Nothing when `env` is empty. */
export function addEnvironment(env) {
  if (!env) return;
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position',
    new THREE.BufferAttribute(unpack(env.positions, Float32Array), 3));
  geometry.setIndex(new THREE.BufferAttribute(unpack(env.indices, Uint32Array), 1));
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
export const LABEL_TEXT_PX = 11;                       // matches #bands in the legend
export const LABEL_FONT_PX = Math.round(LABEL_TEXT_PX * renderer.getPixelRatio() * 1.4);
export const LABEL_CANVAS_H = Math.ceil(LABEL_FONT_PX * 1.6);   // room for the halo
export const LABEL_SS = LABEL_FONT_PX / LABEL_TEXT_PX;          // halo/padding scale
// The whole canvas is taller than the glyphs, so the sprite is scaled up by
// that ratio to land the TEXT at LABEL_TEXT_PX on screen.
export const LABEL_SPRITE_PX = LABEL_TEXT_PX * (LABEL_CANVAS_H / LABEL_FONT_PX);

/** A floating label that can be redrawn when the shown measurement changes. */
export function labelSprite() {
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
export function rodBetween(a, b, radius, material, steps = 1) {
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
export function dashedRod(a, b, radius, material, dash, gap) {
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

// * The drawn centre line, metres. Thin on purpose (half the old 11 mm): the
// * measured and the authored bar sit millimetres apart, and a fat rod hides the gap.
export const BAR_RADIUS = 0.0055;
// The invisible sleeve a click hits, metres: absolute, so a thin bar is not harder to pick.
export const PICK_RADIUS = 0.033;
// * How solid the detected bar is drawn. It is see-through so the dashed
// * authored rod behind it stays readable -- at a few millimetres of error the
// * two sit almost on top of each other. Turn this down towards 0.5 if the
// * authored rod matters more than the colour; up towards 1 for the reverse.
export const BAR_OPACITY = 0.75;
// * Rings along a bar, for the placement shading. Enough that the change from
// * one end to the other reads as a gradient rather than as steps.
export const BAR_STEPS = 24;

/** How far apart the detected and authored centre lines are at one point.
 *
 * Both bars are straight, so the gap between them changes smoothly from one
 * end to the other: the same measurement the detail card reports at the start,
 * the middle and the end, but available at every point in between.
 *
 * @param {object} bar one bar's record, carrying `_gapStart` and `_gapEnd`
 * @param {number} t 0 at the start tip, 1 at the end tip
 * @returns {number} the distance in millimetres
 */
export function gapAt(bar, t) {
  return bar._gapStart.clone().lerp(bar._gapEnd, t).length() * 1000;
}

/** Shade one bar's rod from its own measurements.
 *
 * ? Only the placement error exists at every point ALONG a bar; every other
 * ? measurement is one number for the whole thing, so the rod is flat for them.
 *
 * @param {object} bar one bar's record, carrying `_rod` and `_length`
 * @param {Array|null} flat one rgb triple for the whole rod, or null to shade
 *   each point by its own gap
 * @param {number} high the top of the colour scale
 */
export function paintRod(bar, flat, high) {
  const position = bar._rod.geometry.getAttribute('position');
  const colour = bar._rod.geometry.getAttribute('color');
  for (let i = 0; i < position.count; i++) {
    // The cylinder is built along its own +Y and turned onto the bar
    // afterwards, so the untransformed Y says how far along the bar a vertex is.
    const along = (position.getY(i) + bar._length / 2) / bar._length;
    const shade = flat || vertexColour(gapAt(bar, along), high);
    colour.setXYZ(i, shade[0], shade[1], shade[2]);
  }
  colour.needsUpdate = true;
}

/** One robot's parking spot: a dashed rectangle and a nose saying which way it faced.
 *
 * @param {Array} base16 the base placement, 16 numbers column by column
 * @param {Array} size `[length, width]` of the chassis, metres
 * @param {object} userBar what a click on it should select
 * @returns {THREE.Group} the footprint, already placed
 */
export function footprintFor(base16, size, userBar) {
  const [length, width] = size;
  const base = new THREE.Matrix4().fromArray(base16);
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
  holder.userData.bar = userBar;
  return holder;
}

// The mocap camera rig: a small body at each camera, and a cone showing where
// it looks. Motive records no field of view, so the cone's width is a readable
// stand-in -- it says where a camera points, not exactly what it can see.
export const CAMERA_COLOUR = 0x0f8c7e;

/** Draw the camera rig into `groups.cameras`. `cone` is `{length, half_angle_deg}`. */
export function addCameras(cameras, cone) {
  if (!cameras || !cameras.length) return;
  const reach = cone.length;
  const radius = reach * Math.tan(cone.half_angle_deg * Math.PI / 180);

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

  for (const camera of cameras) {
    const holder = new THREE.Group();
    holder.add(new THREE.Mesh(coneGeometry, coneMaterial));
    holder.add(new THREE.LineSegments(outlineGeometry, outlineMaterial));
    holder.add(new THREE.Mesh(bodyGeometry, bodyMaterial));
    holder.applyMatrix4(new THREE.Matrix4().fromArray(camera.matrix));
    groups.cameras.add(holder);
  }
}

/** The robot's link shapes, unpacked once and shared by every robot drawn.
 *
 * @param {object} meshes `{link: {positions, indices, bits}}`, or nothing
 * @returns {object} `{link: BufferGeometry}`, empty when there is nothing
 */
export function unpackLinkGeometries(meshes) {
  const geometries = {};
  if (!meshes) return geometries;
  for (const [name, packed] of Object.entries(meshes)) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(
      unpack(packed.positions, Float32Array), 3));
    geometry.setIndex(new THREE.BufferAttribute(
      unpack(packed.indices, packed.bits === 16 ? Uint16Array : Uint32Array), 1));
    geometry.computeVertexNormals();
    // ! Shared by every arm drawn, so clearGroup() must leave it alone.
    geometry.userData.shared = true;
    geometries[name] = geometry;
  }
  return geometries;
}

/** One robot's arm (and chassis) with its real link geometry.
 *
 * @param {object} links `{link: 16 numbers}` placing each link in the world
 * @param {object} geometries from `unpackLinkGeometries`
 * @param {number} opacity how solid to draw it
 * @returns {THREE.Group} the links, ready to add to `groups.arm`
 */
export function armGroup(links, geometries, opacity = 0.62) {
  const group = new THREE.Group();
  const material = new THREE.MeshStandardMaterial({ color: 0xb9c2cf,
    transparent: true, opacity, roughness: 0.65, metalness: 0.1 });
  for (const [name, matrix] of Object.entries(links)) {
    const geometry = geometries[name];
    if (!geometry) continue;
    // Each link's shape is stored in its own frame, so placing it is exactly
    // one matrix -- which is why every robot costs only 23 matrices.
    const mesh = new THREE.Mesh(geometry, material);
    mesh.matrixAutoUpdate = false;
    mesh.matrix.fromArray(matrix);
    group.add(mesh);
  }
  return group;
}

/** Empty a group and give its graphics memory back.
 *
 * ! `group.clear()` only unlinks the children: their geometries, materials and
 * ! label textures stay on the graphics card until the page is closed. A page
 * ! that rebuilds its scene on every slider move would leak them all. Anything
 * ! marked `userData.shared` (the robot's link shapes) is kept, because the
 * ! next build uses it again.
 */
export function clearGroup(group) {
  group.traverse(object => {
    if (object === group) return;
    if (object.geometry && !object.geometry.userData.shared) object.geometry.dispose();
    const materials = Array.isArray(object.material) ? object.material
      : (object.material ? [object.material] : []);
    for (const material of materials) {
      if (material.userData.shared) continue;
      if (material.map) material.map.dispose();
      material.dispose();
    }
  });
  group.clear();
}

// ---------------------------------------------------------------------------
// * Numbers, legend and the detail card
// ---------------------------------------------------------------------------

/** How many decimals a metric deserves, given how fine its scale is. */
export function decimalsFor(metric) {
  return metric.high <= 0.5 ? 3 : 2;
}

/** Draw the colour key for one metric.
 *
 * @param {object} metric the measurement the colours show
 * @param {Array} values one value per bar, null where a bar has none
 * @param {Array} overNames the bars past the top of the scale, by name
 */
export function drawLegend(metric, values, overNames) {
  const inside = values.filter(v => v !== null && v <= metric.high).length;
  const above = values.filter(v => v !== null && v > metric.high).length;
  const none = values.filter(v => v === null).length;
  const stops = RAMP.map((c, i) =>
    `${css(c)} ${(100 * i / (RAMP.length - 1)).toFixed(0)}%`).join(',');

  let bands = `<div><span class="swatch" style="background:${css(RAMP[4])}"></span>`
    + `0 – ${metric.high} ${metric.unit} · ${inside} bar${inside === 1 ? '' : 's'}</div>`;
  if (above) bands += `<div><span class="swatch" style="background:${css(OVER)}"></span>`
    + `above ${metric.high} ${metric.unit} · ${above} bar${above === 1 ? '' : 's'}`
    + ` — ${overNames.join(', ')}</div>`;
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

export const fmt = (v, d, u) => (v === null || v === undefined)
  ? '<span class="muted">--</span>' : `${Number(v).toFixed(d)} ${u}`;
export const row = (k, v) => `<div class="row"><span class="muted">${k}</span><span>${v}</span></div>`;
/** The same value, in bold when it is the one being asked for. */
export const strong = (text, yes) => yes ? `<b>${text}</b>` : text;

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
 * @param {string} chosenKey the measurement the "data by" picker asks for
 * @returns {string} the block's html
 */
export function block(key, title, sub, body, chosenKey) {
  const chosen = chosenKey === key;
  return `<div class="block${chosen ? ' focus' : ''}"><h2>${title}`
    + (chosen ? '<span class="pill">data by</span>' : '') + '</h2>'
    + (sub ? `<div class="sub">${sub}</div>` : '') + body + '</div>';
}

/** Every number's definition, one click away, as the card's closing block.
 *
 * Measuring something is only useful if the reader knows exactly what was
 * measured. `openDetail` wires the button that unfolds it.
 */
export function formulasHtml(metrics, extra) {
  return '<h2>how these are measured</h2>'
    + '<button id="formula-toggle">show the formulas</button>'
    + '<div id="formulas" style="display:none">'
    + metrics.concat(extra || []).map(m =>
        `<div class="formula"><b>${m.label}</b> <span class="muted">`
        + `(${m.unit})</span><code>${m.formula}</code>`
        + `<div class="sub">${m.detail || m.help || ''}</div></div>`).join('')
    + '</div>';
}

const detailBox = document.getElementById('detail');

/** Show the detail card with this html; `onClose` runs when its x is clicked. */
export function openDetail(html, onClose) {
  detailBox.innerHTML = html;
  detailBox.classList.add('open');
  // The card sits directly on top of the colour key, however tall it ends up.
  const legend = document.getElementById('legend');
  detailBox.style.bottom = (legend.offsetHeight + 24) + 'px';
  document.getElementById('detail-close').onclick = onClose;
  const toggle = document.getElementById('formula-toggle');
  const box = document.getElementById('formulas');
  if (!toggle || !box) return;
  toggle.onclick = () => {
    const open = box.style.display === 'none';
    box.style.display = open ? 'block' : 'none';
    toggle.textContent = open ? 'hide the formulas' : 'show the formulas';
    detailBox.style.bottom = (document.getElementById('legend').offsetHeight + 24) + 'px';
  };
}

/** Hide and empty the detail card. */
export function closeDetail() {
  detailBox.classList.remove('open');
  detailBox.innerHTML = '';
}

// ---------------------------------------------------------------------------
// * Interaction
// ---------------------------------------------------------------------------

/** Make a left click on any of `pickable` call `onPick` with its `userData.bar`.
 *
 * `pickable` is read live, so a page that rebuilds its scene can empty and
 * refill the same array.
 */
export function wirePicking(pickable, onPick) {
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
    if (bar) onPick(bar);
  });

  // ? Right-click deliberately does NOT clear the selection: the right button is
  // ? how the view is moved, and a pan that ends without quite enough movement
  // ? would throw the card away. Esc and the card's x are the only two ways out.
  // ? (OrbitControls already stops the browser menu appearing on a right drag.)
}

/** Fill a picker with the metric list and run `onPick` when it changes. */
export function wirePicker(id, metrics, chosen, onPick) {
  const picker = document.getElementById(id);
  if (!picker) return;
  picker.innerHTML = metrics.map((m, i) =>
    `<option value="${i}"${m === chosen ? ' selected' : ''}>${m.label} (${m.unit})</option>`).join('');
  picker.onchange = () => onPick(Number(picker.value));
}

/** Each tick box shows or hides one group of things in the scene; the reset button runs `onReset`. */
export function wireLayers(onReset) {
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
      if (group === 'cameras' && box.checked) onReset();
    };
  }
  // ! Called with no argument: a click handler would hand onReset the mouse event.
  document.getElementById('reset').onclick = () => onReset();
}

/** Frame the view on the bars, widening to take in the mocap rig when shown.
 *
 * ! The cameras hang 2-6 m up and as much as 10 m out, so a view framed on the
 * ! bars alone leaves them off screen -- ticking them on would look like
 * ! nothing happened.
 *
 * @param {THREE.Object3D} target what to frame (default: the measured bars); a
 *   page whose measured bars can be absent frames its authored bars instead
 */
export function resetView(target = groups.bars) {
  const bounds = new THREE.Box3().setFromObject(target);
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
export function resize() {
  const w = host.clientWidth, h = host.clientHeight;
  if (!w || !h) return;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(host);

/** Start drawing: every frame sizes the labels to the view, then renders.
 *
 * A label in `groups.labels` whose `userData.muted` is set is hidden; the
 * numbers in `groups.notes` are only sized. `groups.notes` may also hold
 * shapes (a page's arrows): only its sprites are sized.
 */
export function startRenderLoop() {
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
    for (const label of groups.labels.children) {
      size(label);
      label.visible = groups.labels.visible && !label.userData.muted;
    }
    for (const label of groups.notes.children) if (label.isSprite) size(label);
    controls.update();
    renderer.render(scene, camera);
  });
}
