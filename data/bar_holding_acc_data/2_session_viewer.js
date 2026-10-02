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

let metric = DATA.metrics[0];
let selected = null;

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

/** The value a bar contributes to the current metric, or null if it has none. */
function valueOf(bar) {
  if (metric.key === 'load' && !bar.load_valid) return null;
  const v = bar[metric.key];
  return (v === null || v === undefined) ? null : v;
}

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
                    'cameras', 'labels']) {
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
    canvas.width = Math.ceil(ctx.measureText(text).width) + LABEL_SS * 6;
    sprite.userData.aspect = canvas.width / canvas.height;
    // Resizing the canvas clears it and resets the context, so set up again.
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

const pickable = [];
for (const bar of DATA.bars) {
  const material = new THREE.MeshStandardMaterial({ roughness: 0.55, metalness: 0.1 });
  const mesh = barMesh(bar.fitted[0], bar.fitted[1], BAR_SIDE, material);
  mesh.userData.bar = bar;
  groups.bars.add(mesh);
  pickable.push(mesh);

  // The authored pose as a thin outline, so the error reads as a visible gap.
  const ghost = barMesh(bar.authored[0], bar.authored[1], BAR_SIDE * 0.92,
    new THREE.MeshBasicMaterial({ color: 0x55606f, wireframe: true,
      transparent: true, opacity: 0.45 }));
  groups.authored.add(ghost);

  // The markers pick the same bar, so a click near one does the obvious thing.
  const markerGeometry = new THREE.SphereGeometry(MARKER_R, 10, 8);
  for (const point of bar.markers) {
    const dot = new THREE.Mesh(markerGeometry, material);
    dot.position.fromArray(point);
    dot.userData.bar = bar;
    groups.markers.add(dot);
    pickable.push(dot);
  }

  const label = labelSprite();
  label.position.fromArray(bar.fitted[1]).z += 0.12;
  groups.labels.add(label);
  bar._material = material;
  bar._label = label;
}

// Every robot's footprint, so all 20 parking spots read at once.
if (DATA.robot) {
  const [length, width] = DATA.robot.footprint;
  for (const bar of DATA.bars) {
    if (!bar.robot) continue;
    const base = new THREE.Matrix4().fromArray(bar.robot.base);
    // Just the outline of where the robot stood, flat on the floor -- an
    // unfilled rectangle reads as a footprint without hiding what is behind it.
    const half = [length / 2, width / 2];
    const outline = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(-half[0], -half[1], 0), new THREE.Vector3(half[0], -half[1], 0),
      new THREE.Vector3(half[0], half[1], 0), new THREE.Vector3(-half[0], half[1], 0),
      new THREE.Vector3(-half[0], -half[1], 0),
    ]);
    const rectangle = new THREE.Line(outline,
      new THREE.LineDashedMaterial({ color: FOOTPRINT,
        dashSize: 0.045, gapSize: 0.03 }));
    // ! A dashed line stays solid until the per-vertex distances are worked
    // ! out -- the material alone does nothing.
    rectangle.computeLineDistances();
    // A flat triangle just past the front edge says which way the robot faced.
    const nose = new THREE.Mesh(
      new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(half[0] + 0.22, 0, 0),
        new THREE.Vector3(half[0] + 0.04, 0.10, 0),
        new THREE.Vector3(half[0] + 0.04, -0.10, 0),
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
    bar._material.color.setHex(hex(colourFor(valueOf(bar))));
    // The label carries whichever measurement is being coloured, so the scene
    // answers "how much?" as well as "how bad?" without clicking anything.
    const value = valueOf(bar);
    bar._label.userData.draw(value === null
      ? `${bar.bar} · --`
      : `${bar.bar} · ${value.toFixed(decimalsFor(metric))} ${metric.unit}`);
  }
  drawLegend();
  if (selected) showDetail(selected);
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

const detailBox = document.getElementById('detail');

function showDetail(bar) {
  let html = '<button id="detail-close" title="unpin (Esc)">&times;</button>'
    + `<h2>bar ${bar.bar}</h2>`
    + `<div class="sub">${bar.file}${bar.run ? '<br>' + bar.run : ''}</div>`
    + '<h2>mocap</h2>'
    + row('placement', fmt(bar.placement, 2, 'mm'))
    + row('rotation', fmt(bar.rotation, 3, 'deg'))
    + row('fit residual', fmt(bar.fit_residual, 2, 'mm'))
    + row('bar length', fmt(bar.bar_length, 4, 'm'))
    + row('takes', `${bar.n_takes} (spread ${bar.placement_spread.toFixed(2)} mm)`);

  html += '<h2>servo, last iteration</h2>';
  if (bar.iterations) {
    html += row('tool0 left', fmt(bar.servo_left, 2, 'mm'))
      + row('tool0 right', fmt(bar.servo_right, 2, 'mm'))
      + row('rotation left', fmt(bar.rot_left, 3, 'deg'))
      + row('rotation right', fmt(bar.rot_right, 3, 'deg'))
      + row('iterations', bar.iterations);
  } else {
    html += '<div class="sub">no servo run matched to this bar</div>';
  }

  html += '<h2>load — is the bar being bent?</h2>';
  if (!bar.load_valid) {
    html += '<div class="sub warn">load not measured. The force sensor was '
      + 'zeroed while the bar was already gripped, so the bar\'s own load was '
      + 'tared away and this reading cannot say anything about bending.</div>'
      + row('left', fmt(bar.force_left, 2, 'N'))
      + row('right', fmt(bar.force_right, 2, 'N'));
  } else {
    const bending = bar.load > DATA.metrics[3].high;
    html += `<div class="sub">The gap between the two grippers is what the bar `
      + `absorbs sideways.</div>`
      + row('<b>imbalance</b>', `<b${bending ? ' class="warn"' : ''}>`
        + `${bar.load.toFixed(2)} N</b>`)
      + row('left', fmt(bar.force_left, 2, 'N'))
      + row('right', fmt(bar.force_right, 2, 'N'));
  }
  detailBox.innerHTML = html;
  detailBox.classList.add('open');
  // The card sits directly on top of the colour key, however tall it ends up.
  const legend = document.getElementById('legend');
  detailBox.style.bottom = (legend.offsetHeight + 24) + 'px';
  document.getElementById('detail-close').onclick = () => select(null);
}

function select(bar) {
  selected = bar;
  // The chosen bar glows so you can tell at a glance which one the panel is
  // describing, even after you have rotated away from it.
  for (const other of DATA.bars) {
    other._material.emissive.setHex(other === bar ? 0x33425c : 0x000000);
  }
  showArm(bar);
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

// Right-click on empty space clears, but a right-DRAG is OrbitControls panning.
renderer.domElement.addEventListener('contextmenu', e => {
  e.preventDefault();
  if (wasAClick(e)) select(null);
});

addEventListener('keydown', e => {
  if (e.key === 'Escape') select(null);
  // M cycles the colour metric without reaching for the dropdown.
  if (e.key === 'm' || e.key === 'M') {
    setMetric((DATA.metrics.indexOf(metric) + 1) % DATA.metrics.length);
  }
});

// * The same choice is offered in two places -- in the side panel and in the
// * legend, where the eye goes when asking what the colours mean. One handler
// * drives both and keeps them showing the same thing.
const metricPickers = ['metric', 'metric-legend'];

/** Switch which measurement the colours show. */
function setMetric(index) {
  metric = DATA.metrics[index];
  for (const id of metricPickers) {
    const picker = document.getElementById(id);
    if (picker) picker.value = String(index);
  }
  document.getElementById('metric-help').textContent = metric.help;
  recolour();
}

for (const id of metricPickers) {
  const picker = document.getElementById(id);
  if (!picker) continue;
  picker.innerHTML = DATA.metrics.map((m, i) =>
    `<option value="${i}">${m.label} (${m.unit})</option>`).join('');
  picker.onchange = () => setMetric(Number(picker.value));
}

for (const [id, group] of [['show-env', 'env'], ['show-authored', 'authored'],
    ['show-markers', 'markers'], ['show-robots', 'robots'],
    ['show-cameras', 'cameras'], ['show-labels', 'labels']]) {
  const box = document.getElementById(id);
  box.onchange = () => {
    groups[group].visible = box.checked;
    // Showing the rig for the first time also pulls the view out far enough to
    // see it, since it sits well outside the bars.
    if (group === 'cameras' && box.checked) resetView();
  };
}

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
document.getElementById('reset').onclick = resetView;

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
// The whole canvas is taller than the glyphs, so the sprite is scaled up by
// that ratio to land the TEXT at LABEL_TEXT_PX on screen.
const LABEL_SPRITE_PX = LABEL_TEXT_PX * (LABEL_CANVAS_H / LABEL_FONT_PX);

renderer.setAnimationLoop(() => {
  // How many world metres one screen pixel covers, one metre from the camera.
  // Derived from the camera and the canvas rather than guessed, so the labels
  // keep their size when the window is resized too.
  const viewportPx = renderer.domElement.clientHeight || 1;
  const spread = 2 * Math.tan((camera.fov * Math.PI / 180) / 2) / viewportPx;
  for (const bar of DATA.bars) {
    const distance = camera.position.distanceTo(bar._label.position);
    const height = LABEL_SPRITE_PX * spread * distance;
    bar._label.scale.set(height * bar._label.userData.aspect, height, 1);
    bar._label.visible = groups.labels.visible;
  }
  controls.update();
  renderer.render(scene, camera);
});
