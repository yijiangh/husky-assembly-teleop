// The browser half of 4_assembly_viewer.py: an assembly run, action by action.
// The scene, the drawing primitives, the legend, the card and the picking come
// from viewer_common.js; this file reads the json blob in #scene-data and, for
// the action the slider points at and the record chosen under it, draws:
//
//   - every bar that exists then, as a THIN line at its designed (authored) pose:
//     grey when placed, in the support robot's colour while it holds it, blue
//     for the action's own bar (at the step target, with its seat faint behind it);
//   - every bar MoCap measured in that record, see-through and a little thicker,
//     shaded by its distance to where it should be, with its balls sensed
//     (filled) and authored (hollow), both drawn at the real ball's size;
//   - the robots: the one that recorded, as measured, the others as last saved;
//   - the "data by" measurement drawn on the bars (showAnnotations): on every
//     measured bar, or on the clicked one only.
//
// The actions follow the plan skeleton's order. The record pill picked at an
// action stays picked when the user goes to another action and comes back.
//
// ! Read as text by viewer_common.py's viewer_script() and pasted after
// ! viewer_common.js into ONE module, with the import lines below dropped. So:
// ! keep each import on ONE line, and never declare a top-level name that
// ! viewer_common.js already declares (see the rules in its header).

import * as THREE from 'three';
import { groups, hex, css, MARKER_R, BAR_RADIUS, PICK_RADIUS, BAR_OPACITY, BAR_STEPS, NODATA, colourFor, vertexColour, addEnvironment, placeFloor, labelSprite, rodBetween, paintRod, footprintFor, addCameras, unpackLinkGeometries, armGroup, clearGroup, decimalsFor, drawLegend, fmt, row, strong, block, formulasHtml, openDetail, closeDetail, wirePicking, wirePicker, wireLayers, resetView, resize, startRenderLoop } from './viewer_common.js';

const DATA = JSON.parse(document.getElementById('scene-data').textContent);

// * What the page shows: the colour and label measurements, the action and the
// * record, the bar pinned in the card (by id, so it survives a step change),
// * and whether the pills list every record or only the milestones.
let metric = DATA.metrics[0];
let dataMetric = DATA.metrics[0];
let actionIndex = Math.max(0, DATA.actions.findIndex(a => a.records.length));
let recordIndex = -1;          // -1 = the action's default record
const chosenRecord = {};       // action index -> the record pill picked there (kept until another is picked)
let selectedId = null;
let showAllRecords = false;
let stripBar = '';             // '' = each action's worst bar, else one bar id

const metricByKey = key => DATA.metrics.find(m => m.key === key);
const placementMetric = metricByKey('placement');

// * The authored bars' colours: what a bar is at this action.
const ROLE_COLOUR = { placed: 0x55606f, active: 0x1763b8, Cindy: 0x6f9bd8, Alice: 0xc2410c, Belle: 0x7c3aed };
// * Bar thickness on this page: thin, so the few millimetres between the authored and the measured
// * bar show. Half of the shared BAR_RADIUS (2_session_viewer keeps that one); the authored bar thinner still.
const ROD_RADIUS = BAR_RADIUS / 2;
const SEAT_RADIUS = ROD_RADIUS * 0.72;
// * The arrows of an error are drawn this many times longer than the error itself: a millimetre on a
// * 2 m bar cannot be seen. The numbers written beside them are the true values.
const MAGNIFY = 20;
// The x / y / z axes drawn at a bar's start, metres.
const AXIS_LENGTH = 0.15;

// * Plain words for the record's fields.
const KIND_TEXT = { J: 'joint', R: 'release', H: 'hold', HR: 'hold release' };
const PHASE_TEXT = { start: 'at the start of the step', end: 'at the end of the step', manual: 'recorded by a click' };
// Where the balls came from, said only when it is not the bar's own Motive asset.
const SOURCE_TEXT = { unlabeled: 'Motive\'s unlabelled markers (the fallback)', fake: 'fake balls (desk rehearsal)',
  none: 'no balls' };

// * The drawn axes: x red, y green, z blue (z runs along the bar); a turn purple (not an axis colour);
// * a bow red; a placement gap dark.
const AXIS_COLOUR = [0xd03030, 0x1a8a3e, 0x1763b8];
const TURN_COLOUR = 0x9333ea;
const BOW_COLOUR = 0xd03030;
const GAP_COLOUR = 0x1d2430;
// The small balls marking the start / middle / end of the authored bar (dark) and of the measured one.
const POINT_COLOUR = 0x33425c;

// ---------------------------------------------------------------------------
// * Built once: the environment, the cameras, every bar's seat, the robot shapes
// ---------------------------------------------------------------------------

addEnvironment(DATA.env);
placeFloor(DATA.floor);
addCameras(DATA.cameras, DATA.cone);

// One thin rod per bar at its seat, kept for the whole page: a step only
// shows, hides and recolours them, it never rebuilds them. One continuous rod,
// so its start and end are where the bar's start and end are.
const seatRods = {};
for (const [id, seat] of Object.entries(DATA.assembled)) {
  const material = new THREE.MeshStandardMaterial({ color: ROLE_COLOUR.placed, roughness: 0.7, transparent: true });
  const rod = rodBetween(seat.start, seat.end, SEAT_RADIUS, material);
  rod.visible = false;
  groups.authored.add(rod);
  seatRods[id] = { rod, material };
}
// The per-step authored extras (the active bar at its step target) live in their own group.
const stepAuthored = new THREE.Group();
groups.authored.add(stepAuthored);

const linkGeometries = {};
for (const [model, packed] of Object.entries(DATA.robot_models || {})) {
  linkGeometries[model] = unpackLinkGeometries(packed.meshes);
}

// Ball shapes shared by every step (clearGroup leaves `shared` geometry alone). The sensed (filled)
// and the authored (hollow) ball are the same size, the real ball's (19 mm across), so a matched
// pair sits one inside the other.
const ballGeometry = new THREE.SphereGeometry(MARKER_R * 0.5, 12, 9);
ballGeometry.userData.shared = true;
const hollowGeometry = new THREE.SphereGeometry(MARKER_R * 0.5, 10, 7);
hollowGeometry.userData.shared = true;
// The small balls at the start / middle / end of a bar (placement error).
const pointGeometry = new THREE.SphereGeometry(0.005, 12, 9);
pointGeometry.userData.shared = true;

// ---------------------------------------------------------------------------
// * Which record, which value
// ---------------------------------------------------------------------------

const action = () => DATA.actions[actionIndex];

/** The records the pills list: the milestones (plus the last), or every one. */
function visibleRecords(a) {
  if (showAllRecords) return a.records.map((r, i) => i);
  const picked = a.records.map((r, i) => i).filter(i => a.records[i].milestone);
  const last = a.records.length - 1;
  if (last >= 0 && !picked.includes(last)) picked.push(last);
  return picked;
}

/** The record shown when none was picked: the last one that ended well. */
function defaultRecord(a) {
  for (let i = a.records.length - 1; i >= 0; i--) {
    const outcome = a.records[i].step.outcome;
    if (outcome === null || outcome === undefined || outcome === 'ok') return i;
  }
  return a.records.length - 1;
}

const currentRecord = () => {
  const a = action();
  if (!a.records.length) return null;
  return a.records[recordIndex >= 0 ? recordIndex : defaultRecord(a)];
};

/** One measured bar's value for a measurement, or null. Robot numbers belong to a held bar only. */
function valueFor(bar, which) {
  const record = bar._record;
  let v;
  if (which.key === 'tool0') v = bar.held_by ? record.tool0_worst : null;
  else if (which.key === 'servo') v = bar.held_by && record.servo ? record.servo.worst : null;
  else if (which.key === 'load') v = bar.held_by && record.load ? record.load.imbalance : null;
  else v = bar[which.key];
  return (v === null || v === undefined || Number.isNaN(v)) ? null : v;
}
const valueOf = bar => valueFor(bar, metric);

/** The number one measurement writes on a bar, or "--". */
function labelText(bar, which) {
  const value = valueFor(bar, which);
  return value === null ? '--' : `${value.toFixed(decimalsFor(which))} ${which.unit}`;
}

/** The point a fraction t along a measured bar (0 its start, 1 its end). */
const alongBar = (bar, t) => new THREE.Vector3().fromArray(bar.fitted[0]).lerp(new THREE.Vector3().fromArray(bar.fitted[1]), t);

/** A solid arrow (a thin rod and a cone), as real geometry so it keeps its thickness at any zoom.
 *
 * @param {THREE.Vector3} origin where it starts
 * @param {THREE.Vector3} direction which way it points (any length)
 * @param {number} length metres, head included
 * @param {number} colour hex colour
 * @param {number} radius the shaft's radius, metres
 * @param {number} opacity 1 solid, lower to see through
 * @returns {THREE.Group} the arrow
 */
function arrowMesh(origin, direction, length, colour, radius = 0.003, opacity = 1) {
  const dir = direction.clone().normalize();
  const material = new THREE.MeshBasicMaterial({ color: colour, transparent: opacity < 1, opacity, depthWrite: opacity === 1 });
  // The head keeps a visible size however thin the shaft (the rotation axes are under a millimetre).
  const head = Math.min(length * 0.3, Math.max(radius * 8, 0.012));
  const group = new THREE.Group();
  group.add(rodBetween(origin.toArray(), origin.clone().addScaledVector(dir, length - head).toArray(), radius, material));
  const cone = new THREE.Mesh(new THREE.ConeGeometry(Math.max(radius * 2.6, 0.003), head, 12), material);
  cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
  cone.position.copy(origin).addScaledVector(dir, length - head / 2);
  group.add(cone);
  return group;
}

/** A frame's x / y / z axes drawn as three arrows from its origin.
 *
 * @param {Array} frame16 the frame, 16 numbers column by column
 * @param {number} length metres
 * @param {number} radius the shafts' radius, metres
 * @param {number} opacity 1 solid, lower for the reference frame
 * @param {boolean|Array} named write x / y / z at the tips, or these three texts instead
 * @returns {Array} the shapes, to add to a group
 */
function triad(frame16, length, radius, opacity, named) {
  const m = new THREE.Matrix4().fromArray(frame16);
  const origin = new THREE.Vector3().setFromMatrixPosition(m);
  const out = [];
  for (let i = 0; i < 3; i++) {
    const axis = new THREE.Vector3().setFromMatrixColumn(m, i).normalize();
    out.push(arrowMesh(origin, axis, length, AXIS_COLOUR[i], radius, opacity));
    if (named) {
      const label = labelSprite();
      label.userData.draw(Array.isArray(named) ? named[i] : 'xyz'[i]);
      label.position.copy(origin).addScaledVector(axis, length + 0.03);
      // The z axis runs along the bar: its label goes above the bar, not on it.
      if (i === 2) label.position.z += 0.04;
      out.push(label);
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// * One step
// ---------------------------------------------------------------------------

let measured = [];      // the solid bars of the shown record
const pickable = [];

/** Draw the chosen action and record: the authored bars by role, the measured bars, their balls, the robots. */
function buildStep() {
  const a = action();
  const record = currentRecord();
  for (const group of [groups.bars, groups.markers, groups.robots, groups.arm, groups.labels, groups.notes, stepAuthored]) {
    clearGroup(group);
  }
  pickable.length = 0;
  measured = [];

  // * Every bar that exists at this action, dashed at its seat, coloured by its role.
  const activeBar = record ? record.bars.find(b => b.bar === a.bar_id) : null;
  for (const [id, seat] of Object.entries(seatRods)) {
    const exists = a.exists.includes(id);
    seat.rod.visible = exists;
    if (!exists) continue;
    const holder = a.held[id];
    const role = id === a.bar_id ? 'active' : (holder ? holder : 'placed');
    seat.material.color.setHex(ROLE_COLOUR[role] || ROLE_COLOUR.placed);
    // The action's bar, when it should be somewhere else than its seat now
    // (held above it before the insert): the seat goes faint, the target is drawn.
    const away = role === 'active' && activeBar && activeBar.reference
      && activeBar.reference_kind && activeBar.reference_kind.startsWith('held_authored')
      && activeBar.placement_seat !== undefined && Math.abs((activeBar.placement_seat || 0) - (activeBar.placement || 0)) > 0.5;
    seat.material.opacity = away ? 0.3 : 1.0;
    if (away) {
      stepAuthored.add(rodBetween(activeBar.reference[0], activeBar.reference[1], SEAT_RADIUS,
        new THREE.MeshStandardMaterial({ color: ROLE_COLOUR.active, roughness: 0.7 })));
    }
    if (role !== 'placed') {
      const label = labelSprite();
      label.userData.draw(role === 'active' ? `${id} · ${a.kind} (${a.robot})` : `${id} · held by ${holder}`);
      // At the bar's middle: a tall bar's far end is often out of the view. High above it, so the
      // "data by" labels (about 9 cm above and below the bar) fit under it.
      label.position.fromArray(DATA.assembled[id].start)
        .lerp(new THREE.Vector3().fromArray(DATA.assembled[id].end), 0.5).z += 0.17;
      label.userData.tag = id;
      groups.labels.add(label);
    }
  }

  // * The bars MoCap measured in this record, see-through and shaded.
  for (const bar of record ? record.bars : []) {
    bar._record = record;
    const reference = bar.reference || bar.seat;
    if (bar.ok && bar.fitted) {
      const material = new THREE.MeshStandardMaterial({ roughness: 0.5, metalness: 0.1, vertexColors: true,
        transparent: true, opacity: record.step.outcome && record.step.outcome !== 'ok' ? BAR_OPACITY * 0.5 : BAR_OPACITY,
        depthWrite: false });
      const rod = rodBetween(bar.fitted[0], bar.fitted[1], ROD_RADIUS, material, BAR_STEPS);
      rod.geometry.setAttribute('color', new THREE.BufferAttribute(
        new Float32Array(rod.geometry.getAttribute('position').count * 3), 3));
      groups.bars.add(rod);
      bar._rod = rod;
      bar._gapStart = new THREE.Vector3().fromArray(bar.fitted[0]).sub(new THREE.Vector3().fromArray(reference[0]));
      bar._gapEnd = new THREE.Vector3().fromArray(bar.fitted[1]).sub(new THREE.Vector3().fromArray(reference[1]));
      bar._length = new THREE.Vector3().fromArray(bar.fitted[0]).distanceTo(new THREE.Vector3().fromArray(bar.fitted[1]));
      const sleeve = rodBetween(bar.fitted[0], bar.fitted[1], PICK_RADIUS, new THREE.MeshBasicMaterial({ visible: false }));
      sleeve.userData.bar = bar;
      groups.bars.add(sleeve);
      pickable.push(sleeve);
      const label = labelSprite();
      label.position.fromArray(bar.fitted[1]).z += 0.12;
      groups.labels.add(label);
      bar._label = label;
      measured.push(bar);
    }
    // * Its balls: sensed filled (red when stray, grey when Motive's reading was
    // * dropped), authored hollow (red when no ball matched it).
    bar._ballMaterial = new THREE.MeshStandardMaterial({ roughness: 0.55 });
    const stray = new THREE.MeshStandardMaterial({ color: 0xd03030, roughness: 0.55 });
    const dropped = new THREE.MeshStandardMaterial({ color: 0xa8b0bc, roughness: 0.55, transparent: true, opacity: 0.6 });
    for (const ball of bar.markers_sensed || []) {
      const dot = new THREE.Mesh(ballGeometry, ball.state === 'matched' ? bar._ballMaterial
        : (ball.state === 'stray' ? stray : dropped));
      dot.position.fromArray(ball.pos);
      dot.userData.bar = bar;
      groups.markers.add(dot);
      pickable.push(dot);
    }
    const hollowSeen = new THREE.MeshBasicMaterial({ color: 0x33425c, wireframe: true, transparent: true, opacity: 0.55 });
    const hollowMissing = new THREE.MeshBasicMaterial({ color: 0xd03030, wireframe: true });
    for (const ball of bar.markers_authored || []) {
      const ring = new THREE.Mesh(hollowGeometry, ball.seen ? hollowSeen : hollowMissing);
      ring.position.fromArray(ball.pos);
      groups.markers.add(ring);
    }
  }

  // * The robots: the one that recorded as measured, the others as last saved.
  for (const robot of record ? record.robots : []) {
    const model = DATA.robot_models[robot.model];
    if (!model) continue;
    const foot = footprintFor(robot.base, model.footprint, null);
    groups.robots.add(foot);
    groups.robots.add(armGroup(robot.links, linkGeometries[robot.model], robot.live ? 0.62 : 0.28));
    const label = labelSprite();
    label.userData.draw(`${robot.name}${robot.live ? '' : ' (saved)'}`);
    label.position.set(robot.base[12], robot.base[13], robot.base[14] + 1.25);
    groups.labels.add(label);
  }

  recolour();
  drawControls();
  drawStatus();
  select(selectedId);
}

// ---------------------------------------------------------------------------
// * Colour, labels, the card
// ---------------------------------------------------------------------------

/** Recolour the measured bars and their balls by the colour measurement; redraw the legend. */
function recolour() {
  for (const bar of measured) {
    paintRod(bar, metric.key === 'placement' ? null : vertexColour(valueOf(bar), metric.high), metric.high);
    bar._ballMaterial.color.setHex(hex(colourFor(valueOf(bar), metric.high)));
  }
  relabel();
  drawLegend(metric, measured.map(valueOf),
    measured.filter(b => { const v = valueOf(b); return v !== null && v > metric.high; }).map(b => b.bar));
  if (selectedId) showDetail(selectedId);
}

/** Write the "data by" measurement on every measured bar's name label, unless the view draws it on the bar
 * itself (DRAWN_ON_BAR); the selected bar keeps only its name. */
function relabel() {
  for (const bar of measured) {
    const plain = bar.bar === selectedId || dataMetric.key in DRAWN_ON_BAR || DRAWN_AT_FLANGES.includes(dataMetric.key);
    bar._label.userData.draw(plain ? bar.bar : `${bar.bar} · ${labelText(bar, dataMetric)}`);
  }
}

const mm = (v, d = 2) => fmt(v, d, 'mm');
const deg = (v, d = 3) => fmt(v, d, 'deg');
const devRow = (name, dev) => row(name, dev ? `${dev.dpos_mm.toFixed(2)} mm · ${dev.dang_deg.toFixed(3)} deg`
                                            : '<span class="muted">--</span>');
const MILESTONE_TEXT = {
  before_insert: 'before the insert', after_insert: 'after the insert', after_fix: 'after the foundation fix',
  before_retreat: 'before the retreat', after_retreat: 'after the retreat', hold_approach: 'support robot at its approach',
  hold_at_grasp: 'support gripper around the bar', hold_grasped: 'support gripper closed',
  hold_released: 'support gripper opened', hold_retreated: 'support robot backed off', bystander: 'standing by',
};

/** The bar's every measurement this day since it was placed, as a small line chart and a list. */
function historyBlock(id, record) {
  const { entries, skipped } = DATA.history[id] || { entries: [], skipped: 0 };
  // * Records from before the bar was placed (on its loading spot, standing by) are left out
  // * (4_assembly_viewer.add_history): metres from the seat, they would flatten the chart.
  const left = skipped ? `<div class="sub">${skipped} earlier record${skipped === 1 ? '' : 's'} left out: the bar `
    + 'was not placed yet (on its loading spot, or standing by)</div>' : '';
  if (!entries.length) return left ? block('history', 'history of this bar', '', left, dataMetric.key) : '';
  const values = entries.map(e => e.placement_seat).filter(v => v !== null && v !== undefined);
  const top = Math.max(1, ...values);
  const W = 300, H = 46;
  const x = i => entries.length === 1 ? W / 2 : 6 + (W - 12) * i / (entries.length - 1);
  const y = v => H - 6 - (H - 12) * Math.min(1, v / top);
  const points = entries.map((e, i) => (e.placement_seat === null || e.placement_seat === undefined)
    ? null : `${x(i).toFixed(1)},${y(e.placement_seat).toFixed(1)}`).filter(Boolean).join(' ');
  const dots = entries.map((e, i) => {
    if (e.placement_seat === null || e.placement_seat === undefined) return '';
    const here = e.action === actionIndex && action().records[e.record] === record;
    return `<circle cx="${x(i).toFixed(1)}" cy="${y(e.placement_seat).toFixed(1)}" r="${here ? 4 : 2.5}"`
      + ` fill="${here ? '#1d2430' : css(colourFor(e.placement_seat, placementMetric.high))}"/>`;
  }).join('');
  const svg = `<svg width="${W}" height="${H}" style="display:block;margin:4px 0"><polyline points="${points}"`
    + ` fill="none" stroke="#9aa6b6" stroke-width="1.2"/>${dots}</svg>`
    + `<div class="sub">up: distance from the seat, the worst of start / middle / end (0 – ${top.toFixed(1)} mm) · `
    + 'across: the records in time order</div>';
  // * How far the bar moved since the robot let go of it (its first record after the retreat),
  // * for the records after that one only.
  const releasedAt = entries.findIndex(e => e.milestone === 'after_retreat');
  const released = releasedAt >= 0 ? entries[releasedAt] : null;
  const moved = e => (!released || !e.fitted || !released.fitted) ? null : Math.max(
    new THREE.Vector3().fromArray(e.fitted[0]).distanceTo(new THREE.Vector3().fromArray(released.fitted[0])),
    new THREE.Vector3().fromArray(e.fitted[1]).distanceTo(new THREE.Vector3().fromArray(released.fitted[1]))) * 1000;
  const three = e => [e.seat_start, e.seat_middle, e.seat_end]
    .map(v => (v === null || v === undefined) ? '--' : v.toFixed(2)).join(' / ');
  const rows = entries.map((e, i) => row(
    `${e.time} · ${e.kind}${(e.kind === 'H' || e.kind === 'HR') ? ' by ' + e.robot : ''}`
      + `${e.milestone && e.milestone !== 'bystander' ? ' · <b>' + (MILESTONE_TEXT[e.milestone] || e.milestone) + '</b>' : ''}`,
    `${three(e)} mm${e.bend !== null && e.bend !== undefined ? ' · bow ' + e.bend.toFixed(2) : ''}`
    + (i > releasedAt && releasedAt >= 0 && moved(e) !== null ? ` · moved ${moved(e).toFixed(2)} since the retreat` : '')))
    .join('');
  return block('history', 'history of this bar',
    'at every record since it was placed: its distance from the seat at its start / middle / end (mm), its '
    + 'largest bow, and after the robot let go, how far its ends moved since',
    svg + left + rows, dataMetric.key);
}

/** The bar's shape and fit, from its balls: one block, ending with the verdict.
 *
 * * A plate (one T20 MoCap joint) carries two balls a known distance apart. That
 * * distance cannot change when the bar bends, so a wrong one is a bad ball. The
 * * distances between plates and the bow use the seen balls only; the fit's
 * * numbers compare every ball with its design spot after the best rigid fit, so
 * * they mix a bad ball, a plate off the table and bending. The verdict
 * * (4_assembly_viewer.ball_findings) tells them apart over the whole day.
 *
 * @param {object} bar the measured bar
 * @param {function} asked whether the "data by" picker asks for a measurement
 * @returns {string} the block's html
 */
function shapeBlock(bar, asked) {
  const short = joint => joint.replace(/^joint_/, '').replace(/_mocap$/, '');
  const signed = v => `${v >= 0 ? '+' : ''}${v.toFixed(2)}`;
  const day = (DATA.ball_check || {})[bar.bar] || null;
  const joints = (bar.pairs || {}).joints || [];
  let body = '<div class="sub">unit: mm · measured / design (off by)</div>';
  // * Each plate: its two balls' distance against the design.
  const worstPair = Math.max(0, ...joints.filter(j => j.length_err_mm !== null).map(j => Math.abs(j.length_err_mm)));
  for (const j of joints) {
    body += row(short(j.joint), j.length_mm === null ? `only ${j.n_seen} of its 2 balls seen`
      : strong(`${j.length_mm.toFixed(1)} / ${(j.length_mm - j.length_err_mm).toFixed(1)} (off ${signed(j.length_err_mm)})`,
        asked('pair_length') && Math.abs(j.length_err_mm) === worstPair));
  }
  // * The distance between neighbouring plates (their midpoints), against the design.
  const gaps = (bar.pairs || {}).spacing || [];
  const worstGap = Math.max(0, ...gaps.map(g => Math.abs(g.err_mm)));
  for (const s of gaps) {
    body += row(`${short(s.from)} → ${short(s.to)}`,
      strong(`${s.mm.toFixed(1)} / ${(s.mm - s.err_mm).toFixed(1)} (off ${signed(s.err_mm)})`,
        asked('spacing') && Math.abs(s.err_mm) === worstGap));
  }
  // * The bow at every inner plate (off the straight line through the outer two), the largest in bold.
  const bowed = joints.filter(j => j.bend_mm !== null);
  if (bowed.length) {
    const largest = Math.max(...bowed.map(j => j.bend_mm));
    body += row('bow', bowed.map(j => strong(`${short(j.joint)}: ${j.bend_mm.toFixed(2)}`, j.bend_mm === largest)).join(', '));
  }
  if (day && day.noise_mm !== null && day.noise_mm !== undefined) {
    body += row('Motive noise this day', `±${day.noise_mm.toFixed(2)} mm <span class="muted">(how much a plate's ball `
      + `distance wanders between ${day.n_records} records)</span>`);
  }
  // * The fit: every seen ball against its design spot, after the best rigid placement of the whole bar.
  body += row('balls found', `${bar.n_matched} of ${bar.n_design}`)
    + row('typical mismatch (rms)', mm(bar.fit_rms))
    + '<div class="sub">the root-mean-square distance between each seen ball and its design spot, after the '
    + 'best rigid fit of the whole bar</div>'
    + row('worst ball', strong(mm(bar.fit_residual), asked('fit_residual')))
    + row('hidden balls <span class="muted">(design balls nothing matched)</span>',
      bar.missing.length ? bar.missing.join(', ') : 'none')
    + row('extra balls near the bar <span class="muted">(seen, matched nothing)</span>', String(bar.strays));
  if (bar.source && bar.source !== 'asset') body += `<div class="sub warn">balls from ${SOURCE_TEXT[bar.source] || bar.source}</div>`;
  if (bar.ambiguous) body += '<div class="sub warn">another assignment of the balls fits as well</div>';
  // * The verdict.
  const good = (bar.findings || []).length === 1 && bar.findings[0].startsWith('balls good');
  body += '<div class="sub" style="margin-top:6px"><b>verdict</b></div>'
    + (bar.findings || []).map(line => `<div class="sub${good ? '' : ' warn'}">• ${line}</div>`).join('');
  const key = ['pair_length', 'spacing', 'fit_residual'].find(asked) || 'bend';
  return block(key, 'bar shape & fit', 'from its balls: is a ball off, is the bar bent, does the fit hold?',
    body, dataMetric.key);
}

/** Open the card of one bar in the shown record (its history even when it was not measured there). */
function showDetail(id) {
  const record = currentRecord();
  const bar = record ? record.bars.find(b => b.bar === id) : null;
  const asked = key => dataMetric.key === key;
  let html = '<button id="detail-close" title="unpin (Esc)">&times;</button>' + `<h2>bar ${id}</h2>`;
  if (!bar) {
    html += '<div class="sub">not measured in this record</div>' + historyBlock(id, record);
    openDetail(html, () => select(null));
    return;
  }
  const outcome = record.step.outcome;
  // * Which step and robot, then when the record was taken and what moment of the bar's life it is.
  html += `<div class="sub"><b>${record.step.movement_id || 'a click'} / ${record.robot}</b> (${record.time})<br>`
    + `${PHASE_TEXT[record.step.phase] || record.step.phase} · ${MILESTONE_TEXT[bar.milestone] || 'no milestone'}</div>`;
  if (outcome && outcome !== 'ok') {
    html += `<div class="flag">This step ended <b>${outcome}</b>: nothing was carried forward from it.</div>`;
  }
  if (!bar.ok) {
    html += `<div class="flag"><b>The fit failed:</b> ${bar.reason}</div>`;
  } else {
    // * The three places along the bar, the largest in bold (also written on the bar in the view).
    const threeRows = values => {
      const known = values.filter(v => v !== null && v !== undefined);
      const largest = known.length ? Math.max(...known) : null;
      return ['at the start', 'at the middle', 'at the end']
        .map((name, i) => row(name, strong(mm(values[i]), values[i] === largest))).join('');
    };
    html += block('placement', 'placement error', 'MoCap bar vs where it should be',
      threeRows([bar.tip_start, bar.tip_middle, bar.tip_end]), dataMetric.key);
    html += block('placement_seat', 'placement error vs the seat',
      'MoCap bar vs its final place in the design (its seat). The same as the placement error once the bar is '
      + 'placed; they differ only while it is held on its way in (before the insert it should be 15 mm above '
      + 'its seat)',
      threeRows([bar.seat_start, bar.seat_middle, bar.seat_end]), dataMetric.key);
    const dev = bar.vs_reference;
    const turn = dev ? dev.rotvec_bar_deg : null;
    html += block('rotation', 'rotation error', 'how the measured bar is turned against where it should be',
      row('angle between the centre lines', strong(deg(bar.rotation), asked('rotation')))
      + row('the same, vs the seat', deg(bar.rotation_seat))
      + (turn ? row('tilt about the bar\'s x <span class="muted">(red)</span>', deg(turn[0]))
        + row('tilt about the bar\'s y <span class="muted">(green)</span>', deg(turn[1]))
        + row('roll about the bar\'s own axis z <span class="muted">(blue)</span>', deg(turn[2]))
        + row('offset at its start: x / y across, z along', dev.dpos_bar_mm.map(v => v.toFixed(2)).join(' / ') + ' mm')
        : '')
      + '<div class="sub">One value for the whole bar: it is rigid, so it turns as one piece and the angle is the '
      + 'same wherever it is measured. It has no anchor: the axes are drawn at the bar\'s start only because the '
      + 'bar\'s own frame sits there. In the view (data by: rotation error): faint axes = where it should be, solid '
      + 'axes = the measured bar, each with its letter (X / Y / Z) at its tip and the turn about it just above '
      + '(about X / Y / Z); around each '
      + `axis near its tip, a purple arc sweeps that turn (${MAGNIFY}× the true turn, none when too small to see); `
      + 'the angle between the centre lines is written under the z axis.</div>', dataMetric.key);
    // * Each flange by FK against the step's target, the servo's last iteration, then the error split:
    // * what the robot can see (FK) and what it cannot (MoCap vs FK).
    let robot = '';
    for (const [side, t] of Object.entries(record.tool0 || {})) {
      robot += row(`tool0 vs target, ${side}`, strong(t.fk_vs_target
        ? `${t.fk_vs_target.dpos_mm.toFixed(2)} mm · ${t.fk_vs_target.dang_deg.toFixed(3)} deg` : '--', asked('tool0')));
    }
    const servo = record.servo;
    if (servo) {
      for (const [side, s] of Object.entries(servo.arms)) {
        robot += row(`servo residual, ${side}`, strong(`${mm(s.pos_norm_mm)} · ${deg(s.rot_max_deg)}`, asked('servo')));
      }
      robot += row('servo', `${servo.iterations} iterations (last one above) · aims ${servo.feedback}`)
        + `<div class="sub">${servo.file}${servo.note ? '<br>' + servo.note : ''}</div>`;
    } else {
      robot += '<div class="sub">servo residual: no servo ran inside this step (only the end records of a transfer '
        + 'and of a grounded bar\'s insert have one)</div>';
    }
    if (bar.fk) {
      robot += devRow('FK bar vs target <span class="muted">(robot sees)</span>', bar.fk.fk_vs_reference)
        + devRow('MoCap vs target <span class="muted">(total)</span>', bar.fk.mocap_vs_reference)
        + devRow('MoCap vs FK <span class="muted">(robot blind)</span>', bar.fk.mocap_vs_fk);
    } else {
      robot += `<div class="sub">${bar.held_by ? 'no tool0 FK in this record' + (record.fk_note ? ': ' + record.fk_note : '')
        : 'no arm holds this bar now'}</div>`;
    }
    html += block(asked('servo') ? 'servo' : 'tool0', 'robot: tool0 vs target, servo residual',
      'where FK puts each flange against the step\'s target, the servo\'s last iteration, then the bar as the '
      + 'robot believes it is (tool0 FK with the grasp) against MoCap', robot, dataMetric.key);
    html += shapeBlock(bar, asked);
    const load = record.load;
    const zero = load && load.zero
      ? `force sensors zeroed at ${(load.zero.at || '').slice(11, 19)} ${load.zero.note || ''}`
      : 'relative to the sensors\' last zero (not saved in records before 2026-10-21)';
    html += block('load', 'load', 'Cindy\'s two wrists (force magnitudes)', load
      ? row('left', fmt(load.left_N, 2, 'N')) + row('right', fmt(load.right_N, 2, 'N'))
        + row('<b>gap</b> <span class="muted">(| left − right |, the load imbalance)</span>',
          strong(fmt(load.imbalance, 2, 'N'), asked('load')))
        + `<div class="sub">${zero}</div>`
      : '<div class="sub">no reading</div>', dataMetric.key);
  }
  html += historyBlock(id, record);
  html += formulasHtml(DATA.metrics, DATA.extra_formulas.map(([label, formula]) => ({ label, unit: '', formula })));
  openDetail(html, () => select(null));
}

/** A label in the view, with the "data by" drawings (cleared with them). */
function note(text, position) {
  const label = labelSprite();
  label.userData.draw(text);
  label.position.copy(position);
  groups.notes.add(label);
}

/** A small ball marking one point of a bar. */
function pointBall(position, colour) {
  const ball = new THREE.Mesh(pointGeometry, new THREE.MeshBasicMaterial({ color: colour }));
  ball.position.copy(position);
  groups.notes.add(ball);
}

/** The gap between two points as an arrow from `from` towards `to`, MAGNIFY times longer (1 cm at least). */
function gapArrow(from, to, colour) {
  const gap = to.clone().sub(from);
  if (gap.length() < 1e-7) return;
  groups.notes.add(arrowMesh(from, gap, Math.max(0.01, MAGNIFY * gap.length()), colour, 0.0018));
}

/** A curved arrow turning about an axis: from `start` by `angle` radians (right-hand rule), `radius` from `centre`.
 *
 * @param {THREE.Vector3} centre the arc's centre
 * @param {THREE.Vector3} axis the axis it turns about (unit)
 * @param {THREE.Vector3} start where the arc starts, a unit vector across the axis
 * @param {number} angle radians, signed
 * @param {number} radius metres
 * @param {number} colour hex colour
 * @returns {THREE.Vector3} the arc's middle point, for its label
 */
function arcArrow(centre, axis, start, angle, radius, colour) {
  const n = Math.max(4, Math.ceil(Math.abs(angle) / 0.05));
  const points = [];
  for (let k = 0; k <= n; k++) {
    points.push(centre.clone().addScaledVector(start.clone().applyAxisAngle(axis, angle * k / n), radius));
  }
  const material = new THREE.MeshBasicMaterial({ color: colour });
  groups.notes.add(new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(points), n * 2, 0.001, 6), material));
  // * Its head, pointing on along the turn (never longer than half the arc).
  const tangent = points[n].clone().sub(points[n - 1]).normalize();
  const head = Math.min(0.014, 0.5 * radius * Math.abs(angle));
  const cone = new THREE.Mesh(new THREE.ConeGeometry(0.005, head, 12), material);
  cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), tangent);
  cone.position.copy(points[n]).addScaledVector(tangent, head / 2);
  groups.notes.add(cone);
  return points[Math.floor(n / 2)];
}

/** Placement: a ball at the start, middle and end of the authored bar and of the measured one, an arrow
 * from each authored point towards its measured point (MAGNIFY times longer) and the true distance.
 *
 * @param {object} bar the measured bar
 * @param {Array|null} authored the authored bar's two ends (the step target, or the seat)
 * @param {Array} values the distances at the start, middle and end, mm
 */
function drawPlacement(bar, authored, values) {
  if (!authored) return;   // no target at this step (e.g. before the mount)
  [0, 0.5, 1].forEach((t, i) => {
    const target = new THREE.Vector3().fromArray(authored[0]).lerp(new THREE.Vector3().fromArray(authored[1]), t);
    const real = alongBar(bar, t);
    pointBall(target, POINT_COLOUR);
    pointBall(real, hex(colourFor(values[i], dataMetric.high)));
    gapArrow(target, real, GAP_COLOUR);
    if (values[i] !== null && values[i] !== undefined) note(`${values[i].toFixed(2)} mm`, real.clone().setZ(real.z - 0.07));
  });
}

/** Rotation: the authored bar's x / y / z axes at its start (faint) and the measured bar's (solid), each
 * measured axis's tip labelled with the turn about it; around each axis near its tip a purple arc sweeping
 * MAGNIFY times that turn; and the angle between the two centre lines under the z axis.
 *
 * @param {object} bar the measured bar
 */
function drawRotation(bar) {
  if (!bar.frame_fit || !bar.frame_ref) return;
  const turn = bar.turn_ref_deg || [0, 0, 0];
  // * Thin axes (under a millimetre) so they stay apart from the bars themselves. The number at an
  // * axis's tip is the turn ABOUT that axis.
  for (const shape of triad(bar.frame_ref, AXIS_LENGTH, 0.0004, 0.4, false)) groups.notes.add(shape);
  const names = [0, 1, 2].map(i => 'XYZ'[i]);
  const shapes = triad(bar.frame_fit, AXIS_LENGTH, 0.0006, 1, names);
  for (const shape of shapes) groups.notes.add(shape);
  // * Above each axis's letter, the turn about that axis.
  shapes.filter(shape => shape.isSprite).forEach((letter, i) => {
    note(`about ${'XYZ'[i]}: ${turn[i].toFixed(2)}°`, letter.position.clone().setZ(letter.position.z + 0.035));
  });
  const ref = new THREE.Matrix4().fromArray(bar.frame_ref);
  const origin = new THREE.Vector3().setFromMatrixPosition(ref);
  const axes = [0, 1, 2].map(i => new THREE.Vector3().setFromMatrixColumn(ref, i).normalize());
  // * Which way each arc starts from its axis: about x upwards (y), about y sideways (x), about z upwards (y).
  const START = [1, 0, 1];
  for (let i = 0; i < 3; i++) {
    // At most a near full turn, however large the magnified angle; none when too small to see.
    const sweep = Math.max(-5.8, Math.min(5.8, MAGNIFY * turn[i] * Math.PI / 180));
    if (Math.abs(sweep) > 0.05) {
      arcArrow(origin.clone().addScaledVector(axes[i], AXIS_LENGTH * 0.8), axes[i], axes[START[i]], sweep, 0.03, TURN_COLOUR);
    }
  }
  // * The angle between the two centre lines (the authored and the measured z), under the z axis's tip.
  const between = bar.rotation ?? bar.rotation_seat;
  if (between !== null && between !== undefined) {
    note(`centre lines ${between.toFixed(3)}° apart`,
      origin.clone().addScaledVector(axes[2], AXIS_LENGTH).add(new THREE.Vector3(0, 0, -0.06)));
  }
}

/** A plate's short name, e.g. ``joint_J20-22_mocap`` -> ``J20-22``. */
const plateName = joint => joint.replace(/^joint_/, '').replace(/_mocap$/, '');

/** Where a plate sits on the measured bar (on its centre line, at the plate's distance along it). */
const plateAt = (bar, j) => alongBar(bar, j.z_mm / (DATA.assembled[bar.bar].length * 1000));

/** A plate's label point: below the bar for the 1st, 3rd, ... plate, above it for the others, so the
 * labels of neighbouring plates never sit side by side. */
const plateLabelAt = (point, k) => point.clone().setZ(point.z + (k % 2 ? 0.09 : -0.11));

/** Bending: every plate named; at each inner plate an arrow the way the bar bows there (MAGNIFY times) and the bow.
 *
 * @param {object} bar the measured bar
 */
function drawBow(bar) {
  if (!bar.frame_fit) return;
  const m = new THREE.Matrix4().fromArray(bar.frame_fit);
  const x = new THREE.Vector3().setFromMatrixColumn(m, 0).normalize();
  const y = new THREE.Vector3().setFromMatrixColumn(m, 1).normalize();
  ((bar.pairs || {}).joints || []).forEach((j, k) => {
    const point = plateAt(bar, j);
    if (j.bow_mm && j.bend_mm) {
      const dir = x.clone().multiplyScalar(j.bow_mm[0]).addScaledVector(y, j.bow_mm[1]).normalize();
      groups.notes.add(arrowMesh(point, dir, Math.max(0.01, MAGNIFY * j.bend_mm / 1000), BOW_COLOUR, 0.0025));
    }
    note(`${plateName(j.joint)}${j.bend_mm ? ` · bow ${j.bend_mm.toFixed(2)} mm` : ''}`, plateLabelAt(point, k));
  });
}

/** Plate length: at each plate, its name and how far its two balls' distance is off the design.
 *
 * @param {object} bar the measured bar
 */
function drawPairs(bar) {
  ((bar.pairs || {}).joints || []).forEach((j, k) => {
    const off = j.length_err_mm === null ? `${j.n_seen} of 2 balls seen`
      : `balls off ${j.length_err_mm >= 0 ? '+' : ''}${j.length_err_mm.toFixed(2)} mm`;
    note(`${plateName(j.joint)} · ${off}`, plateLabelAt(plateAt(bar, j), k));
  });
}

/** Spacing: a small ball at each plate on the bar, and between each two neighbouring plates their distance,
 * measured / design (off by), mm.
 *
 * @param {object} bar the measured bar
 */
function drawSpacing(bar) {
  const joints = (bar.pairs || {}).joints || [];
  const byJoint = Object.fromEntries(joints.map(j => [j.joint, j]));
  for (const j of joints) pointBall(plateAt(bar, j), POINT_COLOUR);
  ((bar.pairs || {}).spacing || []).forEach((gap, k) => {
    const middle = plateAt(bar, byJoint[gap.from]).lerp(plateAt(bar, byJoint[gap.to]), 0.5);
    note(`${plateName(gap.from)} → ${plateName(gap.to)} · ${gap.mm.toFixed(1)} / ${(gap.mm - gap.err_mm).toFixed(1)} `
      + `(off ${gap.err_mm >= 0 ? '+' : ''}${gap.err_mm.toFixed(2)}) mm`, plateLabelAt(middle, k));
  });
}

/** Fit residual: at each plate, its name and its balls' distances from their design spots after the fit
 * (M1 / M2, mm).
 *
 * @param {object} bar the measured bar
 */
function drawFit(bar) {
  const byPlate = {};
  for (const ball of bar.markers_sensed || []) {
    if (ball.state !== 'matched' || !ball.name) continue;
    const [plate, label] = [ball.name.replace(/_mocap_M\d+$/, ''), ball.name.split('_').pop()];
    (byPlate[plate] = byPlate[plate] || []).push({ label, pos: ball.pos, residual: ball.residual_mm });
  }
  // In the bar's order (by the plates' distance along it), so the labels alternate above and below.
  ((bar.pairs || {}).joints || []).map(j => plateName(j.joint)).filter(plate => byPlate[plate]).forEach((plate, k) => {
    const balls = byPlate[plate];
    const middle = balls.reduce((sum, b) => sum.add(new THREE.Vector3().fromArray(b.pos)), new THREE.Vector3())
      .divideScalar(balls.length);
    const text = balls.sort((a, b) => a.label.localeCompare(b.label))
      .map(b => b.residual === null || b.residual === undefined ? '--' : b.residual.toFixed(2)).join(' / ');
    note(`${plate} · ${text} mm`, plateLabelAt(middle, k));
  });
}

/** Tool0 vs target / servo residual / load: each arm's number at its flange (where FK puts it), for the
 * record; for the load also the gap between the two wrists, between the flanges.
 *
 * @param {object} record the shown record
 * @param {string} key ``'tool0'``, ``'servo'`` or ``'load'``
 */
function drawFlanges(record, key) {
  const flanges = [];
  for (const [side, t] of Object.entries(record.tool0 || {})) {
    if (!t.pos) continue;
    const at = new THREE.Vector3().fromArray(t.pos);
    flanges.push(at.clone());
    pointBall(at, POINT_COLOUR);
    let text;
    if (key === 'tool0') {
      text = t.fk_vs_target ? `${side} · tool0 vs target ${t.fk_vs_target.dpos_mm.toFixed(2)} mm` : `${side} · no target`;
    } else if (key === 'servo') {
      const s = record.servo && record.servo.arms[side];
      text = s ? `${side} · servo residual ${s.pos_norm_mm.toFixed(2)} mm` : `${side} · no servo in this step`;
    } else {
      const force = record.load ? record.load[`${side}_N`] : null;
      text = force === null || force === undefined ? `${side} · no force reading` : `${side} · ${force.toFixed(2)} N`;
    }
    note(text, at.setZ(at.z + 0.08));
  }
  // * The load's gap (| left - right |), halfway between the two wrists, under the bar (the action's
  // * label sits above its middle).
  if (key === 'load' && record.load && flanges.length === 2) {
    const middle = flanges[0].lerp(flanges[1], 0.5);
    note(`gap ${record.load.imbalance.toFixed(2)} N`, middle.setZ(middle.z - 0.10));
  }
}

// * The measurements drawn on the bars themselves; for the others the number goes on the bar's name label.
const DRAWN_ON_BAR = { placement: bar => drawPlacement(bar, bar.reference, [bar.tip_start, bar.tip_middle, bar.tip_end]),
  placement_seat: bar => drawPlacement(bar, bar.seat, [bar.seat_start, bar.seat_middle, bar.seat_end]),
  rotation: drawRotation, bend: drawBow, pair_length: drawPairs, spacing: drawSpacing, fit_residual: drawFit };
// The measurements of the record's robot, written at its two flanges.
const DRAWN_AT_FLANGES = ['tool0', 'servo', 'load'];

/** Draw the "data by" measurement in the view, and nothing else: on every measured bar, or, once a bar
 * is clicked, on that bar only (so nothing of another measurement or another bar overlaps it). */
function showAnnotations() {
  clearGroup(groups.notes);
  if (DRAWN_AT_FLANGES.includes(dataMetric.key)) {
    const record = currentRecord();
    if (record) drawFlanges(record, dataMetric.key);
    return;
  }
  const draw = DRAWN_ON_BAR[dataMetric.key];
  if (!draw) return;
  for (const bar of measured) {
    if (!selectedId || bar.bar === selectedId) draw(bar);
  }
}

/** Pin one bar by id (null unpins): glow, labels, the numbers along it and its card. */
function select(id) {
  selectedId = id || null;
  for (const bar of measured) {
    bar._ballMaterial.emissive.setHex(bar.bar === selectedId ? 0x33425c : 0x000000);
    bar._label.userData.muted = Boolean(selectedId) && bar.bar !== selectedId;
  }
  showAnnotations();
  relabel();
  if (selectedId) showDetail(selectedId); else closeDetail();
}

// ---------------------------------------------------------------------------
// * The step controls: slider, record pills, the strip of every action
// ---------------------------------------------------------------------------

const slider = document.getElementById('action');
slider.max = String(DATA.actions.length - 1);

/** Go to action k (clamped), on the record picked there before, else its default record. */
function setAction(k) {
  actionIndex = Math.max(0, Math.min(DATA.actions.length - 1, k));
  recordIndex = chosenRecord[actionIndex] ?? -1;
  slider.value = String(actionIndex);
  buildStep();
}

/** Show record i of the current action. */
function setRecord(i) {
  recordIndex = i;
  chosenRecord[actionIndex] = i;
  buildStep();
}

/** The value one action shows in the strip. */
function stripValue(a) {
  if (!stripBar) return a.worst;
  const record = a.records.length ? a.records[defaultRecord(a)] : null;
  const bar = record ? record.bars.find(b => b.bar === stripBar && b.ok) : null;
  return bar ? bar.placement_seat : null;
}

/** Redraw the slider's label, the record pills and the strip of every action. */
function drawControls() {
  const a = action();
  slider.value = String(actionIndex);
  const held = Object.entries(a.held).map(([id, robot]) => `${id} held by ${robot}`).join(', ');
  // * The bar first: B20's J and R read as the same bar (the actions are sorted by bar).
  document.getElementById('action-label').innerHTML = `<b>${a.bar_id} · ${a.kind} (${KIND_TEXT[a.kind] || a.kind})</b>`
    + ` · ${a.robot} <span class="muted">· plan action ${a.index} ${a.action_id} · ${a.exists.length} bars`
    + `${held ? ' · ' + held : ''}</span>`;
  const shown = currentRecord();
  const pills = visibleRecords(a).map(i => {
    const r = a.records[i];
    const cls = ['rec'];
    if (r === shown) cls.push('on');
    if (r.step.outcome && r.step.outcome !== 'ok') cls.push('cancelled');
    return `<button class="${cls.join(' ')}" data-i="${i}" title="${r.step.movement_id || 'a click'} · ${r.step.phase}`
      + ` · ${r.time}">${r.redo ? '↻ ' : ''}${r.label}</button>`;
  });
  const box = document.getElementById('records');
  box.innerHTML = pills.length ? pills.join('') : '<span class="muted">not executed: the authored bars only</span>';
  for (const button of box.querySelectorAll('button')) button.onclick = () => setRecord(Number(button.dataset.i));
  const strip = document.getElementById('timeline');
  strip.innerHTML = DATA.actions.map((act, k) => {
    const v = stripValue(act);
    const colour = !act.records.length ? '#eef0f3' : css(v === null || v === undefined ? NODATA : colourFor(v, placementMetric.high));
    return `<div class="cell${k === actionIndex ? ' cur' : ''}" data-k="${k}"`
      + ` style="background:${colour}" title="${act.bar_id} · ${act.kind} · ${act.action_id}`
      + `${v !== null && v !== undefined ? ' · ' + v.toFixed(2) + ' mm' : ''}"></div>`;
  }).join('');
  for (const cell of strip.querySelectorAll('.cell')) cell.onclick = () => setAction(Number(cell.dataset.k));
}

/** The side panel's lines about the shown action and record. */
function drawStatus() {
  const a = action();
  const record = currentRecord();
  const failed = record ? record.bars.filter(b => !b.ok) : [];
  document.getElementById('status').innerHTML =
    `${a.records.length} record(s) at this action`
    + (record ? `<br>shown: ${record.step.movement_id || 'a click'} · ${record.step.phase} · ${record.time}` : '')
    + (record && record.servo ? `<br>servo: ${record.servo.file}` : '')
    + (failed.length ? `<br><span class="warn">fit failed: ${failed.map(b => b.bar + ' (' + b.reason + ')').join('; ')}</span>` : '')
    + `<br><br>${DATA.actions.filter(x => x.records.length).length} of ${DATA.actions.length} actions recorded · `
    + `${DATA.env ? DATA.env.count : 0} environment solids · ${(DATA.cameras || []).length} mocap cameras<br>`
    + `thin lines (authored): grey placed · blue the action's bar · orange held by Alice · violet held by Belle · `
    + `light blue held by Cindy. See-through, shaded: measured by MoCap. Distances are millimetres.<br>`
    + `The "data by" measurement is drawn on every measured bar, or on the clicked one only; its arrows are `
    + `${MAGNIFY}× longer than the error they show (the numbers are true).`;
}

// ---------------------------------------------------------------------------
// * Interaction
// ---------------------------------------------------------------------------

wirePicking(pickable, bar => select(bar.bar));
slider.oninput = () => setAction(Number(slider.value));
document.getElementById('prev').onclick = () => setAction(actionIndex - 1);
document.getElementById('next').onclick = () => setAction(actionIndex + 1);
document.getElementById('all-records').onchange = e => { showAllRecords = e.target.checked; drawControls(); };
const stripPicker = document.getElementById('strip-by');
stripPicker.innerHTML = '<option value="">worst bar of each action</option>'
  + Object.keys(DATA.history).map(id => `<option value="${id}">${id} (vs its seat)</option>`).join('');
stripPicker.onchange = () => { stripBar = stripPicker.value; drawControls(); };

addEventListener('keydown', e => {
  if (e.target && (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT')) return;
  if (e.key === 'Escape') select(null);
  if (e.key === 'ArrowLeft') setAction(actionIndex - 1);
  if (e.key === 'ArrowRight') setAction(actionIndex + 1);
  const a = action();
  const shownIndex = recordIndex >= 0 ? recordIndex : defaultRecord(a);
  if (e.key === '[' && shownIndex > 0) setRecord(shownIndex - 1);
  if (e.key === ']' && shownIndex < a.records.length - 1) setRecord(shownIndex + 1);
  if (e.key === 'm' || e.key === 'M') setMetric((DATA.metrics.indexOf(metric) + 1) % DATA.metrics.length);
});

/** Switch which measurement the colours show. */
function setMetric(index) {
  metric = DATA.metrics[index];
  for (const id of ['metric', 'metric-legend']) {
    const picker = document.getElementById(id);
    if (picker) picker.value = String(index);
  }
  const help = document.getElementById('metric-help');
  if (help) help.textContent = metric.help;
  recolour();
}

/** Switch which measurement is written on the bars and in bold in the card. */
function setDataMetric(index) {
  dataMetric = DATA.metrics[index];
  const picker = document.getElementById('data-legend');
  if (picker) picker.value = String(index);
  relabel();
  showAnnotations();
  if (selectedId) showDetail(selectedId);
}

wirePicker('metric-legend', DATA.metrics, metric, setMetric);
wirePicker('data-legend', DATA.metrics, dataMetric, setDataMetric);
const frameAll = () => resetView(groups.authored);
wireLayers(frameAll);

// ---------------------------------------------------------------------------
// * Go
// ---------------------------------------------------------------------------

document.getElementById('subtitle').textContent =
  `${DATA.problem} · ${DATA.day} · generated ${DATA.generated}`;
setAction(actionIndex);
setMetric(0);
resize();
// Frame every bar of the design once (the seats, shown or not).
for (const seat of Object.values(seatRods)) seat.rod.visible = true;
frameAll();
buildStep();
window.__viewerReady = true;
startRenderLoop();
