// The browser half of 2_session_viewer.py: what is specific to a bar-holding
// session. The scene, the drawing primitives, the legend, the card and the
// picking come from viewer_common.js; this file reads the json blob in
// #scene-data and builds the bars where mocap found them, their 8 markers,
// the authored pose beside each one, and one footprint per robot (the clicked
// bar's arm is drawn in full).
//
// ! Read as text by viewer_common.py's viewer_script() and pasted after
// ! viewer_common.js into ONE module, with the import line below dropped. So:
// ! keep the import on ONE line, and never declare a top-level name that
// ! viewer_common.js already declares (see the rules in its header).

import * as THREE from 'three';
import { groups, hex, MARKER_R, BAR_RADIUS, PICK_RADIUS, BAR_OPACITY, BAR_STEPS, colourFor, vertexColour, addEnvironment, placeFloor, labelSprite, rodBetween, dashedRod, paintRod, footprintFor, addCameras, unpackLinkGeometries, armGroup, decimalsFor, drawLegend, fmt, row, strong, block, formulasHtml, openDetail, closeDetail, wirePicking, wirePicker, wireLayers, resetView, resize, startRenderLoop } from './viewer_common.js';

const DATA = JSON.parse(document.getElementById('scene-data').textContent);

// * Two independent choices: which measurement drives the COLOUR, and which
// * one each bar is labelled with. They start the same but need not stay so --
// * colouring by placement while reading the load, say.
let metric = DATA.metrics[0];        // drives the gradient and the legend
let dataMetric = DATA.metrics[0];    // drives the number written on each bar
let selected = null;

/** One measurement's definition, found by name rather than by position. */
const metricByKey = key => DATA.metrics.find(m => m.key === key);

/** One bar's value for a given measurement, or null when it has none. */
function valueFor(bar, which) {
  if (which.key === 'load' && !bar.load_valid) return null;
  const v = bar[which.key];
  return (v === null || v === undefined) ? null : v;
}

/** The value a bar contributes to the COLOUR metric. */
function valueOf(bar) { return valueFor(bar, metric); }

// ---------------------------------------------------------------------------
// * Building the pieces
// ---------------------------------------------------------------------------

addEnvironment(DATA.env);
placeFloor(DATA.floor);

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
  const sleeve = rodBetween(bar.fitted[0], bar.fitted[1], PICK_RADIUS,
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
  for (const bar of DATA.bars) {
    if (!bar.robot) continue;
    groups.robots.add(footprintFor(bar.robot.base, DATA.robot.footprint, bar));
  }
}

addCameras(DATA.cameras, DATA.cone);

// The robot's link shapes, built once and reused by whichever bar is selected.
const linkGeometries = unpackLinkGeometries(DATA.robot && DATA.robot.meshes);

/** Draw the selected bar's robot with its real link geometry. */
function showArm(bar) {
  groups.arm.clear();
  if (!bar || !bar.robot) return;
  groups.arm.add(armGroup(bar.robot.links, linkGeometries));
}

// ---------------------------------------------------------------------------
// * Colour, legend and the side panel
// ---------------------------------------------------------------------------

function recolour() {
  for (const bar of DATA.bars) {
    // ? Only the placement error exists at every point ALONG a bar; every
    // ? other measurement is one number for the whole thing, so the rod is flat.
    paintRod(bar, metric.key === 'placement'
      ? null : vertexColour(valueOf(bar), metric.high), metric.high);
    bar._marker.color.setHex(hex(colourFor(valueOf(bar), metric.high)));
  }
  relabel();
  drawLegend(metric, DATA.bars.map(valueOf),
    DATA.bars.filter(b => { const v = valueOf(b); return v !== null && v > metric.high; })
      .map(b => b.bar));
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
    + row('average of the three', fmt(bar.tip_mean, 2, 'mm')), dataMetric.key);

  html += block('rotation', 'rotation error',
    'angle between the detected and authored bar axes',
    row('rotation error',
        strong(fmt(bar.rotation, 3, 'deg'), asked('rotation'))), dataMetric.key);

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
    pairs, dataMetric.key);

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
    : '<div class="sub">no servo run matched to this bar</div>', dataMetric.key);

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
  html += block('load', 'load — is the bar being bent?', '', load, dataMetric.key);

  html += formulasHtml(DATA.metrics, DATA.extra_formulas);
  openDetail(html, () => select(null));
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
    closeDetail();
  }
}

// ---------------------------------------------------------------------------
// * Interaction
// ---------------------------------------------------------------------------

wirePicking(pickable, select);

addEventListener('keydown', e => {
  if (e.key === 'Escape') select(null);
  // M cycles the colour metric without reaching for the dropdown.
  if (e.key === 'm' || e.key === 'M') {
    setMetric((DATA.metrics.indexOf(metric) + 1) % DATA.metrics.length);
  }
});

// * Two pickers, each driving one thing: the gradient, and the number written
// * on the bars. "M" still cycles the colour.
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

wirePicker('metric', DATA.metrics, metric, setMetric);
wirePicker('metric-legend', DATA.metrics, metric, setMetric);
wirePicker('data-legend', DATA.metrics, dataMetric, setDataMetric);
wireLayers(resetView);

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
startRenderLoop();
