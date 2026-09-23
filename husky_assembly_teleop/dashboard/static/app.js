// Dashboard page: run list, live notifications, stats, charts, and the viewer.
//
// ! The three.js viewer is imported lazily, on the first click into it. It is
// ! the heaviest part of the page and the only one with third-party imports, so
// ! a problem there must not stop the runs list, the table and the charts from
// ! working -- an import at the top of the module would do exactly that.

// Outcome -> colour and words. Track breaks are the overwhelming majority, so
// they stay grey and recessive; the informative outcomes are the loud ones.
const OUTCOME = {
  corridor:            { colour: '#0ca30c', label: 'clear path found' },
  blocked:             { colour: '#fab219', label: 'reached home, path blocked' },
  arrival_collision:   { colour: '#ec835a', label: 'reached home, collides there' },
  fine_reverify_failed:{ colour: '#a06ad0', label: 'clear coarse, not fine' },
  track_break:         { colour: '#8a8a86', label: 'never reached home' },
};
const ANCHOR_COLOUR = { horizontal: '#2a78d6', vertical: '#1baf7a', back: '#eb6834' };
const ORDER = ['corridor', 'blocked', 'arrival_collision', 'fine_reverify_failed', 'track_break'];

const $ = (id) => document.getElementById(id);
let currentRun = null;
let viewer = null;
let viewerPayload = null;
// The robot overlay is a few hundred kilobytes of triangles and the operator
// flips it on and off, so each run/configuration pair is fetched once and kept.
const ROBOT_SOUPS = new Map();
const ROBOT_NAMES = { goal: 'robot at the goal pose', start: 'robot at the pick-up pose' };

function fail(what, error) {
  console.error(what, error);
  $('status').textContent = `${what}: ${error && error.message ? error.message : error}`;
  $('status').style.color = '#ec835a';
}

async function getViewer() {
  if (!viewer) {
    const { createViewer } = await import('./viewer.js');
    viewer = createViewer($('viewer-host'));
  }
  return viewer;
}

// A dot says where the bar's middle was; a short stick along the bar says
// which way it pointed. The bar's long axis is its LOCAL Z, so the stick
// direction is the third column of the orientation's rotation matrix.
function barAxis(quat) {
  const [x, y, z, w] = quat;
  return [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)];
}

// Half-lengths to draw a bar stick at, in metres. "span" is the true distance
// between the two tool0 grasps, read from the run.
function stickHalfLength(mode) {
  if (mode === 'off') return 0;
  if (mode === 'span') return graspSpan() / 2;
  return 0.15;
}

function graspSpan() {
  const grasps = currentRun.context.grasps;
  const a = grasps.bar_from_left_tool0.pos;
  const b = grasps.bar_from_right_tool0.pos;
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

// One lines trace holding many separate segments: plotly breaks the line
// wherever a NaN appears, so the whole group costs one trace instead of one
// per candidate.
function stickTrace(cands, colour, half, name) {
  const x = [], y = [], z = [];
  cands.forEach((cand) => {
    const p = cand.home_pos_mb;
    const a = barAxis(cand.home_quat_mb);
    x.push(p[0] - a[0] * half, p[0] + a[0] * half, NaN);
    y.push(p[1] - a[1] * half, p[1] + a[1] * half, NaN);
    z.push(p[2] - a[2] * half, p[2] + a[2] * half, NaN);
  });
  return {
    type: 'scatter3d', mode: 'lines', x, y, z,
    line: { color: colour, width: 2 },
    name, showlegend: false, hoverinfo: 'skip',
  };
}

function fmt(value, digits = 1) {
  return (value === null || value === undefined) ? '-' : Number(value).toFixed(digits);
}

// Which robot configuration to draw, taking the select's answer but falling
// back to the goal pose when the sweep never derived a pick-up pose.
function robotPoseChoice() {
  const hasStart = !!(currentRun.result && currentRun.result.start_conf);
  const select = $('robot-pose');
  select.querySelector('option[value="start"]').disabled = !hasStart;
  if (select.value === 'start' && !hasStart) select.value = 'goal';
  return select.value;
}

// The husky as one plotly mesh3d trace, in the same mobile-base frame as the
// dots. Returns null when the robot could not be fetched, so the caller can
// still plot the dots -- a missing robot must never blank the chart.
async function robotTrace(pose) {
  const key = `${currentRun.id}|${pose}`;
  try {
    if (!ROBOT_SOUPS.has(key)) {
      const soup = await (await fetch(
        `/api/runs/${encodeURIComponent(currentRun.id)}/robot?conf=${pose}`)).json();
      if (!soup.x) throw new Error(soup.error || 'no robot geometry came back');
      ROBOT_SOUPS.set(key, soup);
    }
  } catch (error) {
    fail('could not draw the robot', error);
    return null;
  }
  const soup = ROBOT_SOUPS.get(key);
  return {
    type: 'mesh3d', name: ROBOT_NAMES[pose], showlegend: true,
    x: soup.x, y: soup.y, z: soup.z, i: soup.i, j: soup.j, k: soup.k,
    // Muted and see-through: the robot is the reference, the dots are the data.
    color: '#6b727c', opacity: 0.35, flatshading: true, hoverinfo: 'skip',
  };
}

// ---------------------------------------------------------------- run list
async function refreshRuns(selectId) {
  let runs;
  try {
    runs = await (await fetch('/api/runs')).json();
  } catch (error) {
    fail('could not load the runs list', error);
    return;
  }
  const list = $('run-list');
  list.innerHTML = '';
  if (!runs.length) {
    list.innerHTML = '<li class="empty">No runs yet.</li>';
    return;
  }
  runs.forEach((run) => {
    const item = document.createElement('li');
    const when = (run.created || '').replace('T', ' ').slice(5, 16);
    const found = run.result_kind === 'failed'
      ? 'no start found'
      : `${run.result_kind.replace('_', ' ')} after ${fmt(run.t_found_s, 0)} s`;
    item.innerHTML = `<div><strong>${run.bar_action}</strong> &middot; ${run.anchor_selection}</div>
      <div class="when">${when} &middot; ${found} &middot; ${fmt(run.t_total_s, 0)} s total</div>`;
    item.onclick = () => openRun(run.id);
    item.dataset.id = run.id;
    list.appendChild(item);
  });
  if (selectId) openRun(selectId);
}

async function openRun(id) {
  currentRun = await (await fetch(`/api/runs/${encodeURIComponent(id)}`)).json();
  [...$('run-list').children].forEach((item) =>
    item.classList.toggle('active', item.dataset.id === id));
  renderSummary();
  renderVariantTable();
  renderPositions();
  renderTimeline();
}

// ---------------------------------------------------------------- summary
function renderSummary() {
  $('summary').innerHTML = '<h2>Summary</h2>'
    + (currentRun.summary || []).map((line) => `<p>${line}</p>`).join('');
}

// --------------------------------------------------------- per-variant table
function renderVariantTable() {
  const rows = new Map();
  currentRun.candidates.forEach((cand) => {
    if (!rows.has(cand.variant)) {
      rows.set(cand.variant, { variant: cand.variant, anchor: cand.anchor,
        tried: 0, reached: [], seconds: 0, counts: {}, breaks: { ik_miss: 0, branch_flip: 0 } });
    }
    const row = rows.get(cand.variant);
    row.tried += 1;
    row.counts[cand.outcome] = (row.counts[cand.outcome] || 0) + 1;
    row.reached.push((cand.reached || 0) * (cand.travel_cm || 0));
    row.seconds += cand.t_track_s || 0;
    if (cand.break && cand.break.reason) row.breaks[cand.break.reason] += 1;
  });
  const cuts = new Set((currentRun.profile.budget_cuts || []).map((cut) => cut[1]));
  const head = `<tr><th>home pose tried</th><th>attempts</th><th>clear path</th>
    <th>blocked</th><th>collides at home</th><th>IK missed</th><th>branch flip</th>
    <th>median cm reached</th><th>seconds</th></tr>`;
  const body = [...rows.values()].map((row) => {
    const sorted = row.reached.slice().sort((a, b) => a - b);
    const median = sorted.length ? sorted[Math.floor(sorted.length / 2)] : 0;
    const cut = cuts.has(row.variant) ? ' class="cut"' : '';
    return `<tr${cut}><td>${row.variant}${cuts.has(row.variant) ? ' &mdash; time share ran out here' : ''}</td>
      <td>${row.tried}</td><td>${row.counts.corridor || 0}</td><td>${row.counts.blocked || 0}</td>
      <td>${row.counts.arrival_collision || 0}</td><td>${row.breaks.ik_miss}</td>
      <td>${row.breaks.branch_flip}</td><td>${median.toFixed(0)}</td>
      <td>${row.seconds.toFixed(1)}</td></tr>`;
  }).join('');
  $('variant-table').innerHTML = `<table>${head}${body}</table>`;
}

// ------------------------------------------------------- chart A: positions
async function renderPositions() {
  const mode = $('colour-by').value;
  const cands = currentRun.candidates;
  const traces = [];

  if (mode === 'outcome') {
    ORDER.forEach((outcome) => {
      const picked = cands.filter((cand) => cand.outcome === outcome);
      if (!picked.length) return;
      traces.push(scatterTrace(picked, OUTCOME[outcome].label,
        { color: OUTCOME[outcome].colour, size: outcome === 'track_break' ? 3 : 6 }));
    });
  } else {
    const values = cands.map((cand) => mode === 'seconds'
      ? (cand.t_track_s || 0)
      : (cand.reached || 0) * (cand.travel_cm || 0));
    traces.push(scatterTrace(cands, mode === 'seconds' ? 'seconds' : 'cm reached', {
      color: values, size: 5, colorscale: 'Viridis', showscale: true,
      colorbar: { title: mode === 'seconds' ? 's' : 'cm', thickness: 10 },
    }));
  }

  // Bar direction sticks, coloured like their dots.
  const half = stickHalfLength($('stick-mode').value);
  if (half > 0) {
    if (mode === 'outcome') {
      ORDER.forEach((outcome) => {
        const picked = cands.filter((cand) => cand.outcome === outcome);
        if (picked.length) {
          traces.push(stickTrace(picked, OUTCOME[outcome].colour, half, outcome));
        }
      });
    } else {
      traces.push(stickTrace(cands, '#7f8794', half, 'bar direction'));
    }
  }

  const goal = currentRun.context.goal.bar_pos_mb;
  traces.push({
    type: 'scatter3d', mode: 'markers', name: 'goal bar',
    x: [goal[0]], y: [goal[1]], z: [goal[2]],
    marker: { size: 10, color: '#ffffff', symbol: 'diamond' },
    hovertemplate: 'the bar where M1 must end<extra></extra>',
  });
  // The goal bar always at true grasp span, as the orientation reference.
  const goalAxis = barAxis(currentRun.context.goal.bar_quat_mb);
  const goalHalf = graspSpan() / 2;
  traces.push({
    type: 'scatter3d', mode: 'lines', name: 'goal bar direction',
    x: [goal[0] - goalAxis[0] * goalHalf, goal[0] + goalAxis[0] * goalHalf],
    y: [goal[1] - goalAxis[1] * goalHalf, goal[1] + goalAxis[1] * goalHalf],
    z: [goal[2] - goalAxis[2] * goalHalf, goal[2] + goalAxis[2] * goalHalf],
    line: { color: '#ffffff', width: 6 }, hoverinfo: 'skip',
  });

  // The robot goes in before the single draw, so a slow or failed fetch never
  // leaves the chart half-rendered or throws the camera away on a second pass.
  const pose = robotPoseChoice();
  if (pose !== 'off') {
    const robot = await robotTrace(pose);
    if (robot) traces.push(robot);
  }

  Plotly.react($('chart-positions'), traces, layout3d(), { displaylogo: false });
  $('chart-positions').removeAllListeners?.('plotly_click');
  $('chart-positions').on('plotly_click', (event) => {
    const index = event.points[0].customdata;
    if (index !== undefined) showCandidate(index);
  });
}

function scatterTrace(cands, name, marker) {
  return {
    type: 'scatter3d', mode: 'markers', name,
    x: cands.map((cand) => cand.home_pos_mb[0]),
    y: cands.map((cand) => cand.home_pos_mb[1]),
    z: cands.map((cand) => cand.home_pos_mb[2]),
    customdata: cands.map((cand) => cand.i),
    text: cands.map((cand) => cand.text),
    hovertemplate: '%{text}<extra></extra>',
    marker,
  };
}

function layout3d() {
  const dark = { color: '#9aa1ad', gridcolor: '#2c3039', zerolinecolor: '#3a414c' };
  return {
    paper_bgcolor: '#1c1f25', plot_bgcolor: '#1c1f25',
    font: { color: '#e6e8ec', size: 11 },
    margin: { l: 0, r: 0, t: 0, b: 0 },
    legend: { orientation: 'h', y: -0.02 },
    scene: {
      aspectmode: 'data',
      xaxis: { title: 'x forward [m]', ...dark },
      yaxis: { title: 'y left [m]', ...dark },
      zaxis: { title: 'z up [m]', ...dark },
    },
  };
}

// -------------------------------------------------------- chart B: timeline
function renderTimeline() {
  const cands = currentRun.candidates;
  const anchors = [...new Set(cands.map((cand) => cand.anchor))];
  const traces = ORDER.filter((outcome) => cands.some((cand) => cand.outcome === outcome))
    .map((outcome) => {
      const picked = cands.filter((cand) => cand.outcome === outcome);
      return {
        type: 'scatter', mode: 'markers', name: OUTCOME[outcome].label,
        x: picked.map((cand) => cand.t_at_s),
        y: picked.map((cand) => cand.anchor),
        customdata: picked.map((cand) => cand.i),
        text: picked.map((cand) => cand.text),
        hovertemplate: '%{x:.1f} s &mdash; %{text}<extra></extra>',
        marker: { color: OUTCOME[outcome].colour,
          size: outcome === 'track_break' ? 5 : 9, opacity: 0.85 },
      };
    });

  const shapes = (currentRun.profile.budget_cuts || []).map((cut) => {
    const at = cands[Math.min(cut[2] || 0, cands.length - 1)];
    return { type: 'line', x0: at ? at.t_at_s : 0, x1: at ? at.t_at_s : 0,
      y0: -0.5, y1: anchors.length - 0.5,
      line: { color: '#f0b429', width: 1, dash: 'dash' } };
  });
  const found = currentRun.result.t_found_s;
  if (found !== null && found !== undefined) {
    shapes.push({ type: 'line', x0: found, x1: found, y0: -0.5, y1: anchors.length - 0.5,
      line: { color: '#0ca30c', width: 2 } });
  }

  Plotly.react($('chart-timeline'), traces, {
    paper_bgcolor: '#1c1f25', plot_bgcolor: '#1c1f25',
    font: { color: '#e6e8ec', size: 11 },
    margin: { l: 90, r: 20, t: 10, b: 40 }, height: 300, shapes,
    xaxis: { title: 'seconds since the sweep started', gridcolor: '#2c3039' },
    yaxis: { categoryarray: anchors, gridcolor: '#2c3039' },
    legend: { orientation: 'h', y: -0.25 },
  }, { displaylogo: false });
  $('chart-timeline').removeAllListeners?.('plotly_click');
  $('chart-timeline').on('plotly_click', (event) => {
    const index = event.points[0].customdata;
    if (index !== undefined) showCandidate(index);
  });
}

// ------------------------------------------------------------------ viewer
async function showCandidate(index) {
  const panel = $('viewer-panel');
  panel.classList.remove('hidden');
  const cand = currentRun.candidates[index];
  $('viewer-title').textContent =
    `${currentRun.bar_action} &middot; ${cand.variant} &middot; ${OUTCOME[cand.outcome].label}`
      .replace('&middot;', '·');
  $('viewer-caption').textContent = cand.text || '';

  let view;
  try {
    view = await getViewer();
  } catch (error) {
    fail('3D viewer could not load', error);
    $('viewer-caption').textContent =
      'The 3D viewer failed to load. Re-run scripts/fetch_dashboard_vendor.sh; '
      + 'the rest of the dashboard still works.';
    return;
  }

  const payload = await (await fetch(
    `/api/runs/${encodeURIComponent(currentRun.id)}/candidates/${index}/frames`)).json();
  // The viewer asks for the contact points of whichever frame is showing.
  payload.collisionsFor = (frameIndex) =>
    (payload.frames[frameIndex] || {}).colliding?.length ? cand.collisions : null;
  viewerPayload = payload;

  await view.loadScene(`/api/scenes/${currentRun.context.scene.glb}`,
                       currentRun.context.static_bodies);
  const n = view.showCandidate(payload);
  const slider = $('frame-slider');
  slider.max = String(Math.max(0, n - 1));
  slider.value = String(n - 1);            // land on the interesting end
  setFrame(n - 1);
  view.setHiddenVisible($('show-hidden').checked);
}

function setFrame(index) {
  if (!viewer || !viewerPayload) return;
  $('frame-label').textContent = viewer.setFrame(index, viewerPayload) || '';
}

// ------------------------------------------------------------ notifications
function connectEvents() {
  const source = new EventSource('/events');
  source.onopen = () => { $('status').textContent = 'listening for new runs'; };
  source.onerror = () => { $('status').textContent = 'reconnecting…'; };
  source.addEventListener('run_added', (event) => {
    const id = JSON.parse(event.data).id;
    refreshRuns($('auto-open').checked ? id : null);
    if (!$('auto-open').checked) toast(id);
  });
}

function toast(id) {
  const box = $('toast');
  box.innerHTML = `<div>New derivation run: <strong>${id.split('_').slice(-2).join(' ')}</strong></div>`;
  const open = document.createElement('button');
  open.textContent = 'Open';
  open.onclick = () => { box.classList.add('hidden'); openRun(id); };
  box.appendChild(open);
  box.classList.remove('hidden');
  setTimeout(() => box.classList.add('hidden'), 15000);
}

// -------------------------------------------------------------------- wiring
$('colour-by').onchange = () => currentRun && renderPositions();
$('stick-mode').onchange = () => currentRun && renderPositions();
$('robot-pose').onchange = () => currentRun && renderPositions();
$('frame-slider').oninput = (event) => setFrame(Number(event.target.value));
$('viewer-reset').onclick = () => viewer && viewer.resetView();
$('viewer-close').onclick = () => $('viewer-panel').classList.add('hidden');
$('show-hidden').onchange = (event) => viewer && viewer.setHiddenVisible(event.target.checked);
document.addEventListener('keydown', (event) => {
  if ($('viewer-panel').classList.contains('hidden')) return;
  const slider = $('frame-slider');
  if (event.key === 'ArrowLeft') slider.value = String(Math.max(0, Number(slider.value) - 1));
  else if (event.key === 'ArrowRight') slider.value = String(Math.min(Number(slider.max), Number(slider.value) + 1));
  else return;
  setFrame(Number(slider.value));
});

refreshRuns();
connectEvents();
window.addEventListener('error', (event) => fail('page error', event.error || event.message));
