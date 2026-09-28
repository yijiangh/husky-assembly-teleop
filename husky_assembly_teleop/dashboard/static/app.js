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

// A dot says where the MIDDLE of the bar was; the stick through it is the bar
// itself. The bar's long axis is its LOCAL Z, so the stick direction is the
// third column of the orientation's rotation matrix.
function barAxis(quat) {
  const [x, y, z, w] = quat;
  return [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)];
}

// ! A stored bar pose is the pose of the bar's own FRAME, whose origin is at one
// ! END of the bar -- and these bars are up to two metres long. Drawing the bar
// ! centred on that origin put it up to a metre from where the planner really
// ! had it (on B3 the grips are 22 and 118 cm along, so a centred stick missed
// ! by 70 cm, which is what made the goal bar look like it hung beside the
// ! robot, through the floor). So: every span here is measured ALONG the bar
// ! from its own origin, and the dot is moved onto the middle of the bar.
function barSpan(mode) {
  if (mode === 'off') return null;
  if (mode === 'direction') {                       // a direction hint, not the bar
    const mid = barMidAlong();
    return [mid - 0.15, mid + 0.15];
  }
  const extent = currentRun.context.active_bar_extent_local;
  if (mode === 'grips' || !extent) return graspSpan();
  return [extent.min[2], extent.max[2]];
}

// How far along the bar, from its own origin, the middle of the bar sits. Run
// files written before the bar extent was recorded only know where the grips
// are, so their midpoint stands in.
function barMidAlong() {
  const extent = currentRun.context.active_bar_extent_local;
  if (extent) return (extent.min[2] + extent.max[2]) / 2;
  const [near, far] = graspSpan();
  return (near + far) / 2;
}

// The point plotted for one bar pose: the middle of the bar.
function barMiddle(pos, quat) {
  const axis = barAxis(quat);
  const along = barMidAlong();
  return pos.map((v, i) => v + axis[i] * along);
}

// Where along its own axis the two grippers hold the bar, in metres from the
// bar's origin: the grasps are tool0 poses in the BAR's frame, so their third
// coordinate is exactly that distance.
function graspSpan() {
  const grasps = currentRun.context.grasps;
  const a = grasps.bar_from_left_tool0.pos[2];
  const b = grasps.bar_from_right_tool0.pos[2];
  return [Math.min(a, b), Math.max(a, b)];
}

// The two ends of the drawn bar for one pose, in the robot's own frame.
function barEnds(pos, quat, span) {
  const axis = barAxis(quat);
  return span.map((along) => pos.map((v, i) => v + axis[i] * along));
}

// One lines trace holding many separate segments: plotly breaks the line
// wherever a NaN appears, so the whole group costs one trace instead of one
// per candidate.
function stickTrace(cands, colour, span, name) {
  const x = [], y = [], z = [];
  cands.forEach((cand) => {
    const [from, to] = barEnds(cand.home_pos_mb, cand.home_quat_mb, span);
    x.push(from[0], to[0], NaN);
    y.push(from[1], to[1], NaN);
    z.push(from[2], to[2], NaN);
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

// ---------------------------------------------------------- design problem
// The runs list shows ONE design problem at a time; the server skips the other
// problems' run files by name. It starts on the monitor's DESIGN_PROBLEM_NAME.
const selectedProblem = () => $('problem').value;

async function refreshProblems() {
  let payload;
  try {
    payload = await (await fetch('/api/problems')).json();
  } catch (error) {
    fail('could not load the design problems', error);
    return;
  }
  const select = $('problem');
  // Keep the operator's pick across refreshes; otherwise start on the default.
  const keep = select.value || payload.default;
  const names = payload.problems.map((problem) => problem.name);
  // The default problem may have no runs yet; list it anyway so it can be picked.
  if (!names.includes(payload.default)) {
    payload.problems.unshift({ name: payload.default, n_runs: 0 });
  }
  select.innerHTML = '';
  payload.problems.forEach((problem) => {
    const option = document.createElement('option');
    option.value = problem.name;
    option.textContent = `${problem.name} (${problem.n_runs} runs)`;
    select.appendChild(option);
  });
  select.value = keep;
}

// ---------------------------------------------------------------- run list
async function refreshRuns(selectId) {
  let runs;
  try {
    runs = await (await fetch(
      `/api/runs?problem=${encodeURIComponent(selectedProblem())}`)).json();
  } catch (error) {
    fail('could not load the runs list', error);
    return;
  }
  const list = $('run-list');
  list.innerHTML = '';
  if (!runs.length) {
    list.innerHTML = `<li class="empty">No runs yet for ${selectedProblem()}.</li>`;
    return;
  }
  runs.forEach((run) => {
    const item = document.createElement('li');
    const when = (run.created || '').replace('T', ' ').slice(5, 16);
    const found = run.result_kind === 'failed'
      ? 'no start found'
      : run.result_kind === 'reused_start'
        ? 'planned from the stored start'
        : `${run.result_kind.replace('_', ' ')} after ${fmt(run.t_found_s, 0)} s`;
    // A full plan run says how its search ended; a derive-only run has no search.
    const search = run.rrt_outcome
      ? ` &middot; RRT: ${String(run.rrt_outcome).replace(/_/g, ' ')}` : '';
    const attempt = run.attempt ? ` (attempt ${run.attempt})` : '';
    item.innerHTML = `<div><strong>${run.bar_action}</strong> &middot; ${run.anchor_selection}${attempt}</div>
      <div class="when">${when} &middot; ${found} &middot; ${fmt(run.t_total_s, 0)} s total${search}</div>`;
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
  renderRrt();
}

// ----------------------------------------------------- the RRT search
async function renderRrt() {
  const card = $('rrt-card');
  const rrt = currentRun.rrt;
  if (!rrt) {
    card.classList.add('hidden');
    return;
  }
  card.classList.remove('hidden');
  $('rrt-text').innerHTML = (currentRun.rrt_text || []).map((line) => `<p>${line}</p>`).join('');

  const traces = [];
  const cloud = (points, name, colour) => ({
    type: 'scatter3d', mode: 'markers', name,
    x: points.map((p) => p[0]), y: points.map((p) => p[1]), z: points.map((p) => p[2]),
    marker: { size: 2.5, color: colour, opacity: 0.7 }, hoverinfo: 'skip',
  });
  if (rrt.trees.start.length) traces.push(cloud(rrt.trees.start, `tree from the pick-up pose (${rrt.n_nodes_start} nodes)`, '#ec5a4f'));
  if (rrt.trees.goal.length) traces.push(cloud(rrt.trees.goal, `tree from the goal (${rrt.n_nodes_goal} nodes)`, '#4f9df7'));
  if (rrt.path_mb && rrt.path_mb.length) {
    traces.push({
      type: 'scatter3d', mode: 'lines', name: `path (${rrt.n_waypoints} waypoints)`,
      x: rrt.path_mb.map((p) => p[0]), y: rrt.path_mb.map((p) => p[1]), z: rrt.path_mb.map((p) => p[2]),
      line: { color: '#37c871', width: 6 }, hoverinfo: 'skip',
    });
  }
  if (rrt.start_bar_pos_mb) {
    const s = rrt.start_bar_pos_mb;
    traces.push({ type: 'scatter3d', mode: 'markers', name: "pick-up pose (bar's origin)",
      x: [s[0]], y: [s[1]], z: [s[2]],
      marker: { size: 9, color: '#37c871', symbol: 'diamond' }, hoverinfo: 'skip' });
  }
  const goal = currentRun.context.goal.bar_pos_mb;
  traces.push({ type: 'scatter3d', mode: 'markers', name: "goal (bar's origin)",
    x: [goal[0]], y: [goal[1]], z: [goal[2]],
    marker: { size: 10, color: '#ffffff', symbol: 'diamond' }, hoverinfo: 'skip' });
  // The robot at the goal for scale; cached, and never allowed to blank the chart.
  const robot = await robotTrace('goal');
  if (robot) traces.push(robot);
  Plotly.react($('chart-rrt'), traces, layout3d(), { displaylogo: false });
}

// ---------------------------------------------------------------- summary
function renderSummary() {
  $('summary').innerHTML = '<h2>Summary</h2>'
    + (currentRun.summary || []).map((line) => `<p>${line}</p>`).join('');
  // Which base the whole chart is measured from belongs next to the chart, not
  // only in the summary: it is the first thing to check when the bar looks
  // misplaced relative to the robot.
  $('frame-note').textContent = (currentRun.base_text || []).join(' ');
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

  // The bars themselves, coloured like their dots.
  const span = barSpan($('stick-mode').value);
  if (span) {
    if (mode === 'outcome') {
      ORDER.forEach((outcome) => {
        const picked = cands.filter((cand) => cand.outcome === outcome);
        if (picked.length) {
          traces.push(stickTrace(picked, OUTCOME[outcome].colour, span, outcome));
        }
      });
    } else {
      traces.push(stickTrace(cands, '#7f8794', span, 'the bar'));
    }
  }

  const goal = currentRun.context.goal.bar_pos_mb;
  const goalMiddle = barMiddle(goal, currentRun.context.goal.bar_quat_mb);
  traces.push({
    type: 'scatter3d', mode: 'markers', name: 'middle of the goal bar',
    x: [goalMiddle[0]], y: [goalMiddle[1]], z: [goalMiddle[2]],
    marker: { size: 10, color: '#ffffff', symbol: 'diamond' },
    hovertemplate: 'the bar where M1 must end<extra></extra>',
  });
  // The goal bar drawn the same way as the candidates, so the two are directly
  // comparable; it falls back to the grip span when the sticks are switched off.
  const goalSpan = span || graspSpan();
  const goalEnds = barEnds(goal, currentRun.context.goal.bar_quat_mb, goalSpan);
  traces.push({
    type: 'scatter3d', mode: 'lines', name: 'the goal bar',
    x: [goalEnds[0][0], goalEnds[1][0]],
    y: [goalEnds[0][1], goalEnds[1][1]],
    z: [goalEnds[0][2], goalEnds[1][2]],
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
  const middles = cands.map((cand) => barMiddle(cand.home_pos_mb, cand.home_quat_mb));
  return {
    type: 'scatter3d', mode: 'markers', name,
    x: middles.map((point) => point[0]),
    y: middles.map((point) => point[1]),
    z: middles.map((point) => point[2]),
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
  source.addEventListener('run_added', async (event) => {
    const { id, problem } = JSON.parse(event.data);
    await refreshProblems();  // the run counts changed
    // Another design problem's run: not listed, not opened, no toast.
    if (problem !== selectedProblem()) return;
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
$('problem').onchange = () => refreshRuns();
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

refreshProblems().then(() => refreshRuns());
connectEvents();
window.addEventListener('error', (event) => fail('page error', event.error || event.message));
