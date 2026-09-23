"""A small always-on web server for the M1 derivation dashboard.

Start it once per session; it watches the runs folder and pushes a
notification to any open page as soon as a derivation writes a new run, so the
operator can decide whether to look at it. Everything is stdlib: a threading
HTTP server, and Server-Sent Events (a plain text/event-stream response the
browser reconnects to on its own) for the notifications.

Endpoints
    /                                      the page
    /api/runs                              one summary line per run, newest first
    /api/runs/<id>                         the whole run, plus the sentences
    /api/runs/<id>/candidates/<i>/frames   poses for the 3D viewer
    /api/runs/<id>/robot?conf=goal|start   the robot as triangles, base frame
    /api/scenes/<problem>/scene.glb        the baked scene geometry
    /events                                notifications (Server-Sent Events)
"""
import json
import os
import queue
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlparse

from husky_assembly_teleop.dashboard.kinematics import SceneKinematics
from husky_assembly_teleop.dashboard.run_schema import (
    describe_candidate, describe_run, runs_dir_default, scenes_dir_default,
)

def _static_dir():
    """Where to serve the page from, preferring the source tree.

    colcon copies the package into ``build/``, so the module that is running is
    usually NOT the one in ``src/`` -- and the vendored javascript is downloaded
    into ``src/``. Serving the source copy when it exists means a page edit or a
    vendor download takes effect on the next reload, with no rebuild. Mirrors
    what ``husky_assembly_teleop._get_data_directory`` does for the data folder.

    Returns:
        str: the ``static`` folder to serve.
    """
    from husky_assembly_teleop import DATA_DIRECTORY
    source = os.path.abspath(os.path.join(
        DATA_DIRECTORY, '..', 'husky_assembly_teleop', 'dashboard', 'static'))
    if os.path.isdir(source):
        return source
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')


STATIC_DIR = _static_dir()
# Without these the page cannot render at all, so say so at startup rather than
# leaving the operator with a blank tab and a console error.
# ! Every file here is imported by another one: three.module pulls three.core,
# ! and GLTFLoader pulls BufferGeometryUtils AND SkeletonUtils. A missing file
# ! breaks the whole module graph and the page renders nothing at all, so the
# ! list must stay complete.
REQUIRED_VENDOR = (
    'vendor/three-r185/build/three.module.min.js',
    'vendor/three-r185/build/three.core.min.js',
    'vendor/three-r185/examples/jsm/controls/OrbitControls.js',
    'vendor/three-r185/examples/jsm/loaders/GLTFLoader.js',
    'vendor/three-r185/examples/jsm/utils/BufferGeometryUtils.js',
    'vendor/three-r185/examples/jsm/utils/SkeletonUtils.js',
    'vendor/plotly.min.js',
)


class RunStore:
    """Reads run files from the watched folder, newest first.

    Args:
        runs_dir (str): folder the producers write into.
        scenes_dir (str): folder holding one scene per design problem.
    """

    def __init__(self, runs_dir, scenes_dir):
        self.runs_dir = runs_dir
        self.scenes_dir = scenes_dir
        self._kinematics = None
        self._scene_nodes = {}
        # (run id, 'goal' | 'start') -> the robot triangle soup. The page asks
        # for the same two soups over and over as the operator toggles the
        # overlay, and each one costs a few thousand matrix multiplies.
        self._robot_soups = {}
        self._lock = threading.Lock()

    def kinematics(self):
        """The shared forward-kinematics helper (built on first use)."""
        with self._lock:
            if self._kinematics is None:
                from husky_assembly_teleop.cfab_session import HUSKY_DUAL_URDF_PATH
                self._kinematics = SceneKinematics(HUSKY_DUAL_URDF_PATH)
            return self._kinematics

    def list_runs(self):
        """Summary lines for every run on disk, newest first."""
        summaries = []
        for name in os.listdir(self.runs_dir) if os.path.isdir(self.runs_dir) else []:
            if not name.endswith('.json'):
                continue
            try:
                run = self.load(name[:-len('.json')])
            except Exception as exc:
                print(f'[dashboard] skipping unreadable run {name}: {exc}')
                continue
            result = run.get('result') or {}
            summaries.append({
                'id': run.get('id'), 'created': run.get('created'),
                'problem': run.get('problem'), 'bar_action': run.get('bar_action'),
                'anchor_selection': run.get('anchor_selection'),
                'source': run.get('source'),
                'result_kind': result.get('kind'),
                't_found_s': result.get('t_found_s'),
                't_total_s': (run.get('profile') or {}).get('t_total'),
                'n_candidates': len(run.get('candidates') or []),
            })
        summaries.sort(key=lambda entry: entry.get('created') or '', reverse=True)
        return summaries

    def load(self, run_id):
        """One run record straight from disk."""
        path = os.path.join(self.runs_dir, f'{run_id}.json')
        with open(path) as handle:
            return json.load(handle)

    def load_described(self, run_id):
        """One run record with the plain-language sentences added.

        Args:
            run_id (str): the run's id.

        Returns:
            dict: the run, with ``summary`` and per-candidate ``text``.
        """
        run = self.load(run_id)
        run['summary'] = describe_run(run)
        for cand in run.get('candidates') or []:
            cand['text'] = describe_candidate(run, cand)
        return run

    def scene_nodes(self, problem):
        """The node names one problem's exported scene contains (cached).

        Args:
            problem (str): design problem name.

        Returns:
            set[str] | None: the node names, or None when the scene is missing
            (the viewer then simply finds nothing for the extra names).
        """
        if problem not in self._scene_nodes:
            path = os.path.join(self.scenes_dir, problem, 'scene_nodes.json')
            try:
                with open(path) as handle:
                    self._scene_nodes[problem] = {
                        node['name'] for node in json.load(handle)['nodes']}
            except (OSError, ValueError, KeyError):
                self._scene_nodes[problem] = None
        return self._scene_nodes[problem]

    def frames(self, run_id, index):
        """Viewer poses for one candidate of one run."""
        run = self.load(run_id)
        return self.kinematics().frames_for_candidate(
            run, index, scene_nodes=self.scene_nodes(run.get('problem')))

    def robot_soup(self, run_id, conf='goal'):
        """The robot drawn as triangles, in the mobile-base frame (cached).

        Args:
            run_id (str): the run's id.
            conf (str): ``'goal'`` for the configuration the sweep walked back
                FROM, or ``'start'`` for the bar-loading pose it settled on.

        Returns:
            dict: ``{conf, n_triangles, x, y, z, i, j, k}`` -- the six arrays a
            plotly ``mesh3d`` trace needs, plus which configuration they show.

        Raises:
            ValueError: for an unknown ``conf``, or when the run derived no
                start configuration.
        """
        if conf not in ('goal', 'start'):
            raise ValueError(f'unknown robot configuration {conf!r}')
        key = (run_id, conf)
        if key not in self._robot_soups:
            run = self.load(run_id)
            if conf == 'start':
                conf12 = (run.get('result') or {}).get('start_conf')
                if not conf12:
                    raise ValueError('this run derived no start configuration')
            else:
                conf12 = run['context']['goal']['conf']
            soup = self.kinematics().robot_triangles(
                conf12, run['context']['joint_names_12'])
            soup['conf'] = conf
            self._robot_soups[key] = soup
        return self._robot_soups[key]


class SseHub:
    """Fans notifications out to every connected page."""

    def __init__(self):
        self._clients = []
        self._lock = threading.Lock()

    def subscribe(self):
        """Register a new page; returns the queue its events arrive on."""
        client = queue.Queue()
        with self._lock:
            self._clients.append(client)
        return client

    def unsubscribe(self, client):
        """Drop a page that has gone away."""
        with self._lock:
            if client in self._clients:
                self._clients.remove(client)

    def broadcast(self, event, payload):
        """Send one event to every connected page."""
        with self._lock:
            clients = list(self._clients)
        for client in clients:
            client.put((event, payload))


class RunsWatcher:
    """Tells the hub whenever a new run file lands in the folder.

    Uses ``watchdog`` when it is importable and falls back to polling, so the
    dashboard works even in an environment without it.

    Args:
        runs_dir (str): the folder to watch.
        hub (SseHub): where to announce new runs.
    """

    def __init__(self, runs_dir, hub):
        self.runs_dir = runs_dir
        self.hub = hub
        self._seen = set()

    def _scan(self, announce=True):
        """Announce every run file not seen before."""
        try:
            names = os.listdir(self.runs_dir)
        except OSError:
            return
        for name in sorted(names):
            if not name.endswith('.json') or name in self._seen:
                continue
            self._seen.add(name)
            if announce:
                print(f'[dashboard] new run: {name}')
                self.hub.broadcast('run_added', {'id': name[:-len('.json')]})

    def start(self):
        """Begin watching (never raises; falls back to polling)."""
        os.makedirs(self.runs_dir, exist_ok=True)
        self._scan(announce=False)          # runs already on disk are not "new"
        try:
            from watchdog.events import FileSystemEventHandler
            from watchdog.observers import Observer
        except ImportError:
            print('[dashboard] watchdog not available; polling the runs folder.')
            threading.Thread(target=self._poll, daemon=True).start()
            return

        watcher = self

        class _Handler(FileSystemEventHandler):
            """Rescans on any change; the producer renames a .tmp into place."""

            def on_any_event(self, event):
                watcher._scan()

        observer = Observer()
        observer.schedule(_Handler(), self.runs_dir, recursive=False)
        observer.daemon = True
        observer.start()

    def _poll(self):
        """Polling fallback for environments without watchdog."""
        while True:
            self._scan()
            time.sleep(2.0)


def make_handler(store, hub):
    """Build the request handler class bound to one store and hub.

    Args:
        store (RunStore): the run store to serve from.
        hub (SseHub): the notification hub.

    Returns:
        type: a ``SimpleHTTPRequestHandler`` subclass.
    """

    class DashboardHandler(SimpleHTTPRequestHandler):
        """Serves the page, the run API and the notification stream."""

        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=STATIC_DIR, **kwargs)

        def log_message(self, fmt, *args):
            """Quiet by default -- the interesting lines are printed elsewhere."""

        def _send_json(self, payload, status=200):
            """Write one JSON response."""
            body = json.dumps(payload).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _send_file(self, path, content_type):
            """Write one file response, or 404 with a hint."""
            if not os.path.isfile(path):
                self._send_json({'error': f'{os.path.basename(path)} not found',
                                 'hint': 'run scripts/derive_m1_headless.py once '
                                         'to export the scene for this problem'}, 404)
                return
            with open(path, 'rb') as handle:
                body = handle.read()
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _serve_events(self):
            """Hold the connection open and forward notifications."""
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            client = hub.subscribe()
            try:
                self.wfile.write(b': connected\n\n')
                self.wfile.flush()
                while True:
                    try:
                        event, payload = client.get(timeout=15)
                        message = (f'event: {event}\n'
                                   f'data: {json.dumps(payload)}\n\n')
                    except queue.Empty:
                        message = ': keep-alive\n\n'   # stops proxies timing out
                    self.wfile.write(message.encode('utf-8'))
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                hub.unsubscribe(client)

        def do_GET(self):
            """Route API calls; everything else is a static file."""
            parsed = urlparse(self.path)
            path = unquote(parsed.path)
            # ! Only this handler's robot route carries a query string, and
            # ! ``parsed.path`` has already dropped it -- so read it here.
            query = parse_qs(parsed.query)
            try:
                if path == '/events':
                    self._serve_events()
                    return
                if path == '/api/runs':
                    self._send_json(store.list_runs())
                    return
                if path.startswith('/api/runs/'):
                    rest = path[len('/api/runs/'):]
                    parts = rest.split('/')
                    if len(parts) == 1:
                        self._send_json(store.load_described(parts[0]))
                        return
                    if len(parts) == 4 and parts[1] == 'candidates' and parts[3] == 'frames':
                        self._send_json(store.frames(parts[0], int(parts[2])))
                        return
                    if len(parts) == 2 and parts[1] == 'robot':
                        self._send_json(store.robot_soup(
                            parts[0], query.get('conf', ['goal'])[0]))
                        return
                if path.startswith('/api/scenes/'):
                    rest = path[len('/api/scenes/'):]
                    target = os.path.normpath(os.path.join(store.scenes_dir, rest))
                    if not target.startswith(os.path.abspath(store.scenes_dir)):
                        self._send_json({'error': 'bad scene path'}, 400)
                        return
                    self._send_file(target, 'model/gltf-binary' if target.endswith('.glb')
                                    else 'application/json')
                    return
            except FileNotFoundError:
                self._send_json({'error': 'not found'}, 404)
                return
            except Exception as exc:
                self._send_json({'error': str(exc)}, 500)
                return
            super().do_GET()

    return DashboardHandler


def serve(runs_dir=None, scenes_dir=None, port=8765, host='127.0.0.1'):
    """Run the dashboard until interrupted.

    Args:
        runs_dir (str | None): folder to watch (default: the producers' one).
        scenes_dir (str | None): scene cache root.
        port (int): TCP port.
        host (str): interface to bind.
    """
    runs_dir = os.path.abspath(runs_dir or runs_dir_default())
    scenes_dir = os.path.abspath(scenes_dir or scenes_dir_default())
    missing = [name for name in REQUIRED_VENDOR
               if not os.path.isfile(os.path.join(STATIC_DIR, name))]
    if missing:
        raise SystemExit(
            'The dashboard\'s javascript libraries are missing:\n  '
            + '\n  '.join(missing)
            + '\nRun scripts/fetch_dashboard_vendor.sh once to download them.')

    hub = SseHub()
    RunsWatcher(runs_dir, hub).start()
    server = ThreadingHTTPServer((host, port), make_handler(
        RunStore(runs_dir, scenes_dir), hub))
    server.daemon_threads = True
    print(f'[dashboard] serving http://{host}:{port}')
    print(f'[dashboard] watching runs in {runs_dir}')
    print(f'[dashboard] scenes from  {scenes_dir}')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\n[dashboard] stopped.')
