"""Serve the M1 start-derivation diagnostics dashboard.

Leave this running for the whole session: it watches the runs folder and tells
any open page the moment a derivation writes a new run, so you can decide
whether to look at it.

Usage:
    cd /home/su/ros2_ws
    source venv/bin/activate
    source install/setup.bash
    python src/husky-assembly-teleop/scripts/m1_dashboard_server.py [--port 8765]

Run ``scripts/fetch_dashboard_vendor.sh`` once first -- the page needs three.js
and plotly, which are downloaded rather than committed.
"""
import argparse

from husky_assembly_teleop.dashboard.run_schema import runs_dir_default, scenes_dir_default
from husky_assembly_teleop.dashboard.server import serve


def main():
    """Parse the arguments and serve until interrupted."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--runs-dir', default=runs_dir_default(),
                        help='folder the derivations write run files into')
    parser.add_argument('--scenes-dir', default=scenes_dir_default(),
                        help='folder holding one baked scene per design problem')
    args = parser.parse_args()
    serve(runs_dir=args.runs_dir, scenes_dir=args.scenes_dir,
          port=args.port, host=args.host)


if __name__ == '__main__':
    main()
