"""Console summary of one M1 start derivation.

``derive_constrained_start_tracked`` (husky_assembly_tamp) walks the held bar
backward from the goal pose toward many candidate "home" poses (carry anchor x
orientation variant x position delta) and keeps the first one that arrives
collision-free. Even a successful sweep can burn its whole time budget on
candidates that die early, and the counters alone do not say where. The planner
records one entry per candidate plus a timing profile; this prints them as a
table you can read in the terminal.

The full picture -- which link hit which body, where every home pose sat, and
the scene in 3D -- lives in the web dashboard (``husky_assembly_teleop.dashboard``,
served by ``scripts/m1_dashboard_server.py``); this stays as the quick look.
"""
from collections import Counter, OrderedDict

import numpy as np

from husky_assembly_teleop.dashboard.run_schema import OUTCOMES as OUTCOME_ORDER


def _tracked(info: dict) -> dict:
    """The sweep's own info block (counters + candidates + profile), or {}."""
    return (info or {}).get('tracked') or {}


def print_m1_derivation_summary(info: dict) -> None:
    """Print the outcome table per home variant and the timing profile.

    Args:
        info: The ``info`` dict returned by
            ``_derive_constrained_start_for_plan`` (success or failure).
    """
    tracked = _tracked(info)
    candidates = tracked.get('candidates') or []
    profile = tracked.get('profile') or {}
    stage_t = (info or {}).get('stage_times') or {}
    if not candidates and not profile:
        print("[M1 derive report] no tracked-sweep trace in info (cold sweep only?).")
        return

    print("\n---------------- M1 start derivation: candidates per home variant ----------------")
    # Keep the variants in the order they were tried.
    by_variant: "OrderedDict[str, Counter]" = OrderedDict()
    for c in candidates:
        by_variant.setdefault(c['variant'], Counter())[c['outcome'] or 'unknown'] += 1
    header = f"{'variant':<22}" + "".join(f"{o[:14]:>16}" for o in OUTCOME_ORDER) + f"{'reached(med)':>14}"
    print(header)
    for variant, counts in by_variant.items():
        reached = [c['reached'] for c in candidates if c['variant'] == variant]
        med = float(np.median(reached)) if reached else 0.0
        print(f"{variant:<22}" + "".join(f"{counts.get(o, 0):>16d}" for o in OUTCOME_ORDER)
              + f"{med:>14.2f}")
    cuts = profile.get('budget_cuts') or []
    if cuts:
        # Each cut is (anchor, variant, candidates tried so far); older runs
        # carry only the first two.
        print("time share ran out for: "
              + ", ".join(f"{cut[0]} at {cut[1]}"
                          + (f" after {cut[2]} candidates" if len(cut) > 2 else "")
                          for cut in cuts))

    print("---------------- where the time went ----------------")
    t_total = float(profile.get('t_total', 0.0))
    t_track = float(profile.get('t_track', 0.0))
    t_fine = float(profile.get('t_fine_retrack', 0.0))
    t_cc = float(profile.get('t_cc', 0.0))
    n_ik = int(profile.get('n_ik', 0))
    n_cc = int(profile.get('n_cc', 0))
    budget = float(profile.get('max_time', 0.0))
    other = max(0.0, t_total - t_track - t_fine - t_cc)
    print(f"sweep wall time {t_total:6.1f} s of a {budget:.0f} s budget "
          f"({100.0 * t_total / budget if budget else 0:.0f} %)")
    print(f"  per-waypoint IK (screening) {t_track:6.1f} s  {n_ik:6d} solves  "
          f"{1000.0 * t_track / max(1, n_ik):6.1f} ms each")
    print(f"  fine re-track of a winner   {t_fine:6.1f} s")
    print(f"  collision checks            {t_cc:6.1f} s  {n_cc:6d} checks  "
          f"{1000.0 * t_cc / max(1, n_cc):6.1f} ms each")
    print(f"  everything else             {other:6.1f} s")
    spent = profile.get('anchor_spent') or {}
    allowance = float(profile.get('anchor_allowance', 0.0))
    for anchor, t in spent.items():
        print(f"  anchor {anchor:<11} {t:6.1f} s of its {allowance:.0f} s share")
    if stage_t:
        print("api stages: " + ", ".join(f"{k} {v:.1f} s" for k, v in stage_t.items()))
    print("---------------------------------------------------------------------------------\n")
