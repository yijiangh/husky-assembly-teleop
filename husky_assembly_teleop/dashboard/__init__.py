"""Diagnostics dashboard for the M1 start derivation.

The planner's M1 "derive start" sweep tries a thousand home bar poses and
usually spends its whole 120 s budget doing it. This package turns one such
sweep into something readable:

- ``run_schema``   what a run file holds, and how to say it in plain English.
- ``scene_export`` one glTF scene per design problem (robot + tools + bodies).
- ``run_writer``   producer side: build a run file from a derivation result.
- ``kinematics``   server side: forward kinematics -> per-candidate frames.
- ``server``       a stdlib HTTP server + live notifications for the page.

Run ``scripts/m1_dashboard_server.py`` to serve it; the monitor's
"M1: Derive Start/Goal only" button and ``scripts/derive_m1_headless.py``
both write runs into the watched folder.
"""
