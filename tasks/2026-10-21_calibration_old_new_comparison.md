# Old scripts and the plugin side by side (2026-10-21)

Status: **stages 1 and 2 done, not committed; stage 3 waits for the first lab run.**

## Goal

The calibration plugin keeps writing a session in the old formats, so the old offline scripts
(`data/calibration_data/0_…4_*.py`) run on it unchanged. The plugin's own analysis goes to a folder the old scripts
never touch, where new math can be tried later. One comparison tool runs both on a session and compares them, and
with a reference when there is one. It is used in three stages:

1. **Old real sessions** (pytest): Drive sessions recorded by the old monitor, frozen into test data, written in the
   plugin's format. The old scripts' result, the plugin's and the one stored on the Drive must agree.
2. **Sim**: a session the plugin recorded in the simulator, under a known calibration error. Both results are
   compared with the true calibration.
3. **Real new data** (at the first lab run): a session the plugin recorded on a robot. The two analyses must agree
   with each other; validation and punch say how good the result is.

## Decisions

- **Old scripts: yh's versions** (`origin/yh/multi-robot-monitor`), which have the motion-based axis signs. Only
  `config_loader.py` and `run_calibration_pipeline.py` change, as far as this branch needs: the sessions folder is
  `CALIBRATION_DATA_DIR` if set, else `<Drive>/data_experiment/calibration_data`; the session is `CALIBRATION_DATE`
  if set, else `DEFAULT_DATE_FOLDER`; the URDF is the one crl-husky's `model.yaml` names (the plugin's). The
  scripts' math stays as it is.
- **Dependencies:** `circle-fitting-3d` and `scikit-spatial` in the venv and `requirements.txt`.
- **The plugin's own analysis:** `<session>/analysis/<digits>_<arm>/` (`report.write_analysis`), written at Solve
  and after each validation spot: `solution.json` and `summary.txt`. The old formats and outputs stay where they are
  (`old_outputs.py`).
- **Comparison tool: `scripts/solve_calibration.py`**, extended rather than a new script: `--old-scripts` copies
  the takes the plugin solved (each into the folder of the joint it turns), the validation and punch takes and
  `config.yaml` to a temporary folder, runs the old scripts 0–4 there, and compares results and circles;
  `--truth-error` compares both with a simulated session's true calibration. The session is never written.
- **Test:** `test/plugins/calibration/test_calibration_old_scripts.py` (slow, ~75 s): Belle 20261013 and Cindy's
  right arm 20260625. Old script 0 alone takes ~30 s a session (circle_fitting_3d), so Cindy's left arm is left out.

## Results

| Session | Old scripts vs plugin | Plugin vs stored | vs truth |
|---|---|---|---|
| 20261013 Belle (Drive) | 0.000 mm, 0.0000° | 0.000 mm | — |
| 20261016 Cindy left (Drive) | 0.000 mm, 0.0000° | 0.062 mm (extra take in `j0/`) | — |
| 20260623 Alice (Drive) | 0.000 mm, 0.0000° | 0.000 mm | — |
| sim domain_72/20261021 Belle | 0.000 mm, 0.0000° | 0.026 mm (takes of five runs in `j0/`) | 0.480 mm, 0.018° both |

The 0.48 mm against the truth is the old math's known model bias (noise-free circles, axes 90.018° apart, 0.23 mm
skew on Belle's calibrated URDF).

## Open

- Stage 3: run `solve_calibration.py <session> --old-scripts` after the first lab session.
- Takes of earlier runs on the same day stay in `j0/` and `j1/`; the offline tools read them all.
