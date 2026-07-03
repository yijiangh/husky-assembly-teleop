# 2026-06-26 — Robust v0/v1 sign disambiguation in base-frame calibration

## Problem

Two symptoms, one root cause:
1. Robot lying on the floor (rolled 90°) in the live teleop PyBullet GUI.
2. `20260625` (right arm) calibration validation 100× worse: position-offset 95% CDF
   `0.96 mm` (20260622 left) → `500.65 mm`. j0/j1 raw fits clean, Motive residual 0.81 mm.

Proof both = same bug — `base_mocap_from_base_footprint` orientation:
- 20260622 (left): roll/pitch/yaw `[0.2°, 0.1°, 23.7°]` → upright, val 0.96 mm
- 20260625 (right): `[-89.9°, 0.3°, -156.7°]` → rolled 90°, val 500.65 mm

Live app loads the same file (`__init__.py:67 CALIBRATION_DATE='20260625'` →
`calibrated_transformation_0806_rhino.json` → `common.py:332`), so the rolled frame both
lays the robot down and makes the verifier place the base wrong.

## Root cause

`compute_base_frame()` (`data/calibration_data/1_calibration_analysis.py`) builds the base frame
from fitted joint-axis directions `v0` (j0), `v1` (j1). A fitted line has **no inherent sign** — the
fitter returns `v0`/`v1` with arbitrary direction. The per-arm hardcoded branches assumed a fixed
sign; the right-arm 20260625 fit came back flipped → frame rolled 90°. (Same class as the 2026-06-09
left-arm tilt fix.) The intended sign-disambiguation existed only as commented-out dead code, and it
compared frames that didn't match (v0/v1 in base_mocap vs URDF axes in inertia frame).

## Fix (implemented)

`1_calibration_analysis.py`:
1. `compute_base_frame(..., nominal_j0=None, nominal_j1=None)`: before frame construction, flip
   `v0`/`v1` to the same hemisphere as the URDF-nominal joint axes
   (`if np.dot(v, nominal) < 0: v = -v`). Fallback for v0 if no nominal: `v0[2] > 0`.
   The husky is upright so `base_footprint ≈ base_mocap` → a dot-sign (hemisphere) test is robust to
   the residual moderate base yaw.
2. Collapsed the now-identical dual-arm left/right branches into one arm-independent block; the old
   per-arm code is kept commented as a toggleable `#---`-delimited archive.
3. `main()`: compute `nominal_j0`/`nominal_j1` in the base_footprint frame via **PyBullet FK** at the
   zero config (joint axis rotated by its child-link frame). NOTE: `parse_joint_axes_from_urdf` is
   insufficient — it reads each joint's local axis (both come out `[0,0,1]`) and ignores the
   shoulder_pan→shoulder_lift chain rotation. The dual-UR5e arms sit on a ~45° bracket, so j0 is NOT
   vertical; the FK nominal captures that (`nominal_j0≈[0,-0.70,0.71]`, `nominal_j1≈[0,-0.71,-0.70]`,
   orthogonal).

No changes to `config_loader.py`, `3_verify_calibration.py`, or the live app.

## Result (regenerated 20260625)

- `base_mocap_from_base_footprint` rpy now `[0.4°, 0.1°, 23.8°]` (upright), matching 20260622
  `[0.2°, 0.1°, 23.7°]`; positions match too.
- `3_verify_calibration.py`: position max-from-mean **1.12 mm** (std 0.85 mm), was 500.65 mm → under
  the 5 mm goal. Angular max ~0.5°.
- Live app needs no change — it loads the regenerated file, robot now upright (Z-up).

## How to run

```
source /home/su/ros2_ws/venv/bin/activate
cd /home/su/ros2_ws/src/husky-assembly-teleop/data/calibration_data
MPLBACKEND=Agg python 1_calibration_analysis.py
MPLBACKEND=Agg python 2_convert_and_visualize_transformation.py
MPLBACKEND=Agg python 3_verify_calibration.py
```
(config_loader DEFAULT_DATE_FOLDER='20260625', 20260625/config.yaml arm: right.)

## Follow-ups / notes

- The `v0`/`v1` hemisphere test assumes the husky-rigidbody (base_mocap) yaw is within 90° of
  base_footprint. True for current mounting; if a future rig violates it, use the full nominal
  rotation match instead of two independent dot tests.
- Regenerated files (modified, untracked) under `20260625/` should be committed when the user is ready.
