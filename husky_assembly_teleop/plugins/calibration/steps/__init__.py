"""
The calibration's steps, one module each, numbered in their order (`STEPS`).

Each owns its widgets, checks, actions and drawing; `base.py` holds what they share (`Calibration`) and their base
class.

  s1_setup         robot arm, new or resumed run, tool body, checklist
  s2_marker_error  drive with the D-pad; both bodies' marker errors while moving; refine in Motive if too large
  s3_sweeps        the predefined takes checked for collisions by the arm planner
  s4_record        per take: planned move to its start (Go), then the sweep, a sample per waypoint, its circle
  s5_solve         the old pipeline's math; the session's files on the Drive, where the relay finds them
  s6_validate      the tool body in tool0 under the new calibration, at several base spots
  s7_apply         the relay reloads and applies the new calibration
  s8_tip           optional, for Punch: the punch tip's offset from poses touching one point, the base still
  s9_punch         optional: the punch tip put onto one fixed target from several base spots
  s10_overview     every step's verdict, the headline numbers, the figures written
"""

from .s1_setup import Setup
from .s2_marker_error import MarkerError
from .s3_sweeps import Sweeps
from .s4_record import Record
from .s5_solve import Solve
from .s6_validate import Validate
from .s7_apply import Apply
from .s8_tip import TipOffset
from .s9_punch import Punch
from .s10_overview import Overview

#: The steps, in order.
STEPS = (Setup, MarkerError, Sweeps, Record, Solve, Validate, Apply, TipOffset, Punch, Overview)
