"""
The executor: runs one step of the assembly on its robot and records it in the run.

A motion runs on Exec; a tool step, a step by hand and the servo's first pass after the operator's Confirm.

  plugin.py     the plugin: takes requests, leases every robot while it works, records every part
  sequences.py  how each kind of part runs: a motion, a Robotiq move, a step by hand
  servo_part.py the servo part: pass by pass onto the authored targets, or the MoCap bar (spec M7)
  servo_measure.py  what the servo says and records around its loop: the gate after an insert, the panel line
  panel.py      its part of the Assembly tab and the phone's Exec tab: what runs, the prompt, Confirm, Cancel,
                the dry run, the servo line

What it offers (`MovementExecContract`) and the requests are `shared/movement_exec_contract.py`; timing, sending,
tools, the prompt, the motion watch and staleness are `shared/execution/`.
"""

from . import plugin  # noqa: F401  registers MovementExecPlugin
