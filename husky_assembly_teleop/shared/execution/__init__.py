"""
Running a planned movement on the real robots: timing, sending, tools, the operator's Confirm, whether anything moves.

  retime.py     a planned joint path timed for the arm's trajectory controller, within its limits
  send.py       one motion command sent to a robot's arms and watched until it arrives, stops or times out
  tools.py      a Robotiq move and a scaffolding tool's grasp, watched until done, always stopped on the way out
  operator.py   the prompt the operator confirms or cancels, one at a time
  moving.py     whether any arm or base moves, so an automatic motion starts only while nothing else does
  basis.py      what a plan was made from, and why it no longer fits
  servo.py      the servo's plan, each reading's errors, the MoCap bar's correction and the stop tests
  servo_record.py  the servo run's JSON record and its four-panel PNG

! The functions that command a robot run on the main thread (as every robot command), from a plugin's task.
"""
