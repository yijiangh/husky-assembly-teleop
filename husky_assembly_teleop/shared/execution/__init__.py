"""
Running a planned movement on the real robots: timing, sending, tools, the operator's Confirm, and who holds a robot.

  retime.py     a planned joint path timed for the arm's trajectory controller, within its limits
  send.py       one motion command sent to a robot's arms and watched until it arrives, stops or times out
  tools.py      a Robotiq move and a scaffolding tool's grasp, watched until done, always stopped on the way out
  operator.py   the prompt the operator confirms or cancels, one at a time
  interlock.py  the one-motion-at-a-time lock across every robot, and other plugins' commands refused meanwhile
  basis.py      what a plan was made from, and why it no longer fits

! The functions that command a robot run on the main thread (as every robot command), from a plugin's task.
"""
