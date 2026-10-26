"""
Running a planned movement on the real robots: timing, sending, tools, the operator's Confirm, the compliant parts.

  retime.py      a planned joint path timed for the arm's trajectory controller, within its limits
  send.py         one motion command sent to a robot's arms and watched until it arrives, stops or times out
  tools.py        a Robotiq move and a scaffolding tool's grasp, watched until done, always stopped on the way out
  operator.py     the prompt the operator confirms or cancels, one at a time
  moving.py       whether any arm or base moves, so an automatic motion starts only while nothing else does
  basis.py        what a plan was made from, and why it no longer fits
  controllers.py  the switch of a robot's arms to the compliance controller, and the way back every compliant part takes
  stream.py       Cartesian targets streamed along lines fixed in the world, the split of a path, a part's record
  spiral.py       the spiral searched at a compliant insert's end, until the screws stall
  precheck.py     the checks every compliant part runs before it sends anything
  insert.py       the insert: rigid under joint control, then compliant while the mating screws tighten
  retreat.py      the retreat: the tools let go and back out compliantly, then the rest replanned and run rigid
  handoff.py      the compliant Robotiq close on a bar another robot holds

! The functions that command a robot run on the main thread (as every robot command), from a plugin's task.
"""
