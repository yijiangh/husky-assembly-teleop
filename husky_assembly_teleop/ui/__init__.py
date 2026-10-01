"""
The operator's view, served by viser.
  visualization.py  the viser server, robots drawn each tick, one corner per plugin
  scene_view.py     scene bodies and tracked objects drawn from each tick's snapshot
  style.py          the shared look of panels: chips, section bars, number rows
  checklist.py      a selectable list widget
  pose_input.py     a floor-pose input: number fields plus a 3D gizmo
  ghost.py          see-through robot copies for targets and plans, shown while in use
  pybullet_window.py  a checkbox opening a plugin's PyBullet mirror in its own window
  quaternion.py     xyzw -> viser's wxyz, the one place the order flips
"""
