"""
Small example plugins, one idea each.

Read them in order:

  ui.py              example_ui           widgets, buttons, intents
  robot_state.py     example_robot_state  reading configuration and measured state
  pybullet_scene.py  example_pybullet     adding to, querying and cleaning up the PyBullet scene
  sequence.py        example_sequence     a long sequence with timers, Next and Cancel
  recording.py       example_recording    live plots and recordings of signals (ctx.trace, ctx.record)
  plot.py            example_plot         live plots with viser's uPlot widget

Then robot_control/, which puts these together for real robots. Importing this
package registers all six; each runs only when named in `-p plugins:=[...]`.
"""

from . import plot, pybullet_scene, recording, robot_state, sequence, ui  # noqa: F401  registers them
