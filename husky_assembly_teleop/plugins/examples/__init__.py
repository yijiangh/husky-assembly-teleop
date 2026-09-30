"""
Small example plugins, one idea each. Read them in order:

  ui.py              example_ui           widgets, buttons, intents
  robot_state.py     example_robot_state  reading configuration and measured state
  pybullet_scene.py  example_pybullet     adding to, querying and cleaning up the PyBullet scene
  sequence.py        example_sequence     a long sequence with timers, Next and Cancel
  recording.py       example_recording    recording a stream at the full ROS rate
  plot.py            example_plot         live plots with viser's uPlot widget

Then robot_control.py, which puts these together for real robots.

Importing this package registers all six; none runs unless named in `-p plugins:=[...]`.
"""

from .plot import ExamplePlotPlugin
from .pybullet_scene import ExamplePybulletPlugin
from .recording import ExampleRecordingPlugin
from .robot_state import ExampleRobotStatePlugin
from .sequence import ExampleSequencePlugin
from .ui import ExampleUiPlugin

__all__ = ["ExamplePlotPlugin", "ExamplePybulletPlugin", "ExampleRecordingPlugin", "ExampleRobotStatePlugin", "ExampleSequencePlugin", "ExampleUiPlugin"]
