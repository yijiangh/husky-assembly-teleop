"""
What a plugin is and what it is handed.

  plugin.py       HuskyPlugin, @register, and loading plugins in order
  context.py      PluginContext: a plugin's queue, tasks, scene and UI corner
  concurrency.py  asyncio tasks on the main thread, timeouts, waiting on ROS
  trace.py        Signal and Trace: values sampled once per tick, for live plots and recordings
  signals.py      ready-made signals of robots, links and tracked objects
"""
