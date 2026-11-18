"""
A process bar measured by mocap; registers no plugin itself.

The markers on a bar's connector halves, matched to the raw markers near where the bar should be by
`world/marker_match.py`, give the bar's pose; its deviation from the design drives the servo and the mocap_bars readout.

  layouts.py  each bar's marker layout from the process, its fit to the raw markers near it, its drawing
  pose.py     a bar's pose as a matrix: deviation from its design, the servo's correction, start and ends
  readings.py the old monitor's readings of a fit: each marker, each plate's ball pair, the console lines
"""
