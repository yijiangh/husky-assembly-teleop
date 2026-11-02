"""
A process bar measured by mocap; registers no plugin itself.

The markers on a bar's connector halves, matched to Motive's markers by `world/marker_match.py`, give the bar's pose;
its deviation from the design drives the servo and the mocap_bars readout.

  layouts.py  each bar's marker layout from the process, its Motive asset (Bxx -> 10xx), its fit to sensed markers
  pose.py     a bar's pose as a matrix: deviation from its design, the servo's correction, start and ends
"""
