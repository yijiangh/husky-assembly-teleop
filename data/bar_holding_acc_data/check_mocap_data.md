Conclusion:
- Don't forget to turn on the labeled and unlabled markers in Motive's streaming setting panel
- labeled (from markerset defined by rb, created auto) and unlabled markers are returning the exact same data
- after creating a rb, the marker data returned from the rb data description is NOT the same as the raw marker data. Motive does some solving process on the data, which is a fixed relative position to the rb pose, which doesn't change no matter rb position.


 # from rb data description
	#   0 Marker Label: 0 Position: [-0.47 -0.03 -0.03] Marker1
	#   1 Marker Label: 0 Position: [-0.27 -0.04 0.02] Marker2
	#   2 Marker Label: 0 Position: [-0.47 0.02 0.04] Marker3
	#   3 Marker Label: 0 Position: [-0.27 0.03 -0.02] Marker4
	#   4 Marker Label: 0 Position: [0.47 -0.04 0.01] Marker5
	#   5 Marker Label: 0 Position: [0.47 0.05 -0.01] Marker6
	#   6 Marker Label: 0 Position: [0.27 0.01 0.04] Marker7
	#   7 Marker Label: 0 Position: [0.27 -0.00 -0.04] Marker8

 # from labeled markers and markerset

 Model Name      : bar rig
Marker Count    : 8
	Marker   0 : [0.04,0.10,-0.15]
	Marker   1 : [-0.16,0.07,-0.09]
	Marker   2 : [0.04,0.02,-0.12]
	Marker   3 : [-0.17,0.04,-0.17]
	Marker   4 : [-0.91,0.08,-0.07]
	Marker   5 : [-0.91,0.02,-0.13]
	Marker   6 : [-0.71,0.02,-0.08]
	Marker   7 : [-0.71,0.08,-0.14]
Model Name      : all
Marker Count    : 14
	Marker   0 : [0.15,0.13,0.84]
	Marker   1 : [-0.13,0.10,0.81]
	Marker   2 : [-0.12,0.10,0.75]
	Marker   3 : [-0.06,0.09,0.83]
	Marker   4 : [-0.04,0.09,0.77]
	Marker   5 : [0.14,0.13,0.87]
	Marker   6 : [0.04,0.10,-0.15]
	Marker   7 : [-0.16,0.07,-0.09]
	Marker   8 : [0.04,0.02,-0.12]
	Marker   9 : [-0.17,0.04,-0.17]
	Marker  10 : [-0.91,0.08,-0.07]
	Marker  11 : [-0.91,0.02,-0.13]
	Marker  12 : [-0.71,0.02,-0.08]
	Marker  13 : [-0.71,0.08,-0.14] 

ID     : [MarkerID:   1] [ModelID: 4570]
  pos  : [0.04, 0.10, -0.15]
  size : [0.02]
  err  : [0.00]
ID     : [MarkerID:   2] [ModelID: 4570]
  pos  : [-0.16, 0.07, -0.09]
  size : [0.02]
  err  : [0.00]
ID     : [MarkerID:   3] [ModelID: 4570]
  pos  : [0.04, 0.02, -0.12]
  size : [0.02]
  err  : [0.00]
ID     : [MarkerID:   4] [ModelID: 4570]
  pos  : [-0.17, 0.04, -0.17]
  size : [0.02]
  err  : [0.00]
ID     : [MarkerID:   5] [ModelID: 4570]
  pos  : [-0.91, 0.08, -0.07]
  size : [0.02]
  err  : [0.00]
ID     : [MarkerID:   6] [ModelID: 4570]
  pos  : [-0.91, 0.02, -0.13]
  size : [0.02]
  err  : [0.00]
ID     : [MarkerID:   7] [ModelID: 4570]
  pos  : [-0.71, 0.02, -0.08]
  size : [0.02]
  err  : [0.00]
ID     : [MarkerID:   8] [ModelID: 4570]
  pos  : [-0.71, 0.08, -0.14]
  size : [0.02]
  err  : [0.00]


  RB:   2 ID: 4570
	Position    : [-0.44, 0.05, -0.12]
	Orientation : [0.02, -0.28, 0.96, 0.00]
	Marker Error: 0.00
	Tracking Valid: True

# By unchecking the rb set from asset panel

Marker Set Count:1
Model Name      : all
Marker Count    : 0
Unlabeled Markers Count:9
	Marker   0 : [-0.91,0.08,-0.07]
	Marker   1 : [-0.71,0.08,-0.14]
	Marker   2 : [-0.17,0.04,-0.17]
	Marker   3 : [-0.16,0.07,-0.09]
	Marker   4 : [0.04,0.10,-0.15]
	Marker   5 : [0.04,0.02,-0.12]
	Marker   6 : [-0.71,0.02,-0.08]
	Marker   7 : [-0.91,0.02,-0.13]
	Marker   8 : [-1.43,0.48,-4.56]
Rigid Body Count:1
RB:   0 ID:   0
	Position    : [0.00, 0.00, 0.00]
	Orientation : [0.00, 0.00, 0.00, 1.00]
	Marker Error: 0.00
	Tracking Valid: False
Skeleton Count:0
Labeled Marker Count:9
ID     : [MarkerID:   1] [ModelID:   0]
  pos  : [-0.91, 0.08, -0.07]
  size : [0.02]
  err  : [0.00]
ID     : [MarkerID:   2] [ModelID:   0]
  pos  : [-0.71, 0.08, -0.14]
  size : [0.02]
  err  : [0.00]
ID     : [MarkerID:   3] [ModelID:   0]
  pos  : [-0.17, 0.04, -0.17]
  size : [0.02]
  err  : [0.00]
ID     : [MarkerID:   4] [ModelID:   0]
  pos  : [-0.16, 0.07, -0.09]
  size : [0.02]
  err  : [0.00]
ID     : [MarkerID:   5] [ModelID:   0]
  pos  : [0.04, 0.10, -0.15]
  size : [0.02]
  err  : [0.00]
ID     : [MarkerID:   6] [ModelID:   0]
  pos  : [0.04, 0.02, -0.12]
  size : [0.02]
  err  : [0.00]
ID     : [MarkerID:   7] [ModelID:   0]
  pos  : [-0.71, 0.02, -0.08]
  size : [0.02]
  err  : [0.00]
ID     : [MarkerID:   8] [ModelID:   0]
  pos  : [-0.91, 0.02, -0.13]
  size : [0.02]
  err  : [0.00]
ID     : [MarkerID:  10] [ModelID:   0]
  pos  : [-1.43, 0.48, -4.56]
  size : [0.01]
