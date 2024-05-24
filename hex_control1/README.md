# Hexapod Task #2

Note: There are two programs.
1. hex_manual.py: Manual entry of parameters for force and angle. No steering during simulation.
2. hex_control.py: User steers using arrow keys. No manual control.

Short version: 
(a) Update GUI and add option to record video to file.
(b) Add input boxes to GUI for camera settings.
(c) Add a keyboard input to change direction (left, straight, right).

Long version: 

(a) Update GUI and add option to record video to file.

Box folder for videos and other temporary stuff:
https://hws.box.com/s/r8m8jcgtzirjo1lelxedv8o1n86q9f0r

GUI updates:
* Add radio button and input boxes for saving video
* Add labels for grid at top

(b) Add input boxes to GUI for camera settings.
* Select mjCAMERA_TRACKING or mjCAMERA_FIXED
* elevation  distance  trackbodyid
* Be sure to save/load all of these parameters.
* NEW: Make sure that default input file works.

(c) Add a keyboard input to change direction (left, straight, right).
* Try to use left-arrow = left, up-arrow = straight, right-arrow = right.
