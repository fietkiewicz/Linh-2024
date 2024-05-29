# Hexapod Task #3

Short version: 
(a) Add separate horizontal (yaw) force parameters for all 6 legs.
(b) Add a rolling ball!

## Long version: 

### (a) Add separate horizontal (yaw) force parameters for all 6 legs.

We were able to control the turns by using different left/right forces for horizontal (yaw) movement, but it was difficult to control. I believe this is because all 3 legs use the same force (example: 1 leg on left and 2 legs on right use the same force). What I wanted was to increase or decrease the force on only one side, but my design doesn't allow us to do that.

Currently, the GUI has 8 input boxes for horizontal (yaw) force:

Left,phase#0 Left,phase#1 Left,phase#2 Left,phase#3
Right,phase#0 Right,phase#1 Right,phase#2 Right,phase#3

Now I want 24 (6 legs for 4 phases), like this:

https://github.com/fietkiewicz/Linh-2024/blob/main/hex_control2/GUI%20example.png

I know this is a lot, but it will allow us to control the forces better. The GUI should save/load all of them. You'll have to change the simulation to use the correct values.

### (b) Add a rolling ball!

Add a ball that the robot can push like this:

https://www.youtube.com/watch?v=RhRLjbb7pBE&list=PLzWiCM7Cn8WYCrvMYGszduvFhkw-RvjAl&t=40s&ab_channel=YuvalTassa

