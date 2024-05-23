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

Left-Front#0 Left-Front#1 Left-Front#2 Left-Front#3
Left-Middle#0 Left-Middle#1 Left-Middle#2 Left-Middle#3
Left-Back#0 Left-Back#1 Left-Back#2 Left-Back#3
Right-Front#0 Right-Front#1 Right-Front#2 Right-Front#3
Right-Middle#0 Right-Middle#1 Right-Middle#2 Right-Middle#3
Right-Back#0 Right-Back#1 Right-Back#2 Right-Back#3

I know this is a lot, but it will allow us to control the forces better. The GUI should save/load all of them. You'll have to change the simulation to use the correct values.

### (b) Add a rolling ball!

(in development)