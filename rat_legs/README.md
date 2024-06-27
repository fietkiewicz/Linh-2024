# Rat Legs Task #1

Files...

rat_hindlimb_both_legs.xml: Original model from Clayton Jackson.
rat_hindlimbs_on_ground.xml: Newer model from Clayton Jackson.
rat_walk.py: First experiment using rat_hindlimb_both_legs.xml.
rat_walk_2.py: Second experiment using rat_hindlimb_both_legs.xml.
rat_walk_3.py: Walks backward! (Succssful failure!) Uses rat_hindlimbs_on_ground.xml.
rat_walk_4.py: Walks forward. Uses rat_hindlimbs_on_ground.xml.

test.py: Walks forward. New: Video recording feature captures live video with user camera changes.

## Background and original instructions

Clayton Jackson, the PhD student from Case Western Reserve University, made a 2-leg version of the rat legs that walks! His main github location is here (but you DON'T need to go there):

https://github.com/cxj271/Two_layer_CPG_SNS_Toolbox

Short version: (a) Discover how to move hip, knee, and ankle joints. (b) Make a screenshot video of the legs walking manually (no programming). (c) Make a controller that walks and has an option to record as a video.

## Long version:

## (a) Discover how to move hip, knee, and ankle joints

Use the Mujoco app and the control sliders. The knee and ankle only move in one direction. Note that Clayton designed it so that it doesn't fall over. Here's a picture:

https://github.com/fietkiewicz/Linh-2024/blob/main/rat_legs/example1.png

## (b) Make a screenshot video of the legs walking manually (no programming). 

Find a sequence of controller movements that makes it walk! This will be difficult and slow, but that's OK. Maybe you only need the hips to move, or maybe you need to move the knee and ankle. I don't know. (Clayton knows how to do this, but I want you to figure it out yourself.)

Make a screen recording that shows how you change the controllers. Upload the video to:

https://hws.app.box.com/folder/265034682217

## (c) Make a controller that walks and has an option to record as a video.

Using the sequence you found in part (b), make a Python controller with a GUI (like the hexapod), that allows the user to also record a video. It should have camera and video settings. Make a video and upload it to the Box folder above.
