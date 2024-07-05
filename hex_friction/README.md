# Hexapod with friction

The goal is to add friction and mass to the hexapod and study how it affects walking. The hexapod walks, but

Short version: (a) Use the blocks friction demo to study mass. (b) Find a good online XML editor so we can test settings in the Mujoco app. (c) Add friction and mass to hexapod.

## Long version:

## (a) Use the blocks friction demo to study mass. 

Copy the awesome friction demo from rat_legs2. Then add a setting for mass to the 2nd cube. Theoretically, I think the cube should slide less when the mass is greater. For example, if we increase the mass, I think it should increase the effect of friction, so maybe the block will not slide as fast. I'm not sure.

## (b) Find a good online XML editor so we can test settings in the Mujoco app. 

The Mujoco app is great for experimenting with the model by manually changing the controllers. Answer the following questions: (1) Does the app allow us to change the model settings in the XML file, like the friction setting? There's a "save" feature, but what does it do? (2) Assuming we can't change settings in the app, find a good online XML editor for this. Try different editors and write what you like and don't like about them. Note that I want an online editor so that we can both use the same editor (not something we install).

## (c) Add friction and mass to hexapod.

In this part, I want the GUI with manual control (left/right/straight) and the live/video/camera options. We'll experiment with friction and mass settings to see how they affect walking.

Add the following to the hexapod XML and the GUI control panel:

1. mass for the hexapod "torso"
2. friction for the floor
3. friction for all of the feet
