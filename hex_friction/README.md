# Hexapod with friction

The goal is to add friction and mass to the hexapod and study how it affects walking. The hexapod walks, but

Short version: (a) Use the blocks friction demo to study mass. (b) Find a good online XML editor so we can test settings in the Mujoco app. (c) Add friction and mass to hexapod. (d) Analyze effects of changing mass, floor friction, and feet friction. (e) Make a block test with an actuator.

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

## (d) Analyze effects of changing mass, floor friction, and foot friction.

What happens when you increase mass friction in the hexapod?
What happens when you increase floor friction?
What happens when you increase foot friction?

## (e) Make a block test with an actuator.

The block test "friction_test_with_mass.py" is excellent! However, the old demo with the angled floor isn't like the robot leg. So I want a block on a flat floor that we can slide with an actuator. We'll apply a force to slide the block and test different values for friction and mass. Make a new program "friction_test_with_mass_2.py" with the following features:

1. Remove the live/video controls since we're not using them.
2. Use only one block. Remove the 2nd block.
4. Make the floor flat (not angled).
3. Start the block on the floor (I don't want it to drop down like the demo version).
5. Attach an actuator to the side of the block. (I emailed a picture to you.)
6. During the simulation, apply a constant force to the actuator.
7. Add a GUI control for the constant force value.
8. Print (on the screen) the position of the block each time the viewer is rendered. Print just one coordinate which is the value that's changing. I don't know if it will be x or y. I don't know if it will increase or decrease. For example, maybe the y-coordinate will be 2.8, 2.9, 3.0, 3.1 (just an example). The goal is to have values for the changing postion so that I can make a graph and see how fast the block is moving.
9. Collect the screen output (just numbers) for different friction/mass settings and make a graph to compare them. You can use Google sheets or MS Excel. Use two different settings for block friction, two different settings for floor friction, and two different settings for mass. Find a force value where the block moves for all the settings.
