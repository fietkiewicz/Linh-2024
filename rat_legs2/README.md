# Rat Legs Task #2

## Files

1. rat_hindlimbs_on_ground.xml: Chris' version of rat legs is much more detailed compared to Clayson's version
2. rat_walk.py: Old rat_walk.py in rat_legs, use the rat_hindlimb_both_legs.xml with negative friction to show better slippery state of the model
3. rat_walk_friction.py: Adapt from rat_walk_4.py in rat_legs, use the rat_hindlimbs_on_ground.xml with positive friction to show the model's stiffness.

## Introduction

The instructions were updated with a "(tutorial)" step.

The goal is to research how to use friction in Mujoco. 

Short version: (tutorial) Create an example from the tutorial. (a) Can we use friction in the foot to prevent slipping in Clayton's model? (b) How can we change friction in the floor? (c) Make a "walk" controller that lets the user change the friction settings.

Note that (a) and (b) do NOT need a controller. We'll test the friction using the default Mujoco viewer.

## Long version:

## (tutorial) Create an example from the tutorial. 

Run this tutorial example that has two cubes:
https://colab.research.google.com/github/deepmind/mujoco/blob/main/python/tutorial.ipynb#scrollTo=zV5PkYzFXu42

The goal for this part is to let the user change friction settings and see the results very fast. So make a GUI like you have in "rat_legs". It should have the following features:
1. Runs a simulation with the demo from the tutorial (two cubes).
2. Has an Entry for "ground" friction. Default is .1.
3. Has an Entry for the 2nd cube's friction. Default is .33.

## (a) Can we use friction in the foot to prevent slipping in Clayton's model? 

Research how to control friction and try to use this in Clayton's model so that the foot doesn't slip. We do NOT need a controller. We'll test it using the default Mujoco viewer. If this is possible, experiment to see if it's easier to make the rat legs walk.

## (b) How can we change friction in the floor?

Research how to control friction on the floor. We do NOT need a controller. If we can't change the friction of the floor, how can we add a surface or geometry on the floor with friction? If this is possible, experiment to see if it's easier to make the rat legs walk.

## (c) Make a "walk" controller that lets the user change the friction settings.

If part (a) worked, then make a controller that includes a setting (in the GUI) for the foot friction. If part (b) worked, add a setting for the floor friction. I want to be able to experiment with different friction settings to see how it affects the walking.

Please include the video recording option by the end.
