# Rat Legs Task #2

The goal is to research how to use friction in Mujoco. 

Short version: (a) Can we use friction in the foot to prevent slipping in Clayton's model? (b) How can we change friction in the floor? (c) Make a "walk" controller that lets the user change the friction settings.

Note that (a) and (b) do NOT need a controller. We'll test the friction using the default Mujoco viewer.

## Long version:

## (a) Can we use friction in the foot to prevent slipping in Clayton's model? 

Research how to control friction and try to use this in Clayton's model so that the foot doesn't slip. We do NOT need a controller. We'll test it using the default Mujoco viewer. If this is possible, experiment to see if it's easier to make the rat legs walk.

## (b) How can we change friction in the floor?

Research how to control friction on the floor. We do NOT need a controller. If we can't change the friction of the floor, how can we add a surface or geometry on the floor with friction? If this is possible, experiment to see if it's easier to make the rat legs walk.

## (c) Make a "walk" controller that lets the user change the friction settings.

If part (a) worked, then make a controller that includes a setting (in the GUI) for the foot friction. If part (b) worked, add a setting for the floor friction. I want to be able to experiment with different friction settings to see how it affects the walking.
