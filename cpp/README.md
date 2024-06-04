# C++ Task #1

Goal: Make a C++ version of the NEURON/Mujoco simulation from 2023. We want to see if it's faster than Python.

Steps: 
(a) Learn how to run Mujoco using C++.
(b) Learn how to run NEURON using C++.
(c) Recreate the rat-leg simulation from 2023 using C++.

## (a) Learn how to run Mujoco using C++.

Use the C++ API to run a simulation of the rat leg from 2023. Just apply a constant force to demonstrate that the leg will move. In Python, it would look like this:

data.ctrl[0] = 1

Note that I do NOT know anything about the C++ API! You'll have to research this.

## (b) Learn how to run NEURON using C++.

This is a totally new feature in NEURON that was literally just created this year, I think. Robert McDougal (fram Yale) has this repo:

https://github.com/mcdougallab/neuron-c-api-demos/tree/main

Read the instructions and try to run the following:
* demo1
* for-proposed-formal-api/helloworld.cpp
* for-proposed-formal-api/hh_sim.cpp

For hh_sim.cpp, it looks like it should save the output in hh_sim.csv. Use Excel to graph that data.

## (c) Recreate the rat-leg simulation from 2023 using C++.

Make a rat-leg simulation like what we did in 2023. However, no GUI, no settings files, and no graphs. You can use the settings here:

https://github.com/fietkiewicz/Linh-2023/blob/main/BodyBuilder/settings2.txt

