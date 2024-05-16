# Hexapod Task #1

Short version: (a) Make the hexapod walk in circle forever. (b) Figure out how to set the camera angle automatically.

Long version:

(a) Walk in a circle

You’ll need to make one set of legs move faster (or farther) than the other set. One way is to use less force on the left-side legs (inside the circle), and another way is to use smaller angles for the horizontal joints. Either way can make the legs on the left side move less (either slower or less distance), as compared to the legs on the right side. This will require more parameters (more arrays).

I *strongly* recommend that you make a new GUI for all the parameters. (You can remove anything you don’t need, like start forces.) You can test new parameters easily by just changing the numbers is the GUI and clicking “Init and run”. Whenever you make progress, I recommend saving those settings (in the text file) so that you won’t forget them!

(b) Automatic camera view

The goal is to have the program *automatically* set the camera view. I think I want to be able to do two views:

1. Far away so that you can see the whole circle, like the picture above.

2. Directly overhead, looking straight down (not from the side).

I tried a lot last year but couldn’t do it the way that I wanted. The best I could do was move the camera closer, which doesn’t help for this model because it’s already super close. For the neuron models, I did this:

viewer = mujoco_viewer.MujocoViewer(legModel, data)
viewer.cam.fixedcamid = 0
viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED

Notice that the custom viewer lets you switch between cameras using the Tab key.

The xml file already has this:

<camera name="global"  pos="-10 10 10" xyaxes="-1 -1 0 1 0 1" mode="trackcom”/>

I think “0” (zero) is the default camera, but I don’t think I ever successfully got a program to select any other camera. Look online to see what you can find in the documentation. I found stuff like this:

https://github.com/google-deepmind/mujoco/issues/1007

But things I saw online didn’t work for me.