# Hexapod Task #4

I want to try making a control algorithm for the hexapod to follow the ball and push it around. The goal is to learn abount Mujoco sensors and also do someting fun with all of the work you've done for the robot to turn.

Short version: (a) Add range finders to find the ball. (b) Create a control algorithm that makes the robot walk into the ball.

Long version:

## (a) Add range finders to find the ball. 

I recorded a 15-minute demo of using the file "chris rangefinder demo.xml":

https://hws.hosted.panopto.com/Panopto/Pages/Viewer.aspx?id=8cd87d47-a13a-4614-8dc7-b181011fbe14

I don't know exactly how many we need. Technically, we could probably just use 1 rangefinder in the middle. With only one rangefinder in the middle, you could would need to turn left/right to decide which direction to go. With more range finders, you won't have to turn as many times to find the ball.

Note: In the video I mention that we may want to make the walls on the edges to be shorter so the rangefinder doesn't find those.

I don't know how to get the sensor values. This post:

https://github.com/openai/mujoco-py/issues/184

says you can use "data.sensordata". Maybe it's just a simple array and you can use data.sensordata[0] and data.sensordata[1], etc. I'm not sure, so you'll have to search and experiment with this.

## (b) Create a control algorithm that makes the robot walk into the ball.

The goal is to have the hexapod find the ball with the rangefinders and decide whether to turn left or right. If it's not finding the ball, it needs to keep turning. Somehow, it must start walking straight until it can't see the ball. Then it turns to find the ball again and walks straight. This may be very, very slow, but that's OK. 