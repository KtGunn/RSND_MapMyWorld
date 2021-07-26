# Project2-ChaseIt
This is project #2 of the Udacity Robotics Software Nano Degree program. The objective is to create a model of a simple robot and equip it with a lidar scanner and RGB camera. It is then placed in the simulation environment of Project 1. A white ball object is created and also placed in the simulation environment. If the robot's camera sees the white ball the robot is to chase it.

## Specific Requirements
To accomplish the project two packages are to be created: my_robot and ball_chaser.

### my_robot
The package is to contain a robot, the world created in Project #1 and a white ball object. The robot is a differetial drive robot. This is an exercise in using URDF to describe the robot. A base model is given for reference but is to be augmented. In my design it was augmented by coloring its various components. The robot is to be equipped with a lidar scanner and rgb camera.

![my_robot](</workspace/images/my_robot.png>)

The robot is to be housed in the simulation environment of Project #1. In addition a white ball object is to be created and also housed in the simulation environment. Finally, a launch file, world.launch is to be created that brings the package up with the robot and ball included in the simulation environment.

### ball_chaser
The ball_chaser package will contain two nodes. First node is drive_bot which provides a service to drive the robot called 'ball_chaser/command_robot.' This command takes two arguments, forward speed and rotational rate about the vertical axis. The service publishes to wheel joints of my_robot.

The second node, process_image, subscribes to images from the camera. As images are received they are processed to find the white ball if is is in the image. If the ball is detected, process_image requests service from drive_robot to move toward the ball. A launch file, ball_chaser.launch is created to bring up the two nodes of the ball_chaser package.

# Installation and Running
These are the step to run the project on an Ubuntu system:

- clone the repository;
- remove the build and devel directories under the catkin_ws directory;
- execute the 'catkin_make' command.

After the project has been compiled:

- in console window execute
> roslaunch my_robot world.launch
This brings up RViz and Gazebo with the robot and white ball.

- in another console exeucte
> roslaunch ball_chaser ball_chaser_launch
This brings up the nodes that activate the robot to chase the white ball.

# Installation and Running
The ball_chaser package is designed to command the robot to turn counter-clockwise in place until the white ball is detected. When the white ball comes into view, the system ensures the whole ball is within view, then moves in the direction of the white ball. The homing strategy uses a fixed forward speed, and a simple proportional control of rotational rate with max/min limits clamping. Heading control seeks to keep the ball in the center of view. If the ball goes out of view, the robot stops forward motion and commences fixed counter-clockwise rotation until the ball is seen again.

The robot will not stop when it closes in on the ball. It will push the ball. This can lead to interesting events. The images below show the robot climbing up on the ball

![climbing](</workspace/images/bot_climbing_ball.png>)

and eventually flip over on its back.

![climbing](</workspace/images/bot_rolled_over.png>)


