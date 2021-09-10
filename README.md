# Introduction
This is Project #3 of the Udacity Robotics Software Nanodegree program. The objective is to exercise the AMCL ROS package. AMCL stand for Adaptive Monte Carlo Localization, a method using sensor data to determine the location, position and orientation, of a robot in a mapped environment. The package will be exercised within a simulation environment created in prior projects.

## Specific Objectives
There are numerous criteria to be met to complete this project. The following are the highlights:

- Create a ROS workspace containing the ROS AMCL package, simluation environment and a robot.
- Create launch files to bring up the simulation.
- Implement methods of driving the robot.
- Configure the RViz application to show status of localization.
- Demonstrate robot's ability to localize.

The simulation is carried out in the simulation environment created earlier and used in the prior project #2 Chase It. The simulation uses the turtlebot robot. Two methods of driving the robot are implemented, a manual method using the teleop keyboard package. By pressing keys the robot is commanded to move linearly or rotate. Speed and rate can be adjusted up or down by key press also.

Another method of robot activation is the move base package. This package receives navigation targets or destinations tthrough mouse clicks on the map in the RViz package. This package avoids obstacles an plan around them.

# Project Results
The following shows the results obtained for this project.

## Install & Run
The application was developed in Linux Ubuntu version 16.04 with ROS distribution Kinetic. The application depends on a few packages that must be present: navigation, map-server, move-base and amcl. These can be installed using,

> sudo apt install ros-kinetic-<name_of_required_package>
