# CleanUpRobot
- Implemented TurtleBot3 Waffle robot using Robot Operating System (ROS) in a Gazebo simulated environment.
- Tasks: 
	- to semi-autonomously navigate the area
	- to detect and return the trashes' coordinates 
	- be able to distinguish between waste and recycling.


## Overview


## Getting Started
### map

1. Put “claire_turtlebot3_house.launch” in ‘launch’ folder under turtlebot3_gazebo directory. In my case, it’s in ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch

2. Put “claire_turtlebot3_house.world” in ‘worlds’ folder under turtlebot3_gazebo directory. In my case, it’s in ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds


### robot

1. copy the entire directory to your workspace.

2. In test_sub.py, line 92, change the location to where you want to save the output file.

3. run install.sh to download required packages. If the script doesn't work, please run the commands below manually.
	 
   pip3 install scipy
	
   pip3 install opencv-python
	 
   pip3 install imutils
 
### How to run

1. roslaunch turtlebot3_gazebo claire_turtlebot3_house.launch

2. roslaunch test_proj robotlaunch.launch

3.  (optional) roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
