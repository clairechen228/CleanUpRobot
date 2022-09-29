# CleanUpRobot
- Implemented TurtleBot3 Waffle robot using Robot Operating System (ROS) in a Gazebo simulated environment.
- Tasks: 
	- to semi-autonomously navigate the area
	- to detect and return the trashes' coordinates 
	- be able to distinguish between waste and recycling.


## Overview
<p align="center">
  <img width=80% src="/results/system_workflow.PNG">
</p>

## Methods
### World Map
- There are 6 objects in TurtleBot3 House, one in each room, and some of them are under the tables. 
- Ball and cylinder are representative of recycling and waste.
- Colors are distinct from each other to avoid double-checking.

<p align="center">
   <img width=80% src="/results/map.jpg">
</p>

### Walking Algorithm
- Autonomous SLAM: SLAM mapping of an unexplored environment.

- Semi-Auto Walking: To increase the efficiency in this project, the 'Auto Walk' node applies a bug0 algorithm. A total of 11 target coordinates are
predetermined so that the robot could automatically navigate the house map by checking each target location.

<p align="center">
   <img width=49% src="/results/slam.png">
   <img width=49% src="/results/route.PNG">
</p>

### Object Detection Algorithm
- A basic threshold method was used for object detection.

<p align="center">
   <img width=80% src="/results/detection.PNG">
</p>

## Results


https://user-images.githubusercontent.com/23229612/192689469-6b13b8d2-1f23-4229-a4c4-866df8c0a4be.mp4


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
