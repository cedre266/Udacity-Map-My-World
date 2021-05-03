# Udacity-Map-My-World

## Overview
Fourth project in Udacity's Robotics Software Engineer Nanodegree. The goal of the project is to implement RTAB-Map, a ROS package for visual SLAM using a laser range finder with a RGB-D camera.
This was implemented leveraging the same world and robot created in the previous projects of the same course.

image?

Made under ROS-Kinetic and Gazebo-7.

**Author: Cedric Perney**

## Installation
- Make sure to have a working installation of ROS-Kinetic
- Clone this git repository on your local computer, check/install any missing dependencies and compile the package:
```
cd catkin_ws/src
git clone https://github.com/cedre266/Udacity-Map-My-World.git
cd ../
rosdep install --from-paths . --ignore-src
catkin_make
```

## Usage
- Source at the root
```
cd  catkin_ws/
source devel/setup.bash
```
- Launch the Gazebo simulation with the robot and Rviz
```
roslaunch my_robot world.launch
```
- In a second terminal, launch teleoperation to control the robot with the keyboard
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
- In a third terminal, launch the mapping node:
```
roslaunch my_robot mapping.launch
```
- Now use the teleoperation to move the robot around in the map. When having finished, shut down the mapping node. It will save the database as `~/.ros/rtabmap.db`.
- In order to view the results, we can use the database viewer provided with the RTAB-Map package:
```
rtabmap-databaseViewer ~/.ros/rtabmap.db
```
- Say yes to using the database parameters
    - `View` -> `Constraint View`
    - `View` -> `Graph View`
- You should be able to see RGB images of the camera, that highlights in yellow detected features and in pink detected loop closures candidates. A second view shows an occupancy grid map with the trajectory. A third view, in 3D, allows to see how neighbouring links and loop closures candidates were detected.
- A 3D map of the environment can be generated:
    - `Edit` -> `View 3D map`
    - `Ok`

## Launch files
- **`world.launch`**: launches the Gazebo with the saved world, launches `robot_description.launch` which finds the xacro description file of the robot and sends the robot states to tf, spawns the robot at the specified initial pose and launches Rviz
- **`mapping.launch`**: launches the `rtabmap` node, remapping input parameters to this particular robot and setting other parameters such as those for output, loop closure, feature detection, etc

## Related Documentation
The RTAB-Map package: http://wiki.ros.org/rtabmap_ros

## Illustrations and Comments

