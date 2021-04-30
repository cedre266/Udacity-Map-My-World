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
- **`amcl.launch`**: launches the `map_server` node for getting the map, launches the `amcl` node for the Monte-Carlo localization with the specified parameters and launches the `move_base` node to allow navigation commands through Rviz

## Related Documentation
The amcl package: http://wiki.ros.org/amcl

Create the map: https://github.com/udacity/pgm_map_creator

## Illustrations and Comments
Here is an example of the robot localizing itself. Another example was shown in the overview section. In both images, it can be seen that the particles are well located around the robot's real pose, and that the laser scans are well aligned with the map.
![Example image](screenshots/2.png)

It appears that the localization performs better when the robot is moving straight compared to when it is rotating. The image below has been taken after a simple 180° rotation of the robot on itself. The particles are much more spread out around the robot, showing a bigger uncertainty on the real pose.
![Example image](screenshots/higher_uncertainty_after_rotation.png)

After commanding the robot to go straight for a small distance, the particles form again a very narrow cloud around the robot's location.
![Example image](screenshots/lower_afte_goin_straight.png)

### Simulating the "kidnapped robot" problem
The kidnapped robot problem is the most difficult problem to solve in localization. It consists of moving the robot to a different pose without giving the robot any clue, and see whether it is capable of recovering its real pose again. In order to try this here, it is possible to command the robot to a certain pose, then deactivate the amcl node and reactivate it so that the robot believes it is located at its initial pose. Then, the robot can be moved to see whether the particles manage to find the robot's real pose again or not.

Here is what happens. At first, the robot is moved to some location and it could successfully localize itself. (Note that the laser scans do not align on the little gray vehicle in Rviz. This is due to that vehicle having moved when the map was created. The laser scans show the real location of that vehicle, which is aligned with the wall on the right. It shows how this localization approach is limited to mostly static environments.)
![Example image](screenshots/3.png)

Then, the amcl node has been deactivated and reactivated. The robot believes it is located at its initial pose. The laser scans do not match at all.
![Example image](screenshots/kidnapped_robot_problem.png)

After commanding the robot to move for a while, the particles separate themselves into multiple clouds as each particle tries to fit the laser scans to the map to find its location.
![Example image](screenshots/kidnapped_robot_problem_2.png)

Finally, the particles converged to a solution. However, the found solution is a wrong location. The laser scans only partially alignn with the map, but that was enough to make the particles converge there. This is certainly because the map consists of multiple long walls, so they are easy to confuse with each other.
![Example image](screenshots/kidnapped_robot_problem_3.png)

It shows then that the robot is in this case not able to recover. In order to let him recover, there could be two possibilities. The first one is to manually give the robot an estimate of its real pose through the `2D Pose Estimate` button in Rviz. As the particles, in this configuration, spread only around the initial pose or manually given estimate, some particles will cover the real pose and allow the robot to recover. The second solution could be changing the particles configuration so that they are initially spread over the entire map. This solution, however, would be obviously limited by the number of particles the computation can afford and the bigger the map, the more it would struggle.
