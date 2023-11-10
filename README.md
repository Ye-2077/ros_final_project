# ros_final_project  
This is the personal final project of ROS course in SUSTech 2024 Spring.

ATTENTION PLEASE  
- The materials in this repository are intended for educational purposes, including reference and learning. They should not be used for any form of academic dishonesty or plagiarism.
- Users of this repository are responsible for abiding by academic integrity principles and checking the specific attribution and licensing details associated with any non-original content
- By accessing and using the materials in this repository, you agree to respect the principles of academic integrity and use the content responsibly. Any misuse or academic dishonesty is the sole responsibility of the user.


## Moveit
---
**1. Install Moveit**

System: Ubuntu 20.04  
ROS version: ros-noetic  
Refer to https://moveit.ros.org/install/source/

This project includes the Moveit! in the `Moveit` package, you can just run below code to install the Moveit!
```
$ catkin buld Moveit # run this in your workspace path
```

Or you can ignore this and download by yourself(then please deleat the Moveit package and follow below steps)

- create new workspace or just in your own ws 
```
$ mkdir ~/ws_moveit
$ cd ~/ws_moveit
```
- load the `${ROS_DISTRO}` variable, needed for the next step
```
$ source /opt/ros/noetic/setup.bash 
```
- download source code, in your workspace's root path
```
$ wstool init src
$ wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
$ wstool update -t src
$ rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
$ catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
```
- build
```
$ catkin build
$ source ~/{your_workspace}/devel/setup.bash # or .zsh, depending on your shell
```
**2. Check if Moveit works**  
Refer to this: [MoveIt Quickstart in RViz](https://ros-planning.github.io/moveit_tutorials/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html)
