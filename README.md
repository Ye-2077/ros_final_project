# ros_final_project  
*This is the personal final project of ROS course in SUSTech 2024 Spring.*

*ATTENTION PLEASE*
- The materials in this repository are intended for educational purposes, including reference and learning. They should not be used for any form of academic dishonesty or plagiarism.
- Users of this repository are responsible for abiding by academic integrity principles and checking the specific attribution and licensing details associated with any non-original content.
- By accessing and using the materials in this repository, you agree to respect the principles of academic integrity and use the content responsibly. Any misuse or academic dishonesty is the sole responsibility of the user.

---

## Moveit

**1. Install Moveit**

System: Ubuntu 20.04  
ROS version: ros-noetic  
Refer to https://moveit.ros.org/install/source/

This project includes the Moveit! in the `Moveit` package, you can just run below code to build the Moveit!
```
$ catkin buld Moveit # run this in your workspace path
```

Or you can ignore this and download by yourself (then please deleat the Moveit package and follow below steps)  
You can refer to https://moveit.ros.org/install/source/

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

---

## Prepare the Camera

  install camera-driver
```
$ sudo apt-get install ros-noetic-usb-cam
```
  test if it install correctly
```
$ roslaunch usb_cam usb_cam-test.launch
```

## How To Use

**1. Joint control & Mover**  

*this part is to test if the joint controller works correctly, and use test_mover.py let the arm moves looply*    

- Terminal_1 
```
$ roslaunch ros_arm arm_rviz.launch
```
  if there is error in camera, after next step then click the "reset" button in the rviz  
- Terminal_2
```
$ roslaunch ros_arm arm_gazebo.launch 
```
- Terminal_3  
  you can pub control command like this
```
$ rostopic pub /arm/joint3_position_controller/command std_msgs/Float64 "data: 1.57" 
```
- Terminal_4
  kill Terminal_3, the run test_mover.py
```
$ python ros_arm/src/test_mover.py 
```

  
**2. Camera Test** 

*this part is to test the Camera*

- Terminal_1 
```
$ roslaunch ros_arm arm_gazebo.launch 
```
- Terminal_2  
```
$ rqt_image_view /rgb_camera/image_raw 
```
- Terminal_3
```
$ python [path to ros_arm]/src/test_camera_mover.py
```

