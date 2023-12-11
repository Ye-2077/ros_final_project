# ros_final_project  
*This is the personal final project of ROS course in SUSTech 2023 Autumn.*

*ATTENTION PLEASE*
- The materials in this repository are intended for educational purposes, including reference and learning. They should not be used for any form of academic dishonesty or plagiarism.
- Users of this repository are responsible for abiding by academic integrity principles and checking the specific attribution and licensing details associated with any non-original content.
- By accessing and using the materials in this repository, you agree to respect the principles of academic integrity and use the content responsibly. Any misuse or academic dishonesty is the sole responsibility of the user.

---

## Workspace Configue

*System: Ubuntu 20.04*  
*ROS version: ros-noetic*  
*Python: 3.9*

**1. Create your workspace**
  
```
$ cd ~/[your_path]
$ mkdir -p ~/[your_workspace_name]/src
```

**2. Clone this reprosity**
You can put this reprosity in workspace/src path
  
```
$ cd ~/[workspace]/src
$ git clone https://github.com/Ye-2077/ros_final_project.git
```

**3. Download plugin GripperGraspFix**

GripperGraspFix is a gazebo plugin to make grasping process works.  
The wike of this plugin: https://github.com/JenniferBuehler/gazebo-pkgs/wiki/The-Gazebo-grasp-fix-plugin

```
$ cd ~/[workspace]/src
$ git clone https://github.com/JenniferBuehler/general-message-pkgs.git
$ git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
```

**4. Build workspace**  

Advice to use `catkin build` instead of `catkin_make`  

```
$ cd ~/[workspace]  
$ catkin build
```

**5. Chmod the python file**  

The main code written in python, you can directly run these with `python path_to_file/filename.py`.  
If you want to run these files more conviently with `rosrun`, please run fellow command
```
$ chmod 777 path_to_file/filename.py
```

---

## Quik Start

**1. arm grasp**

Launch simulator form `arm_gazebo.launch`  
Start the service from `arm_mover.py`  
Start arm from `arm_control.py`  
Start image process from `image_process.py`  

```
$ cd ~/[workspace]
$ source dev/setup.bash

$ roslaunch ros_arm arm_gazebo.launch
$ rosrun ros_arm arm_mover.py
$ rosrun ros_arm arm_control.py
$ rosrun ros_arm image_process.py
```

---

## How To Use Arm

**1. Joint control & Mover**  

*this part is to test if the joint controller works correctly, and use test_mover.py let the arm moves looply*    

- Terminal_1: `$ roslaunch ros_arm arm_rviz.launch`
  if there is error in camera, after next step then click the "reset" button in the rviz  
- Terminal_2: `$ roslaunch ros_arm arm_gazebo.launch `
- Terminal_3: you can pub control command like this `$ rostopic pub /arm/joint3_position_controller/command std_msgs/Float64 "data: 1.57" `
- Terminal_4: kill Terminal_3, the run test_mover.py: `$ python ros_arm/src/test_mover.py `

  
**2. Camera Test**

*this part is to test the Camera*

- Terminal_1: `$ roslaunch ros_arm arm_gazebo.launch `
- Terminal_2: `$ rqt_image_view /rgb_camera/image_raw `
- Terminal_3: `$ python [path to ros_arm]/src/test_camera_mover.py`

**3. Joint Control Service**

*this part is to test the arm_mover.py and the joint control service*  

- Terminal_1: `$ roslaunch ros_arm arm_gazebo.launch` 
- Terminal_2: `$ rosservice call /arm_mover/arm_mover "{joint1: 0.0, joint2: 0.0, joint3: -1.0, joint4: 1.0, joint5: 0.0, joint6: 0.0, finger_joint1: 0.0, finger_joint2: 0.0}"`

**4. Image Process Test**

*this part is to test image_process.py, used to identify the box with CV*  

- Terminal_1: `$ roslaunch ros_arm arm_gazebo.launch `
- Terminal_2: `$ rosrun ros_arm initial_mover.py # remeber to sudo chmod 777 this file`
- Terminal_3: `$ rosrun ros_arm image_process.py `
- Terminal_4: `$ rqt_image_view /rgb_camera/image_processed`

you can also open another terminal and run `rqt_image_view /rgb_camera/image_raw`  
you can compare these two graphs and find the differences


remember to offer 
 by `sudo chmod 777`
