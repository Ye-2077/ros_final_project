#!/usr/bin/env python3

import math
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64, Bool, Float64MultiArray
from sensor_msgs.msg import JointState
from scipy.optimize import fsolve
from sensor_msgs.msg import JointState
from ros_arm.srv import GoalPosition, GoalPositionRequest


class ArmControl(object):
    def __init__(self):

        self.node_name = 'arm_control'
        rospy.init_node(self.node_name)

        # Variables
        ## ros
        self.start_time = rospy.Time.now().to_sec()
        self.r = rospy.Rate(5)
        ## target
        self.x_at_goal = False
        self.y_at_goal = False
        self.z_at_goal = False
        self.midpoint = Float64MultiArray()
        self.image_range = Float64MultiArray()

        # Service
        self.arm_mover = rospy.ServiceProxy('/arm_mover/arm_mover', GoalPosition)
        self.msg = GoalPositionRequest()

        # Subscribers
        self.sub1 = rospy.Subscriber("/rgb_camera/image_processed/point_exist", Bool, self.main_callback)
        
        # Publishers
        self.pub1 = rospy.Publisher('/arm_command/loop_sign', Bool, queue_size=10)
        self.pub2 = rospy.Publisher('/arm_command/program_run', Bool, queue_size=10)



    def main_callback(self, exist):
<<<<<<< Updated upstream
        try:
            service_available = rospy.wait_for_service('/arm_mover/arm_mover', timeout=1)
            print(f"Service '{'/arm_mover/arm_mover'}' is available!")
            self.command_callback(exist.data)
=======
        if self.z_at_goal:
            rospy.loginfo("At goal")
            gripper = self.inverse_kinetic.grasp()
            self.set_gripper(gripper)
            print(self.msg.finger_joint1)
            print(self.msg.finger_joint2)
            rospy.sleep(1)
            self.set_joint(self.joint)
>>>>>>> Stashed changes

        except rospy.exceptions.ROSException:
            print('Service not available within the specified timeout.')

    

    def command_callback(self, exist):
        if exist:
            try:
                rospy.loginfo("point exist")
                
                while not(self.x_at_goal and self.y_at_goal and self.z_at_goal):
                    
                    joint_states = rospy.wait_for_message('/arm/joint_states', JointState)
                    self.midpoint = rospy.wait_for_message('/rgb_camera/image_processed/midpoint', Float64MultiArray)
                    self.image_range = rospy.wait_for_message('/rgb_camera/image_processed/image_range', Float64MultiArray)
                    
                    
                    self.r.sleep()

            except rospy.ServiceException as e:
                rospy.logwarn(f"Service call failed: {e}")
                rospy.logwarn("Midpoint is exist, but service call failed")


    
    def arm_pos_set(self, joint1=1.0, joint2=math.pi/4, joint3=math.pi/2,
                          joint4=0.0, joint5=math.pi/4, joint6=0.0,
                          finger_joint1=0.0, finger_joint2=0.0,
                          pos_init=False):

        print(joint1)
        self.msg.joint1 = joint1
        self.msg.joint2 = joint2
        self.msg.joint3 = joint3
        self.msg.joint4 = joint4
        self.msg.joint5 = joint5
        self.msg.joint6 = joint6
        self.msg.finger_joint1 = finger_joint1
        self.msg.finger_joint2 = finger_joint2

        self.arm_mover(self.msg.joint1, self.msg.joint2, self.msg.joint3,
                       self.msg.joint4, self.msg.joint5, self.msg.joint6,
                       self.msg.finger_joint1, self.msg.finger_joint2)

        if pos_init:
            rospy.loginfo('init all joints position')
            pos_init = True

    def img_data_to_error(self, midpoint, image_range, z_range=340):
        # get error in x,y direction
        camera_midpoint = [640, 360]
        delta_x = camera_midpoint[0] - midpoint.data[0]
        delta_y = camera_midpoint[1] - midpoint.data[1]
        rospy.loginfo('midpoint: %s', midpoint.data)

        # get error in z direction
        z_x = image_range.data[2] - image_range.data[0]
        z_y = image_range.data[3] - image_range.data[1]
        object_range = math.sqrt(z_x * z_x + z_y * z_y)
        delta_z = z_range - object_range

        if not self.z_at_goal:
            self.x_at_goal = self.at_goal(midpoint.data[0], camera_midpoint[0])
            self.y_at_goal = self.at_goal(midpoint.data[1], camera_midpoint[1])
            self.z_at_goal = self.at_goal(object_range, z_range_parameter, error_parameter=30)

        if self.z_at_goal:
            self.x_at_goal = self.at_goal(midpoint.data[0], camera_midpoint[0], error_parameter=50)
            self.y_at_goal = self.at_goal(midpoint.data[1], camera_midpoint[1], error_parameter=50)
            self.z_at_goal = self.at_goal(object_range, z_range_parameter, error_parameter=30)

        rospy.loginfo('x_at_goal: %s, y_at_goal: %s, z_at_goal: %s', self.x_at_goal, self.y_at_goal, self.z_at_goal)

        # get the sign in all direction
        x_sign = self.sign_output(self.x_at_goal, delta_x)
        y_sign = self.sign_output(self.y_at_goal, delta_y)
        z_sign = self.sign_output(self.z_at_goal, delta_z)
        return -x_sign, y_sign, z_sign