#!/usr/bin/env python3

import math
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64, Bool, Float64MultiArray
from sensor_msgs.msg import JointState
from scipy.optimize import fsolve
from sensor_msgs.msg import JointState
from ros_arm.srv import GoalPosition, GoalPositionResponse, GoalPositionRequest


class ArmCommand(object):
    def __init__(self):
        # init symbols of completion of the moving loop
        self.dis_y = 0.0
        self.dis_z = 0.0
        self.joint1_g = 0.0
        self.midpoint = 0.0
        self.image_range = 0.0
        self.joint1_data = 0.0
        self.x_at_goal = False
        self.y_at_goal = False
        self.z_at_goal = False
        self.move_end = False
        self.loop_sign = False

        # init ros data
        self.r = rospy.Rate(1)
        rospy.set_param('~loop_sign', False)
        self.start_time = rospy.Time.now().to_sec()
        self.run_sign = False

        # init Service
        self.arm_mover = rospy.ServiceProxy('/arm_mover/arm_mover', GoalPosition)
        self.msg = GoalPositionRequest()

        if not self.run_sign:
            self.arm_pos_set(pos_init=True)

        # init Publisher
        self.pub1 = rospy.Publisher(
            '/arm_command/loop_sign', Bool, queue_size=10)
        self.pub2 = rospy.Publisher(
            '/arm_command/program_run', Bool, queue_size=10)

        # init Subscriber
        rospy.Subscriber('/rgb_camera/image_processed/point_exist', Bool, self.callback)
        rospy.Subscriber('/arm_mover/move_end', Bool, self.move_callback)
        rospy.loginfo('program is run')

    # translate joints' values between calculated value and true value
    def value_translation(self, joint_value, joint_name, joint_type):
        if joint_type == 'cal2gaz':
            if joint_name == 'joint2':
                value = joint_value
            if joint_name == 'joint3':
                value = joint_value
            if joint_name == 'joint4':
                value = joint_value
            if joint_name == 'joint5':
                value = joint_value
            if joint_name == 'joint6':
                value = joint_value

        if joint_type == 'gaz2cal':
            if joint_name == 'joint2':
                value = joint_value
            if joint_name == 'joint3':
                value = joint_value
            if joint_name == 'joint4':
                value = joint_value
            if joint_name == 'joint5':
                value = joint_value
            if joint_name == 'joint6':
                value = joint_value
        return value
    # set all positions of joints with this function

    def arm_pos_set(self,
                    joint1=0.0,
                    joint2=math.pi/4,
                    joint3=math.pi/2,
                    joint4=0.0,
                    joint5=-math.pi/4,
                    joint6=0.0,
                    finger_joint1=0.0,
                    finger_joint2=0.0,
                    pos_init=False):

        self.msg.joint1 = joint1
        self.msg.joint2 = self.value_translation(joint2, 'joint2', 'cal2gaz')
        self.msg.joint3 = self.value_translation(joint3, 'joint3', 'cal2gaz')
        self.msg.joint4 = self.value_translation(joint4, 'joint4', 'cal2gaz')
        self.msg.joint5 = self.value_translation(joint5, 'joint5', 'cal2gaz')
        self.msg.joint6 = self.value_translation(joint6, 'joint5', 'cal2gaz')
        self.msg.finger_joint1 = finger_joint1
        self.msg.finger_joint2 = finger_joint2

        # rospy.loginfo(self.msg.joint1)
        response = self.arm_mover(self.msg.joint1,
                                  self.msg.joint2,
                                  self.msg.joint3,
                                  self.msg.joint4,
                                  self.msg.joint5,
                                  self.msg.joint6,
                                  self.msg.finger_joint1,
                                  self.msg.finger_joint2)

        if pos_init:
            rospy.loginfo('init all joints position')





    # The following parts are use for feedback loop
    # at goal detection: return true, if it's less than an acceptable error.
    def at_goal(self, pos_joint, goal_joint, error_parameter=40):
        result = abs(pos_joint - goal_joint) <= abs(error_parameter)
        return result

    # determine the direction of the error
    def sign_output(self, at_goal, delta):
        if not at_goal:
            if delta > 0:
                sign = 1
            else:
                sign = -1
        else:
            sign = 0
        return sign

    # get the error direction
    def img_data_to_error(self, midpoint, image_range, z_range_parameter=340):
        # get error in x,y direction
        camera_midpoint = [640, 360]
        delta_x = camera_midpoint[0] - midpoint.data[0]
        delta_y = camera_midpoint[1] - midpoint.data[1]
        rospy.loginfo('midpoint: %s', midpoint.data)

        # get error in z direction
        z_x = image_range.data[2] - image_range.data[0]
        z_y = image_range.data[3] - image_range.data[1]
        object_range = math.sqrt(z_x * z_x + z_y * z_y)
        delta_z = z_range_parameter - object_range

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

    def joint_angle_limit(self, joint_value, joint_name):
        joint_limit = False
        if joint_name == 'joint2': limit = [-3.14, 3.14]
        elif joint_name == 'joint3': limit = [-3.14, 3.14]
        elif joint_name == 'joint4': limit = [-3.14, 3.14]
        elif joint_name == 'joint5': limit = [-3.14, 3.14]
        elif joint_name == 'joint6': limit = [-3.14, 3.14]

        if not limit[0] <= joint_value <= limit[1]:
            joint_value = min(max(limit[0], joint_value), limit[1])
            joint_limit = True

        return joint_value, joint_limit

    def get_joint_value(self):
        joint_state = rospy.wait_for_message('/arm/joint_states', JointState)

        # get joints' real value
        joint1 = joint_state.position[0]
        joint2 = joint_state.position[1]
        joint3 = joint_state.position[2]
        joint4 = joint_state.position[3]
        joint5 = joint_state.position[4]
        joint6 = joint_state.position[5]

        # translate joints' value
        joint2 = self.value_translation(joint2, 'joint2', 'gaz2cal')
        joint3 = self.value_translation(joint3, 'joint3', 'gaz2cal')
        joint4 = self.value_translation(joint4, 'joint4', 'gaz2cal')
        joint5 = self.value_translation(joint5, 'joint5', 'gaz2cal')
        joint6 = self.value_translation(joint6, 'joint6', 'gaz2cal')

        return joint1, joint2, joint3, joint4, joint5, joint6

    ## TODO ##
    def arm_controller(self, x_direction_sign, y_direction_sign, z_direction_sign, effect=0.01):

        joint1, joint2, joint3, joint4, joint5, joint6 = self.get_joint_value()
        joint1 = self.joint1_data
        rospy.loginfo('\nGet: \n joint1: %s,\n joint2: %s,\n joint3: %s,\n joint4: %s,\n joint5: %s,\n joint6: %s',
                      joint1, joint2, joint3, joint4, joint5, joint6)

        # calculate the next joint value
        joint1_cal = joint1 + effect * x_direction_sign
        joint2_cal = joint2 + effect * y_direction_sign
        joint3_cal = joint3 + effect * 5 * z_direction_sign
        joint4_cal = abs(joint3) - abs(joint2) - 0.15
        joint5_cal = joint5
        joint6_cal = joint6
        rospy.loginfo('\nCal: \n joint1: %s,\n joint2: %s,\n joint3: %s,\n joint4: %s,\n joint5: %s,\n joint6: %s',
                      joint1_cal, joint2_cal, joint3_cal, joint4_cal, joint5_cal, joint6_cal)


        # limit joints' value
        joint2_cal, joint2_limit = self.joint_angle_limit(joint2_cal, 'joint2')
        joint3_cal, joint3_limit = self.joint_angle_limit(joint3_cal, 'joint3')
        joint4_cal, joint4_limit = self.joint_angle_limit(joint4_cal, 'joint4')
        joint5_cal, joint5_limit = self.joint_angle_limit(joint5_cal, 'joint3')
        joint6_cal, joint6_limit = self.joint_angle_limit(joint6_cal, 'joint4')

        # rospy.loginfo('limit:joint1:%s, joint2:%s, joint3:%s, joint4:%s', joint1, joint2, joint3, joint4)
        limit = joint2_limit and joint3_limit and joint4_limit and joint5_limit and joint6_limit
        if limit:
            rospy.logwarn('target is out of the range!')

        # set joints' value, at begin just set values of joint1 and joint2
        if not (self.x_at_goal and self.y_at_goal):
            self.arm_pos_set(joint1=joint1_cal, joint2=joint2_cal, joint3=joint3,
                             joint4=joint4_cal, joint5=joint5_cal, joint6=joint6_cal)
        else:
            self.arm_pos_set(joint1=joint1_cal, joint2=joint2_cal, joint3=joint3_cal,
                             joint4=joint4_cal, joint5=joint5_cal, joint6=joint6_cal)
        print('moved')    

        # while not self.move_end:
        #     if self.move_end:
        #         break
        
        return True

    def command_callback(self, exist):
        rospy.loginfo('command_callback')
        self.run_sign = True
        if exist:
            try:
                rospy.loginfo('point is exist!')
                poex = True
                while not (self.x_at_goal and self.y_at_goal and self.z_at_goal):
                    # get img data
            
                    if not poex:
                        break

                    rospy.set_param('~loop_sign', False)
                    rospy.loginfo('start moving...')
                    self.midpoint = rospy.wait_for_message('/rgb_camera/image_processed/midpoint', Float64MultiArray)
                    self.image_range = rospy.wait_for_message('/rgb_camera/image_processed/image_range', Float64MultiArray)
                    joint_states = rospy.wait_for_message('/arm/joint_states', JointState)
                    self.joint1_data = joint_states.position[2]
                    # self.joint1_data = rospy.wait_for_message('/rgb_camera/image_processed/joint1_position', Float64)
                    

                    # deal joints information
                    x_sign, y_sign, z_sign = self.img_data_to_error(self.midpoint, self.image_range)
                    rospy.loginfo('x_sign: %s, y_sign: %s, z_sign: %s',x_sign, y_sign, z_sign)
                    
                    #TODO#
                    self.loop_sign = self.arm_controller(x_sign, y_sign, z_sign, effect=0.01)
                    

                    # self.pub1.publish(self.loop_sign)
                    rospy.set_param('~loop_sign', True)
                    self.r.sleep()

                self.r.sleep()

                if poex:
                    print('start to move gripper')
                    # gripper start run
                    joint1, joint2, joint3, joint4, joint5, joint6 = self.get_joint_value()

                    self.arm_pos_set(joint1=joint1, joint2=joint2, joint3=joint3, joint4=joint4, joint5=joint5, joint6=joint6,
                                     finger_joint1=0.2, finger_joint2=0.2)

                    self.r.sleep()
                    print('return to initial position')
                    ## TODO##
                    # change to fit the 6 joint##
                    # return to initial position
                    self.arm_pos_set(left_joint=0.15, right_joint=0.15)
                    self.r.sleep()

                    # arm turn to predetermined position
                    self.arm_pos_set(joint1=1.0, left_joint=0.15, right_joint=0.15)
                    self.r.sleep()

                    self.arm_pos_set(joint1=1.0, joint2=0.2, joint3=-1.0, joint4=0.8,
                                     left_joint=0.15, right_joint=0.15)
                    self.r.sleep()

                    # gripper opening
                    self.arm_pos_set(joint1=1.0, joint2=0.2, joint3=-1.0, joint4=0.8,
                                     left_joint=0.15, right_joint=0.15)
                    self.r.sleep()

                    self.arm_pos_set(pos_init=True)
                    self.r.sleep()
                    rospy.loginfo('program has finish')
                rospy.on_shutdown(self.my_shutdown_callback)

            except rospy.ServiceException:
                rospy.logwarn("Midpoint is exist, but service call failed")

        else:
            try:
                rospy.loginfo('try to find target')
                elapsed = rospy.Time.now().to_sec() - self.start_time
                joint1 = math.sin(2 * math.pi * 0.05 * elapsed) * (math.pi / 2)
                self.arm_pos_set(joint1=joint1)
                # self.joint1_g = math.sin(2 * math.pi * 0.1 * elapsed) * (math.pi / 2)

            except rospy.ServiceException:
                rospy.logwarn("Midpoint is not exist and Service call failed")

    def move_callback(self, move_end):
        self.move_end = move_end.data

    def my_shutdown_callback():
        rospy.loginfo("Node is shutting down...")

    def callback(self, exist):
        try:
            service_available = rospy.wait_for_service('arm_mover/arm_mover', timeout=1)
            print(f"Service '{'arm_mover/arm_mover'}' is available!")
            self.command_callback(exist.data)

        except rospy.exceptions.ROSException:
            print('Service not available within the specified timeout.')


if __name__ == '__main__':
    try:
        node_name = 'arm_command'
        rospy.init_node(node_name)
        ArmCommand()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
