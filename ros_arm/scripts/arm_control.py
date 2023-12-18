#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Float64, Bool, Float64MultiArray
from ros_arm.srv import GoalPosition, GoalPositionRequest
from inverse_kinetic import *


class ArmControl(object):
    def __init__(self):

        self.node_name = 'arm_control'
        rospy.init_node(self.node_name)

        # Variables
        ## ros
        self.start_time = rospy.Time.now().to_sec()
        self.r = rospy.Rate(10)
        self.count = 0

        ## Constant
        self.joint = [0, math.pi/4, math.pi/2, 0, math.pi/4, 0]
        self.joint_home = [0, 0, 0, 0, 0, 0]
        
        self.carmera_midpoint = [360, 640]
        self.midpoint = Float64MultiArray()
        self.image_range = Float64MultiArray()
        self.at_goal = False
        self.z_at_goal = False
        self.tran_l = None

        # Service
        self.arm_mover = rospy.ServiceProxy('/arm_mover/arm_mover', GoalPosition)
        self.msg = GoalPositionRequest()
        
        # initial position
        self.set_joint(self.joint)
        self.set_gripper([0, 0])

        # Subscribers
        self.sub1 = rospy.Subscriber("/rgb_camera/image_processed/point_exist", Bool, self.main_callback)
        
        # Publishers
        self.pub1 = rospy.Publisher('/arm_command/loop_sign', Bool, queue_size=10)
        self.pub2 = rospy.Publisher('/arm_command/program_run', Bool, queue_size=10)

        self.inverse_kinetic = inverse_kinetic()



    def main_callback(self, exist):
        if self.z_at_goal:
            rospy.loginfo("At goal")
            gripper = self.inverse_kinetic.grasp()
            self.set_gripper(gripper)

            rospy.sleep(1)
            
            self.set_joint(self.joint)


        else:
            if exist.data == True:
                # rospy.sleep(0.5)

                self.midpoint = rospy.wait_for_message("/rgb_camera/image_processed/midpoint", Float64MultiArray)
                self.image_range = rospy.wait_for_message("/rgb_camera/image_processed/image_range", Float64MultiArray)
                self.tran_l, dx, dy = self.inverse_kinetic.get_error(self.midpoint.data)
                joint, x_at_goal, y_at_goal = self.inverse_kinetic.trans_joint_command(dx, dy)
                
                
                if (self.at_goal != True and ((x_at_goal != True) or (y_at_goal != True))):
                    self.set_joint(joint)
                    self.count += 1

                    rospy.loginfo("[Moving Step: %s]", self.count)
                    rospy.loginfo("  -midpoint: {}".format(self.midpoint.data))
                    rospy.loginfo("  -dx: {}".format(dx))
                    rospy.loginfo("  -dy: {}".format(dy))
                
                else:
                    rospy.loginfo("Gasping")
                    rospy.loginfo("  -image_range: {}".format(self.image_range.data))
                    self.pub1.publish(True)
                    self.at_goal = True
                    self.count = 0
                    joint, self.z_at_goal = self.inverse_kinetic.gripper_grasp(self.tran_l, self.image_range.data)
                    self.set_joint(joint)


            else:
                rospy.loginfo("No point detected")

    
    def set_joint(self, joint):
        self.msg.joint1 = joint[0]
        self.msg.joint2 = joint[1]
        self.msg.joint3 = joint[2]
        self.msg.joint4 = joint[3]
        self.msg.joint5 = joint[4]
        self.msg.joint6 = joint[5]

        response = self.arm_mover(self.msg)

    def set_gripper(self, gripper):
        self.msg.finger_joint1 = gripper[0]
        self.msg.finger_joint2 = gripper[1]

        response = self.arm_mover(self.msg)




if __name__ == '__main__':
    try:
        ArmControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
