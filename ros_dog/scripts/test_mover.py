#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64


def mover():
    pub_LF1 = rospy.Publisher('/dog/LF1_position_controller/command', Float64, queue_size=10)
    pub_LF2 = rospy.Publisher('/dog/LF2_position_controller/command', Float64, queue_size=10)
    pub_LF3 = rospy.Publisher('/dog/LF3_position_controller/command', Float64, queue_size=10)

    pub_LB1 = rospy.Publisher('/dog/LB1_position_controller/command', Float64, queue_size=10)
    pub_LB2 = rospy.Publisher('/dog/LB2_position_controller/command', Float64, queue_size=10)
    pub_LB3 = rospy.Publisher('/dog/LB3_position_controller/command', Float64, queue_size=10)

    pub_RF1 = rospy.Publisher('/dog/RF1_position_controller/command', Float64, queue_size=10)
    pub_RF2 = rospy.Publisher('/dog/RF2_position_controller/command', Float64, queue_size=10)
    pub_RF3 = rospy.Publisher('/dog/RF3_position_controller/command', Float64, queue_size=10)

    pub_RB1 = rospy.Publisher('/dog/RB1_position_controller/command', Float64, queue_size=10)
    pub_RB2 = rospy.Publisher('/dog/RB2_position_controller/command', Float64, queue_size=10)
    pub_RB3 = rospy.Publisher('/dog/RB3_position_controller/command', Float64, queue_size=10)
    rospy.init_node('dog_mover')
    rate = rospy.Rate(10)
    start_time = 0

    while not start_time:
        start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        elapsed = rospy.Time.now().to_sec() - start_time
        pub_LF1.publish(0)
        pub_LF2.publish(-0.4)
        pub_LF3.publish(0.8)
        
        pub_LB1.publish(0)
        pub_LB2.publish(-0.4)
        pub_LB3.publish(0.8)

        pub_RF1.publish(0)
        pub_RF2.publish(-0.4)
        pub_RF3.publish(0.8)
        
        pub_RB1.publish(0)
        pub_RB2.publish(-0.4)
        pub_RB3.publish(0.8)
    rate.sleep()


if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
