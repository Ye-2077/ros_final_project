#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64


def mover():
    pub_j1 = rospy.Publisher('/dog/LF1_position_controller/command', Float64, queue_size=10)
    pub_j2 = rospy.Publisher('/dog/LF2_position_controller/command', Float64, queue_size=10)
    pub_j3 = rospy.Publisher('/dog/LB1_position_controller/command', Float64, queue_size=10)
    pub_j4 = rospy.Publisher('/dog/LB2_position_controller/command', Float64, queue_size=10)
    pub_j5 = rospy.Publisher('/dog/RF1_position_controller/command', Float64, queue_size=10)
    pub_j6 = rospy.Publisher('/dog/RF2_position_controller/command', Float64, queue_size=10)
    pub_j7 = rospy.Publisher('/dog/RB1_position_controller/command', Float64, queue_size=10)
    pub_j8 = rospy.Publisher('/dog/RB2_position_controller/command', Float64, queue_size=10)
    rospy.init_node('dog_mover')
    rate = rospy.Rate(10)
    start_time = 0

    while not start_time:
        start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        elapsed = rospy.Time.now().to_sec() - start_time
        pub_j1.publish(0)
        pub_j2.publish(math.sin(2*math.pi*0.1*elapsed)*(math.pi/2))
        pub_j3.publish(0)
        pub_j4.publish(0)
        pub_j5.publish(0)
        pub_j6.publish(0)
        pub_j7.publish(0)
        pub_j8.publish(0)
    rate.sleep()


if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
