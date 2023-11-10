#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64


def mover():
    pub_j4 = rospy.Publisher('/arm/joint4_position_controller/command', Float64, queue_size=10)
    pub_j5 = rospy.Publisher('/arm/joint5_position_controller/command', Float64, queue_size=10)
    rospy.init_node('arm_mover')
    rate = rospy.Rate(10)
    start_time = 0

    while not start_time:
        start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        elapsed = rospy.Time.now().to_sec() - start_time
        pub_j4.publish(math.sin(2*math.pi*0.1*elapsed)*(math.pi/2))
        pub_j5.publish(-1.57)
        rate.sleep()


if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
