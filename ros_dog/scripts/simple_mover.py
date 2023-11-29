import rospy
from std_msgs.msg import Float64
import numpy as np


def move_end_effector(path_points, pub1, pub2):
    rate = rospy.Rate(10)  # 发布频率为10Hz
    for point in path_points:
        joint1_angle = point[0]
        joint2_angle = point[1]

        pub1.publish(joint1_angle)
        pub2.publish(joint2_angle)

        rate.sleep()


def generate_path(radius, resolution):
    path_points = []
    for theta in np.linspace(0, np.pi, resolution):
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        # 这里假设你已经有了逆运动学模型来得到关节角度
        joint1_angle = np.arctan2(y, x)  # 通过逆运动学计算得到关节1角度
        joint2_angle = np.pi - joint1_angle  # 关节2角度，保持和关节1相反

        path_points.append((joint1_angle, joint2_angle))

    return path_points


def circular_motion():
    rospy.init_node('end_effector_controller', anonymous=True)
    joint1_pub = rospy.Publisher(
        '/joint1_controller/command', Float64, queue_size=10)
    joint2_pub = rospy.Publisher(
        '/joint2_controller/command', Float64, queue_size=10)

    radius = 1.0  # 半圆半径
    resolution = 100  # 路径分辨率

    path_points = generate_path(radius, resolution)
    move_end_effector(path_points, joint1_pub, joint2_pub)


if __name__ == '__main__':
    try:
        circular_motion()
    except rospy.ROSInterruptException:
        pass
