#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TeleopJoy:
    def __init__(self):
        # 实例化ROS句柄
        rospy.init_node('keyboard_control')
        
        # 从参数服务器读取参数，如果没有设置则使用默认值
        self.axis_linear = rospy.get_param("~axis_linear", 1)
        self.axis_angular = rospy.get_param("~axis_angular", 2)
        # creating Twist message object
        vel = Twist()
        vel.linear.x = 0
        vel.angular.z = 0
        # 定义发布者对象，用来将手柄数据发布到机器人控制话题上
        self.pub = rospy.Publisher('cmd_vel',Twist, queue_size=10)
        # 定义订阅者对象，用来订阅手柄发送的数据
        self.sub = rospy.Subscriber('joy', Joy, self.callback)

    # 处理手柄发送过来的信息
    def callback(self, joy):
        vel = Twist()
        # 将手柄摇杆轴拨动时值的输出赋值线速度和角速度
        vel.linear.x = joy.axes[self.axis_linear]
        vel.angular.z = joy.axes[self.axis_angular]
        rospy.loginfo("当前线速度为: %.3lf； 角速度为: %.3lf", vel.linear.x, vel.angular.z)
        self.pub.publish(vel)

if __name__ == '__main__':
    try:
        # 初始化TeleopTurtle类
        teleop_joy = TeleopJoy()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

