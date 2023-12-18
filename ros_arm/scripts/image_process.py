#!/usr/bin/env python

import math
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64, Bool, Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError

class ImgProcess(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.node_name = 'img_process'
        self.midpoint = Float64MultiArray()
        self.image_range = Float64MultiArray()

        rospy.init_node(self.node_name)

        self.pub1 = rospy.Publisher('/rgb_camera/image_processed', Image, queue_size=10)
        self.pub2 = rospy.Publisher('/rgb_camera/image_processed/point_exist', Bool, queue_size=10)
        self.pub3 = rospy.Publisher('/rgb_camera/image_processed/midpoint', Float64MultiArray, queue_size=10)
        self.pub4 = rospy.Publisher('/rgb_camera/image_processed/image_range', Float64MultiArray, queue_size=10)

        self.sub1 = rospy.Subscriber("rgb_camera/image_raw", Image, self.img_process_callback, queue_size=10)
        
        rospy.loginfo("Waiting for image topics...")
        rospy.spin()
    
    def image_process(self, image):
        image_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        
        orange_min = np.array([100, 128, 46])
        orange_max = np.array([124, 255, 255])
        mask = cv2.inRange(image_hsv, orange_min, orange_max)
        
        non_zero_pixels = np.transpose(np.nonzero(mask))
        num_pixels = len(non_zero_pixels)
        
        point_exist = num_pixels > 0
        midpoint = min_point = max_point = [0, 0] if not point_exist else np.mean(non_zero_pixels, axis=0).astype(int)
        
        if point_exist:
            min_point = np.min(non_zero_pixels, axis=0)
            max_point = np.max(non_zero_pixels, axis=0)

        res = cv2.bitwise_and(image, image, mask=mask)
        cv2.circle(res, (midpoint[1], midpoint[0]), 5, (0, 0, 255), 2)
        cv2.rectangle(res, (min_point[1], min_point[0]), (max_point[1], max_point[0]), (0, 0, 255))

        return res, midpoint, min_point, max_point, point_exist


    def img_process_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError:
            rospy.loginfo('CvBrigeError!')

        frame = np.array(frame,dtype=np.uint8)
        processed_image, midpoint, min_point, max_point, point_exist = self.image_process(frame)
        self.midpoint.data = midpoint
        self.image_range.data = max_point-min_point

        if point_exist:
            rospy.loginfo('midpoint: %s, image_range: %s, %s', midpoint, min_point, max_point)
            processed_Image = self.bridge.cv2_to_imgmsg(processed_image,"bgr8")
            self.pub1.publish(processed_Image)
            self.pub2.publish(point_exist)
            self.pub3.publish(self.midpoint)
            self.pub4.publish(self.image_range)
        else:
            rospy.loginfo('No targets were detected')
            self.pub2.publish(point_exist)


if __name__ == '__main__':
        try:
            ImgProcess()
        except rospy.ROSInterruptException:
            pass