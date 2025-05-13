#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.color_image_sub = rospy.Subscriber("/sim/camera/D435/colorImage", Image, self.colorImageCallback, queue_size=1)
        self.depth_image_sub = rospy.Subscriber("/sim/camera/D435/depthImage", Image, self.depthImageCallback, queue_size=1)
        self.color_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('/camera/depth/image_rect_raw', Image, queue_size=10)
        rospy.spin()

    def colorImageCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)

        outgray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        grayimg = self.bridge.cv2_to_imgmsg(outgray, "mono8")
        self.color_pub.publish(grayimg)

    def depthImageCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            rospy.loginfo(e)

        outdep = np.array(cv_image, dtype=np.float)
        outdep = outdep * 1000
        outdep = np.round(outdep).astype(np.uint16)
        depthimg = self.bridge.cv2_to_imgmsg(outdep, "16UC1")
        self.depth_pub.publish(depthimg)

if __name__ == '__main__':
    rospy.init_node('sim_image_format_to_slam_node', anonymous=True)  # 初始化ros节点
    rospy.loginfo('node runing...')
    image_converter()  
