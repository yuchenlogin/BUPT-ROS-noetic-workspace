#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import numpy as np
import rospy

import cv2
from cv_bridge import *
from sensor_msgs.msg import *

IMAGE_TOPIC = '/camera/color/image_raw'
CONTROL_ID = 2
VERTICAL_DIFFERENCE = 50

DEFAULT_COLOR_DETECT = {"yellow": "yellow", "blue": "blue", "red": "red"}

YELLOW_LOW = np.array([1, 160, 140])
YELLOW_HIGH = np.array([80, 215, 260])

BLUE_LOW = np.array([70,120, 100])
BLUE_HIGH = np.array([110, 250, 190])

RED_LOW = np.array([0, 43, 46])
RED_HIGH = np.array([10, 255, 255])

DEFAULT_COLOR_HSV_RANGE = {"yellow": [YELLOW_LOW, YELLOW_HIGH], "blue": [BLUE_LOW, BLUE_HIGH], "red":[RED_LOW, RED_HIGH]}

Y_AXIS_INDEX = 1

class Box_color_detecter():
    def __init__(self, target_box_color):
        self.sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.image_callback)
        self.cv_bridge = CvBridge()
        self.frame = None
        self.target_color = target_box_color

    def image_callback(self,msg):
        try:
            self.frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as err:
            rospy.logerr(err)

    def process_ori_to_hsv(self):
        return cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

    def detec_color_center(self, hsv, color):
        mask = None
        if color in DEFAULT_COLOR_DETECT:
            mask = cv2.inRange(hsv, DEFAULT_COLOR_HSV_RANGE[color][0], DEFAULT_COLOR_HSV_RANGE[color][1])
        else:
            exit("color is wrong\n\n\n")
        erosion = cv2.erode(mask, None, iterations=2)
        dilation = cv2.dilate(erosion, np.ones((1, 1), np.uint8), iterations=2)
        ret, binary = cv2.threshold(dilation, 127, 255, cv2.THRESH_BINARY)
        __c, contours,__a = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0: return None

        def cnt_area(cnt):
            area = cv2.contourArea(cnt)
            return area

        contours.sort(key=cnt_area, reverse=True)
        x, y, w, h = cv2.boundingRect(contours[0])
        contours_center = (x + w) // 2, (y + h) // 2
        return contours_center

    def detect_color_position(self, hsv_img):
        try:
            yellow_contours_center = self.detec_color_center(hsv_img, DEFAULT_COLOR_DETECT["yellow"])
            blue_contours_center = self.detec_color_center(hsv_img, DEFAULT_COLOR_DETECT["blue"])
            if yellow_contours_center != None and blue_contours_center != None and abs(yellow_contours_center[Y_AXIS_INDEX] - blue_contours_center[Y_AXIS_INDEX]) < VERTICAL_DIFFERENCE:
                return {DEFAULT_COLOR_DETECT["yellow"]: yellow_contours_center, DEFAULT_COLOR_DETECT["blue"]: blue_contours_center}
        except:
            print(sys.exc_info(), 104)

    def detect_color_box(self, color_position_result):
        if self.target_color in color_position_result:
            target_color_contours_center = color_position_result[self.target_color]
            for color in color_position_result.keys():
                if color !=self.target_color :
                    return "left" if color_position_result[color][0] > target_color_contours_center[0] else "right"
                
        rospy.logerr("UNKNOW BOX COLOR!")
        return

    def stop_detecter(self):
        self.sub.unregister()
        rospy.loginfo("color detecter exit!")

    def main(self):
        time.sleep(0.1)
        hsv_img = self.process_ori_to_hsv()
        color_position_result = self.detect_color_position(hsv_img)
        position = self.detect_color_box(color_position_result)
        print(position)
        self.stop_detecter()
        return position

if __name__ == '__main__':
    
    box_detect = Box_color_detecter()
    box_direction = box_detect.main("blue")
