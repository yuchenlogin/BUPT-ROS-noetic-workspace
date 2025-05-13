#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import rospy
import rospkg
import numpy as np
import cv2 
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
from cv_bridge import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from lejulib import finishsend

import vision.imageProcessing as imgPrcs


BALL_GOAL_POS = [240.0, 260.0, 260.0]

# HSV阈值
lowerRed = np.array([0, 192, 96])
upperRed = np.array([20, 255, 240])

class ball_detecter(imgPrcs.ColorObject):
    """继承自颜色识别类"""
    def __init__(self, debug = False):
        super(ball_detecter, self).__init__(lowerRed, upperRed)
        self.__debug = debug
        image_topic = '/chin_camera/image'
        rospy.Subscriber(image_topic, Image, self.__image_callback) # 订阅图像topic,设置图像回调函数

        self.__cv_bridge = CvBridge()
        self.__img_origin = np.zeros((640, 480, 3), np.uint8)

    def __image_callback(self, msg):
        """图像回调函数"""
        try:
            self.__img_origin = self.__cv_bridge.imgmsg_to_cv2(msg, 'bgr8') # 转化为cv2中的bgr格式
        except CvBridgeError as err:
            rospy.logerr(err)

        if self.__debug:# 如果在debug模式则显示debug图片和hsv取色窗口
            hsvImg = cv2.cvtColor(self.__img_origin, cv2.COLOR_BGR2HSV) # 转换颜色空间到HSV
            def getpos(event,x,y,flags,param):
                    if event==cv2.EVENT_LBUTTONDOWN:
                        print("HSV value:{}".format(hsvImg[y,x])) # 打印所选点的hsv值
            imgPrcs.putVisualization(self.__img_origin, self.detection(self.__img_origin))# 绘制识别结果框
            cv2.imshow("ImageResult",self.__img_origin)# 显示识别结果
            cv2.imshow("hsvImg: click to get HSV",hsvImg)# 显示HSV图像,点击该图像中的像素点可以打印出HSV值
            cv2.setMouseCallback('hsvImg: click to get HSV',getpos)# 设置窗口点击事件回调
            cv2.waitKey(1)
            
    def detect(self):
        """
        获取颜色识别结果
        @return result["find"]:是否寻找到匹配的图像区域,True/False
                result["Cx"]:返回最大匹配区域的中心x坐标, 以左上角为原点
                result["Cy"]:返回最大匹配区域的中心y坐标
                result["boundingR"]:匹配区域的最大内接矩形
                result["contour"]:匹配区域的轮廓
        """
        return self.detection(self.__img_origin)
   

if __name__ == '__main__':
    rospy.init_node("detecter_test", anonymous=True)
    rospy.on_shutdown(finishsend)
    ball_detecter(True)
    rospy.spin()
