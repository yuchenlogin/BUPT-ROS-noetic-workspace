#!/usr/bin/env python3
# coding=utf-8

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String
from ros_fall_detection.srv import FallDetectionService, FallDetectionServiceResponse
from fall import FallDetection
import numpy as np

IMAGE_TOPIC = '/camera/color/image_raw'
FALL_CONFIDENCE_THRESHOLD = 0.8
FALL_RECOGNIZER_PUBLISHER = '/fall_detection_node/fall_result'
FALL_RECOGNIZER_SET_CONFIDENCE_SERVER = '/fall_detection_node/fall_result'

class FallDetectionNode:
    def __init__(self):
        rospy.init_node('fall_detection_node', anonymous=True)
        self.cv_bridge = CvBridge()
        self.fall_classifier = FallDetection()
        self.fall_classifier.load_model()
        self.confidence = rospy.get_param('~confidence', FALL_CONFIDENCE_THRESHOLD)
        self.fall_pub = rospy.Publisher(FALL_RECOGNIZER_PUBLISHER, String, queue_size=1)
        # 创建设置置信度服务
        self.set_confidence_service = rospy.Service(
            FALL_RECOGNIZER_SET_CONFIDENCE_SERVER,
            FallDetectionService,
            self.set_confidence_handler  # 添加服务处理函数
        )
        self.last_image_time = rospy.Time.now()  # 记录上一次图像处理的时间

    def set_confidence_handler(self, request):
        # 处理设置置信度的服务请求
        self.confidence = request.confidence
        return FallDetectionServiceResponse()

    
    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.fall_classifier.load_image(msg)

            # 如果图像是灰度图像，则转换为彩色图像
            if cv_image.ndim == 2:
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            
            # 将OpenCV图像转换为ROS图像消息
            cv_image_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            
            # 使用模型进行摔倒检测
            response, confidence = self.fall_classifier.detect_fall(cv_image_msg, self.confidence)
            
            # 判断 response 是否为 "fall"
            if response == "fall":
                self.fall_pub.publish("fall")
                print("跌倒事件，置信度：", confidence)
            else:
                print("正常，置信度：", confidence)
            
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)  # 等待按键输入（单位：毫秒）
            
        except Exception as e:
            pass
            
    def run(self):
        while not rospy.is_shutdown():
            try:
                # 等待接收图像消息
                msg = rospy.wait_for_message(IMAGE_TOPIC, Image, timeout=1.0)
                # 处理图像消息
                self.image_callback(msg)
            except rospy.exceptions.ROSException:
                continue

if __name__ == '__main__':
    try:
        node = FallDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass