#! /usr/bin/python3
# coding=utf-8

import time
import rospy
from face_detecter import Face_detecter
from digital_detecter import DigitalDetecter
from detect_ball_in_area import  DetectBallInArea

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image
from ros_vision_node.srv import FaceDetectService, FaceVerifyService, DigitalDetectSrv, HeadCamDigitalDetectSrv, BallDetectInAreaSrv
import json

HEAD_CAM_TOPIC = "/camera/color/image_raw"

class vision_node():
    def __init__(self):
        self.face_detector = Face_detecter(display=False)
        self.cv_bridge = CvBridge()
        self.last_active_time = 0
        self.digital_detector = None
        self.ball_detect_in_area = None

    def msg_to_cv2(self,msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as err:
            print(err)
        else:
            return cv_image

    def handle_face_detect(self,req):
        try:
            if req.path == "":
                msg = rospy.wait_for_message("/camera/color/image_raw", Image, 1)
                current_frame = self.msg_to_cv2(msg)
            else:
                current_frame =  cv2.imread(req.path)
            result = self.face_detector.detect_a_frame(current_frame)
        except Exception as err:
            rospy.logerr(err)
            return "Error"
        else:
            self.last_active_time = time.time()
            return result

    def handle_face_verify(self,req):
        encodings1=np.array(req.faceId1)
        encodings2=np.array(req.faceId2)
        if len(encodings1) and len(encodings2):
            distance = self.face_detector.get_distance(encodings1,encodings2)
            return int((1-distance)*100),1-distance
        else:
            rospy.logerr("未检出人脸")
            return None

    def handle_digital_detection(self, req):
        if self.digital_detector == None:
            self.digital_detector = DigitalDetecter()
        self.digital_detector.load_model()
        origin_image = self.msg_to_cv2(req.image)
        conf_map, paf = self.digital_detector.detect(origin_image)
        detect_result = {}
        detect_result["label_total"] = paf[0].item()
        detect_result["map_item"] = conf_map.tolist()
        detect_result = json.dumps(detect_result)
        return detect_result

    def handle_head_cam_digital_detection(self, req):
        try:
            msg = rospy.wait_for_message(HEAD_CAM_TOPIC, Image, 1)
        except rospy.ROSException:
            return "timeout"
        current_frame = self.msg_to_cv2(msg)
        if self.digital_detector == None:
            self.digital_detector = DigitalDetecter()
        self.digital_detector.load_model()
        conf_map, paf = self.digital_detector.detect(current_frame)
        detect_result = {}
        detect_result["label_total"] = paf[0].item()
        detect_result["map_item"] = conf_map.tolist()
        detect_result = json.dumps(detect_result)
        return detect_result

    def handle_ball_detection_in_area_srv(self, req):
        if self.ball_detect_in_area == None:
            self.ball_detect_in_area = DetectBallInArea()
            self.ball_detect_in_area.load_model()
        topic_name = req.topic_name
        try:
            img_msg = rospy.wait_for_message(topic_name, Image, timeout=2)
        except rospy.ROSException:
            return False, "wait for {} timeout.".format(topic_name)
        cv_img = self.msg_to_cv2(img_msg)
        detectRes = self.ball_detect_in_area.detect(cv_img)
        return True, json.dumps(detectRes)

    def run(self):
        rospy.init_node('ros_vision_node')

        # provide service for face detect
        face_detect_srv = rospy.Service('ros_vision_node/face_detect', FaceDetectService, self.handle_face_detect)
        face_verify_srv = rospy.Service('ros_vision_node/face_verify', FaceVerifyService, self.handle_face_verify)
        digital_detection_srv = rospy.Service('ros_vision_node/digital_detection', DigitalDetectSrv, self.handle_digital_detection)
        head_cam_digital_detection_srv = rospy.Service('ros_vision_node/head_cam_digital_detection', HeadCamDigitalDetectSrv, self.handle_head_cam_digital_detection)
        ball_detection_in_area_srv = rospy.Service('ros_vision_node/ball_detection_in_area', BallDetectInAreaSrv, self.handle_ball_detection_in_area_srv)
        print('ready to process vision request')
        rospy.spin()
if __name__ == '__main__':
    vision_node().run()
