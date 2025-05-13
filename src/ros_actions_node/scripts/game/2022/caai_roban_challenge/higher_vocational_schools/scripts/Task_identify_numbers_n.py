#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2 as cv
import sys
if sys.version >= '3':
    import urllib.request as url
else:
    import urllib as url
import os
import math
import time
import numpy as np
import rospy
import rospkg
import yaml
from std_msgs.msg import *
from geometry_msgs.msg import *

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *
from frames import RobanFrames
from public import PublicNode
SCRIPTS_PATH=os.path.split(sys.argv[0])[0]

NODE_NAME = 'identify_numbers_n_node'
CONTROL_ID = 2
SNAPSHOT_HEAD_URL = 'http://localhost:8080/snapshot?topic=/camera/color/image_raw'
POINTS = [[135, 132], [487, 136], [0, 422], [640, 422]]
ROI_RANGE = [88, 124, 440, 270]
MIN_AREA_OF_NUMBER_FRAME = 5500
MODEL_NUMBER_IMG_DIR_PATH = '/home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/caai_roban_challenge/higher_vocational_schools/number_img/'
MODEL_NUMBER_IMG_FILEPATH = MODEL_NUMBER_IMG_DIR_PATH + '1.jpg' 
with open(os.path.join(SCRIPTS_PATH,"slam_identify_num_n.yaml"),"r")as f:
    SLAM_POINT=yaml.load(f)
    print(SLAM_POINT)
is_debug = False
class Identifier():
    def get_eye_camera_img(self):
        try:
            url_response = url.urlopen(SNAPSHOT_HEAD_URL)
        except IOError as err:
            rospy.logerr(err)
            exit(2)
        head_camera_img_data = np.asarray(bytearray(url_response.read()), dtype='uint8')
        head_camera_img = cv.imdecode(head_camera_img_data, cv.IMREAD_COLOR)
        return head_camera_img

    def cal_perspective_params(self,origin_img, points):
        origin_img_size = (origin_img.shape[1], origin_img.shape[0])
        src = np.float32(points)
        dst = np.float32([[0, 0], [origin_img_size[0], 0], [0, origin_img_size[1]], [origin_img_size[0], origin_img_size[1]]])
        M = cv.getPerspectiveTransform(src, dst)
        M_inverse = cv.getPerspectiveTransform(dst, src)
        return M, M_inverse

    def img_perspect_transform(self,origin_img):
        M, M_inverse = self.cal_perspective_params(origin_img, POINTS)
        img_size = (origin_img.shape[1], origin_img.shape[0])
        perspective_img = cv.warpPerspective(origin_img, M, img_size)
        return perspective_img

    def process_eye_img(self):
        global is_debug
        origin_img = self.get_eye_camera_img()
        perspective_img = self.img_perspect_transform(origin_img)
        gray_img = cv.cvtColor(perspective_img, cv.COLOR_BGR2GRAY)
        _, thresh = cv.threshold(gray_img, 150, 255, cv.THRESH_BINARY)
        if is_debug:
            cv.imshow('origin_img', origin_img)
            cv.imshow('perspective_img', perspective_img)
            cv.imshow('thresh', thresh)
            cv.waitKey(0)
        return thresh

    def match_number_one(self,model_img, origin_img):
        model_img_resized = cv.resize(model_img, dsize=(75, 75), fx=1, fy=1, interpolation=cv.INTER_LINEAR)
        w, h = model_img_resized.shape[::-1]
        template_matched = cv.matchTemplate(origin_img, model_img_resized, cv.TM_SQDIFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv.minMaxLoc(template_matched)
        upper_left_point = min_loc
        lower_right_point = (upper_left_point[0] + w, upper_left_point[1] + h)
        return upper_left_point, lower_right_point

    def get_num_n_pos(self):
        global is_debug
        origin_thresh_img = self.process_eye_img()
        model_img = cv.imread(MODEL_NUMBER_IMG_FILEPATH, 0)
        upper_left_point, lower_right_point = self.match_number_one(model_img, origin_thresh_img)
        if is_debug:
            cv.rectangle(origin_thresh_img, lower_right_point, upper_left_point, 0, 2)
            cv.imshow('match', origin_thresh_img)
            cv.waitKey(0)
        center_of_matched_in_origin_img = ((upper_left_point[0] + lower_right_point[0]) / 2, (upper_left_point[1] + lower_right_point[1]) / 2)
        center_of_origin_img = (origin_thresh_img.shape[::-1][0] / 2, origin_thresh_img.shape[::-1][1] / 2)
        
        if center_of_matched_in_origin_img[0] < center_of_origin_img[0]:
            if center_of_matched_in_origin_img[1] < center_of_origin_img[1]:
                map_location_result = 'Upper_left'
            else:
                map_location_result = 'Lower_left'
        else:
            if center_of_matched_in_origin_img[1] < center_of_origin_img[1]:
                map_location_result = 'Upper_right'
            else:
                map_location_result = 'Lower_right'
        return map_location_result

class Task_Identify_numbers(PublicNode):
    def __init__(self,nodename=NODE_NAME,control_id=CONTROL_ID):
        super(Task_Identify_numbers, self).__init__(nodename, control_id)
        self.identifier=Identifier()
    def start_identify_numbers_n(self):
        self.bodyhub_ready()
        self.set_arm_mode(1)
        # self.bodyhub_walk()#导航到起点
        # self.path_tracking(SLAM_POINT["TASK1_origin_POINT"],mode=1)
        self.bodyhub_ready()
        self.frame_action(RobanFrames.squat_frames)
        time.sleep(0.5)
        pos=self.identifier.get_num_n_pos()
        print(pos,SLAM_POINT[pos])
        self.bodyhub_ready()
        self.set_head_rot([0, 10])
        self.bodyhub_walk()
        self.path_tracking(SLAM_POINT[pos],pos_wait_mode=1)
        self.bodyhub_ready()
        time.sleep(2)


if __name__ == '__main__':
    Task_Identify_numbers().start_identify_numbers_n()