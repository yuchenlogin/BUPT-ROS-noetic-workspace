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

NODE_NAME = 'identify_numbers_node'
CONTROL_ID = 2
SNAPSHOT_HEAD_URL = 'http://localhost:8080/snapshot?topic=/camera/color/image_raw'
POINTS = [[162, 60], [481, 60], [40, 380], [620, 380]]
PERSPECT_IMSHOW_FRAME_OFFSET = {"x": 100, "y": 100}
MODEL_NUMBER_IMG_DIR_PATH = '/home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/caai_roban_challenge/colleges/number_img/'
MODEL_NUMBER_IMG_FILEPATH = [MODEL_NUMBER_IMG_DIR_PATH + '1.jpg', MODEL_NUMBER_IMG_DIR_PATH + '2.jpg', MODEL_NUMBER_IMG_DIR_PATH + '3.jpg', MODEL_NUMBER_IMG_DIR_PATH + '4.jpg'] 
with open(os.path.join(SCRIPTS_PATH,"slam_identify_num.yaml"),"r")as f:
    SLAM_POINT=yaml.load(f)
    print(SLAM_POINT)
Adjust_len=0.2 #机器人在slam地图中往前看和往后看的距离差
is_debug = False
class Identifier():
    def __init__(self):
        pass
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
        dst = np.float32([[PERSPECT_IMSHOW_FRAME_OFFSET['x'], PERSPECT_IMSHOW_FRAME_OFFSET['y']], 
                    [origin_img_size[0] - PERSPECT_IMSHOW_FRAME_OFFSET['x'], PERSPECT_IMSHOW_FRAME_OFFSET['y']], 
                    [PERSPECT_IMSHOW_FRAME_OFFSET['x'], origin_img_size[1] - PERSPECT_IMSHOW_FRAME_OFFSET['y']], 
                    [origin_img_size[0] - PERSPECT_IMSHOW_FRAME_OFFSET['x'], origin_img_size[1] - PERSPECT_IMSHOW_FRAME_OFFSET['y']]])
        M = cv.getPerspectiveTransform(src, dst)
        M_inverse = cv.getPerspectiveTransform(dst, src)
        return M, M_inverse

    def img_perspect_transform(self,origin_img):
        M, M_inverse = self.cal_perspective_params(origin_img, POINTS)
        img_size = (origin_img.shape[1], origin_img.shape[0])
        perspective_img = cv.warpPerspective(origin_img, M, img_size)
        return perspective_img

    def filter_frames_position(self,frames_list):
        frames_array_sum = [0, 0]
        frames_array = np.array(frames_list)
        frames_array_lenght = len(frames_array)
        for frame_data in frames_array:
            frames_array_sum = frames_array_sum + frame_data
        center_position_of_frames = frames_array_sum / frames_array_lenght
        left_index = []
        right_index = []
        map_positoin_to_numbers = {}

        for index in range(frames_array_lenght):
            if (frames_array[index][0]) < center_position_of_frames[0]:
                left_index.append(index)
            else:
                right_index.append(index)

        if frames_array[left_index[0]][1] > frames_array[left_index[1]][1]:
            map_positoin_to_numbers[left_index[0] + 1] = 'Lower_left'
            map_positoin_to_numbers[left_index[1] + 1] = 'Upper_left'
        elif frames_array[left_index[0]][1] < frames_array[left_index[1]][1]:
            map_positoin_to_numbers[left_index[0] + 1] = 'Upper_left'
            map_positoin_to_numbers[left_index[1] + 1] = 'Lower_left'

        if frames_array[right_index[0]][1] > frames_array[right_index[1]][1]:
            map_positoin_to_numbers[right_index[0] + 1] = 'Lower_right'
            map_positoin_to_numbers[right_index[1] + 1] = 'Upper_right'
        elif frames_array[right_index[0]][1] < frames_array[right_index[1]][1]:
            map_positoin_to_numbers[right_index[0] + 1] = 'Upper_right'
            map_positoin_to_numbers[right_index[1] + 1] = 'Lower_right'
        return map_positoin_to_numbers

    def process_eye_img(self):
        global is_debug
        origin_img = self.get_eye_camera_img()
        perspective_img = self.img_perspect_transform(origin_img)
        gray_img = cv.cvtColor(perspective_img, cv.COLOR_BGR2GRAY)
        if is_debug == True:
            cv.imshow('origin_img', origin_img)
            cv.imshow('perspective_img', perspective_img)
            cv.imshow('gray_img', gray_img)
            cv.waitKey(0)
        return gray_img

    def process_model_numbers(self):
        global is_debug
        model_number_list = []
        for model_number_file in MODEL_NUMBER_IMG_FILEPATH:
            model_img = cv.imread(model_number_file, 0)
            model_img_resized = cv.resize(model_img, dsize=(256, 256), fx=1, fy=1, interpolation=cv.INTER_LINEAR)
            # _, model_img_threshold = cv.threshold(model_img_resized, 150, 255, cv.THRESH_BINARY)
            model_number_list.append(model_img_resized)
        if is_debug == True:
            for index, model_img in enumerate(model_number_list):
                cv.imshow(str(index), model_img)
            cv.waitKey(0)
        return model_number_list

    def matchTemplate(self,model_img_list, origin_img):
        global is_debug
        copy_img = origin_img.copy()
        upper_left_coordinate_of_matched_result = []
        for model_img in model_img_list:
            model_img_resized = cv.resize(model_img, dsize=(75, 75), fx=1, fy=1, interpolation=cv.INTER_LINEAR)
            w, h = model_img_resized.shape[::-1]
            template_matched = cv.matchTemplate(copy_img, model_img_resized, cv.TM_CCOEFF)
            min_val, max_val, min_loc, max_loc = cv.minMaxLoc(template_matched)
            upper_left_point = max_loc
            lower_right_point = (upper_left_point[0] + w, upper_left_point[1] + h)
            upper_left_coordinate_of_matched_result.append(upper_left_point)
            if is_debug == True:
                cv.rectangle(copy_img, lower_right_point, upper_left_point, 0, 2)
                cv.imshow('model_img_resized', copy_img)
                cv.waitKey(0)
        return upper_left_coordinate_of_matched_result

    def get_numbers_pos(self):
        map_positoin_to_numbers = {}
        gray_img = self.process_eye_img()
        model_number_list = self.process_model_numbers()
        upper_left_coordinate_of_matched_result = self.matchTemplate(model_number_list, gray_img)
        map_positoin_to_numbers = self.filter_frames_position(upper_left_coordinate_of_matched_result)
        return map_positoin_to_numbers

    def debug(self):
        global is_debug
        is_debug = True
        map_positoin_to_numbers = self.get_numbers_pos()
        print(map_positoin_to_numbers)
class Task_Identify_numbers(PublicNode):
    def __init__(self,nodename=NODE_NAME,control_id=CONTROL_ID):
        super(Task_Identify_numbers, self).__init__(nodename, control_id)
        self.identifier=Identifier()
    
    def path_plan(self,map_positoin_to_numbers):
        # [SLAM_POINT[map_positoin_to_numbers[i]][0] for i in range(1,5)]#生成路径点
        print([SLAM_POINT[map_positoin_to_numbers[i]][0] for i in range(1,5)])
        path_points=[]
        p_u=SLAM_POINT["Upper_left"][0]
        p_d=SLAM_POINT["Lower_left"][0]
        
        d_angle=math.atan2(p_u[0]-p_d[0],p_u[1]-p_d[1])
        dx=-math.sin(d_angle)*Adjust_len
        dy=-math.cos(d_angle)*Adjust_len
        print(d_angle,dx,dy)
        
        if "upper"in map_positoin_to_numbers[2].lower():#第二点在上方
            print("upper")
            for i in range(1,5):
                slampos=SLAM_POINT[map_positoin_to_numbers[i]][0]
                if i==3:
                    slampos[0]+=dx#加上偏置量
                    slampos[1]+=dy
                path_points.append(slampos)
        else:
            for i in range(1,5):
                slampos=SLAM_POINT[map_positoin_to_numbers[i]][0]
                if i==2:
                    slampos[0]+=dx/2#加上偏置量
                    slampos[1]+=dy/2
                path_points.append(slampos)

        return path_points

    def debug(self):
        self.bodyhub_ready()
        self.frame_action(RobanFrames.squat_frames)
        time.sleep(0.5)
        
        map_positoin_to_numbers =  self.identifier.get_numbers_pos()
        print(map_positoin_to_numbers)
        task1_path = self.path_plan(map_positoin_to_numbers)#路径规划
        print(task1_path)

    def start_identify_numbers(self):
        self.bodyhub_ready()
        self.set_arm_mode(1)
        # self.bodyhub_walk()#导航到起点
        # self.path_tracking(SLAM_POINT["TASK1_origin_POINT"],mode=1)
        self.bodyhub_ready()
        self.frame_action(RobanFrames.squat_frames)
        time.sleep(0.5)
        
        map_positoin_to_numbers =  self.identifier.get_numbers_pos()
        task1_path = self.path_plan(map_positoin_to_numbers)#路径规划
        print(task1_path)
        self.bodyhub_ready()
        self.set_head_rot([0, 10])
        self.bodyhub_walk()
        self.path_tracking(task1_path,pos_wait_mode=1,ignore_angle=True)

if __name__ == '__main__':
    task=Task_Identify_numbers()
    if len(sys.argv) >=2 and (sys.argv[1] == 'debug'):
        task.debug()
    else:
        task.start_identify_numbers()
        # Identifier().get_numbers_pos()