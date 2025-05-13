#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2 as cv
import urllib as url
import numpy as np

SNAPSHOT_HEAD_URL = 'http://localhost:8080/snapshot?topic=/camera/color/image_raw'
POINTS = [[150, 40], [487, 40], [0, 422], [640, 422]]
ROI_RANGE = [88, 124, 440, 270]
MIN_AREA_OF_NUMBER_FRAME = 5500
MODEL_NUMBER_IMG_DIR_PATH = '/home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/caai_roban_challenge/higher_vocational_schools/number_img/'

class Identifier():
    def __init__(self, digit):
        self.model_number_img_filepath = MODEL_NUMBER_IMG_DIR_PATH + str(digit) + ".jpg"
    
    def get_eye_camera_img(self):
        try:
            url_response = url.urlopen(SNAPSHOT_HEAD_URL)
        except IOError as err:
            print(err)
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
        origin_img = self.get_eye_camera_img()
        perspective_img = self.img_perspect_transform(origin_img)
        gray_img = cv.cvtColor(perspective_img, cv.COLOR_BGR2GRAY)
        _, thresh = cv.threshold(gray_img, 130, 255, cv.THRESH_BINARY)
        return thresh

    def match_number_one(self,model_img, origin_img):
        model_img_resized = cv.resize(model_img, dsize=(90, 90), fx=1, fy=1, interpolation=cv.INTER_LINEAR)
        w, h = model_img_resized.shape[::-1]
        template_matched = cv.matchTemplate(origin_img, model_img_resized, cv.TM_SQDIFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv.minMaxLoc(template_matched)
        upper_left_point = min_loc
        lower_right_point = (upper_left_point[0] + w, upper_left_point[1] + h)
        return upper_left_point, lower_right_point

    def get_num_n_pos(self):
        print(self.model_number_img_filepath)
        origin_thresh_img = self.process_eye_img()
        model_img = cv.imread(self.model_number_img_filepath, 0)
        upper_left_point, lower_right_point = self.match_number_one(model_img, origin_thresh_img)
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

if __name__ == '__main__':
    identify = Identifier(digit=1)
    map_position = identify.get_num_n_pos()