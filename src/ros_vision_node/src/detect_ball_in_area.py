#!/usr/bin/env python3
# coding=utf-8

import onnxruntime as rt
import os, rospkg
import numpy as np
import cv2 as cv
import sys
sys.path.append(os.path.join(rospkg.RosPack().get_path("leju_lib_pkg"), "scripts"))
import url_downloading

MODEL_DOWNLOAD_URL = "https://roban.lejurobot.com/modols/ball_detect_in_area.onnx"
DIGITAL_DETECTION_MODEL_MD5 = "a31d2ef4b113d607b69a7382f1f9b29b"
DIGITAL_DETECTION_MODEL_FILE_NAME = "ball_detect_in_area.onnx"
DIGITAL_DETECTION_MODEL_FOLDER = os.path.join(rospkg.RosPack().get_path('ros_vision_node'), "src/models")
DIGITAL_DETECTION_MODEL = os.path.join(DIGITAL_DETECTION_MODEL_FOLDER, DIGITAL_DETECTION_MODEL_FILE_NAME)
MODEL_INPUT_RESOLUTION = (640, 480)
MEAN = 0.5
STANDARD_DEVIATION = 0.5
LABEL_MAINAREA = 1
LABEL_BALL = 2
KERNEL_SIZE = (9, 9)

class DetectBallInArea():
    def __init__(self):
        self.model = None

    def load_model(self):
        if self.model is not None:
            return
        if os.path.exists(DIGITAL_DETECTION_MODEL):
            if url_downloading.md5sum(DIGITAL_DETECTION_MODEL) == DIGITAL_DETECTION_MODEL_MD5:
                self.model = rt.InferenceSession(DIGITAL_DETECTION_MODEL)
            else:
                os.remove(DIGITAL_DETECTION_MODEL)
        download_result = url_downloading.download_from_url(MODEL_DOWNLOAD_URL, DIGITAL_DETECTION_MODEL_FOLDER, DIGITAL_DETECTION_MODEL_FILE_NAME, DIGITAL_DETECTION_MODEL_MD5)
        if download_result != False:
            self.model = rt.InferenceSession(DIGITAL_DETECTION_MODEL)
        else:
            exit("下载识别小球在区域内的模型失败！")

    def __normalize(self, im, mean, std):
        im = im.astype(np.float32, copy=False) / 255.0
        im -= mean
        im /= std
        return im

    def detect(self, image):
        ball_mask_center = None
        if image is None:
            raise ValueError("Input `image` is none type.")
        if not isinstance(image, np.ndarray):
            raise TypeError("`Image` type is not numpy.")
        rgb = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        rgb_transpose = rgb.transpose(2, 0, 1)
        input_image = self.__normalize(rgb_transpose, MEAN, STANDARD_DEVIATION)
        input_feed = {self.model.get_inputs()[0].name: [input_image]}

        results = self.model.run(None, input_feed)

        result = np.squeeze(results)

        main_area_mask = (result==LABEL_MAINAREA).astype(np.uint8)
        ball_mask = (result==LABEL_BALL).astype(np.uint8)

        M = cv.moments(ball_mask)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            ball_mask_center = (cx, cy)

        kernel = np.ones(KERNEL_SIZE, np.uint8)
        dilated_main_area_mask = cv.dilate(main_area_mask, kernel, iterations=1)

        ball_pixels = np.count_nonzero(ball_mask)
        intersection_mask = cv.bitwise_and(ball_mask, dilated_main_area_mask)
        intersection_pixels = np.count_nonzero(intersection_mask)

        if ball_pixels > 0:
            overlap_percent = intersection_pixels / ball_pixels * 100
        else:
            overlap_percent = 0.0
        return {"overlap_percent": overlap_percent, "ball_mask_center": ball_mask_center}

if __name__ == "__main__":
    db = DetectBallInArea()
    db.load_model()
    img = cv.imread("/home/lemon/robot_ros_application/catkin_ws/src/ros_vision_node/src/ttt3.jpg", cv.IMREAD_COLOR).astype('float32')
    overlap_percent = db.detect(img)
    print(overlap_percent)
