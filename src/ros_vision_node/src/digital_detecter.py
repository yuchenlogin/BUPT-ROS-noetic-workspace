#!/usr/bin/env python3
# coding=utf-8

import onnxruntime as rt
import os, rospkg
import cv2 as cv
import numpy as np
import sys
sys.path.append(os.path.join(rospkg.RosPack().get_path("leju_lib_pkg"), "scripts"))
import url_downloading

MODEL_DOWNLOAD_URL = "https://roban.lejurobot.com/modols/digital_detection.onnx"
DIGITAL_DETECTION_MODEL_FILE_NAME = "digital_detection.onnx"
DIGITAL_DETECTION_MODEL_FOLDER = os.path.join(rospkg.RosPack().get_path('ros_vision_node'), "src/models")
DIGITAL_DETECTION_MODEL = os.path.join(DIGITAL_DETECTION_MODEL_FOLDER, DIGITAL_DETECTION_MODEL_FILE_NAME)
DIGITAL_DETECTION_MODEL_MD5 = "48ea1eb83e0222d19ae4763f1a8f3aad"
MODEL_INPUT_RESOLUTION = (608, 608)

class DigitalDetecter():
    def __init__(self, is_display=False):
        self.model = None
        self.is_display = is_display
        self.draw_detection_result_color = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 0, 0)]

    def load_model(self):
        download_result = url_downloading.download_from_url(MODEL_DOWNLOAD_URL, DIGITAL_DETECTION_MODEL_FOLDER, DIGITAL_DETECTION_MODEL_FILE_NAME, DIGITAL_DETECTION_MODEL_MD5)
        if download_result == False:
            exit("下载失败数字检测模型失败！")
        if self.model == None:
            self.model = rt.InferenceSession(DIGITAL_DETECTION_MODEL)

    def detect(self, image):
        image_resized = cv.resize(image, MODEL_INPUT_RESOLUTION)
        rgb = cv.cvtColor(image_resized, cv.COLOR_BGR2RGB)
        input_image = rgb.transpose(2, 0, 1)
        input_image = input_image.astype(np.float32) / 255.0
        input_feed = {"im_shape": [image_resized.shape[:2]], "image": [input_image], "scale_factor":[[MODEL_INPUT_RESOLUTION[0]/image.shape[0], MODEL_INPUT_RESOLUTION[1]/image.shape[1]]]}
        result = self.model.run(None, input_feed)
        conf_map, paf = result
        if self.is_display:
            draw_detection_result = image.copy()
            for _ in conf_map:
                cv.rectangle(draw_detection_result, (_[2], _[3]), (_[4], _[5]), self.draw_detection_result_color[int(_[0])], 2)
            cv.imshow("detection_result", draw_detection_result)
            cv.waitKey(0)
        return conf_map, paf

if __name__ == "__main__":
    digital_detecter = DigitalDetecter(True)
    digital_detecter.load_model()
    img = cv.imread("/home/lemon/robot_ros_application/catkin_ws/src/ros_vision_node/src/test.png")
    conf_map, paf = digital_detecter.detect(img)
