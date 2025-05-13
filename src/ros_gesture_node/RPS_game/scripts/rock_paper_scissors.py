#!/usr/bin/env python3
# coding=utf-8

import rospkg
import onnxruntime as rt
import os, sys
import json
import numpy as np
import cv2 as cv

sys.path.append(os.path.join(rospkg.RosPack().get_path("leju_lib_pkg"), "scripts"))
import url_downloading
from custom_constants import Constants

ROCKPAPERSCISSORS_MODEL_FILE_NAME = 'rock_paper_scissors.onnx'
ROCKPAPERSCISSORS_MODEL_FOLDER = os.path.join(rospkg.RosPack().get_path('ros_gesture_node'), 'RPS_game', 'models')
ROCKPAPERSCISSORS_MODEL = os.path.join(ROCKPAPERSCISSORS_MODEL_FOLDER, ROCKPAPERSCISSORS_MODEL_FILE_NAME)
ROCKPAPERSCISSORS_MODEL_MD5 = "772a4536cbe59c907c5cc09cb6b34ddd"
MODEL_DOWNLOAD_URL = "https://roban.lejurobot.com/modols/rock_paper_scissors.onnx"
DOWNLOAD_SUCCEED = True

CONFIG_MODEL_PARAMETERS_KEY = 'model_parameters'
CONFIG_INPUT_RESIZE_KEY = 'input_resize'
CONFIG_INPUT_RESIZE_WIDTH_KEY = 'width'
CONFIG_INPUT_RESIZE_HEIGHT_KEY = 'height'
CONFIG_MODEL_MEAN_KEY = 'mean'
CONFIG_MODEL_STD_KEY = 'std'
TRANSPOSE_ORDER = (2, 0, 1)
CONFIG_LABEL_KEY = 'label'

class RockPaperScissors():
    def __init__(self):
        self.load_model()
        self.load_config()

    def load_config(self):
        configs = json.loads(open(Constants.config_file, 'r').read())
        model_parameters = configs[CONFIG_MODEL_PARAMETERS_KEY]
        self.input_resized_width = model_parameters[CONFIG_INPUT_RESIZE_KEY][CONFIG_INPUT_RESIZE_WIDTH_KEY]
        self.input_resized_height = model_parameters[CONFIG_INPUT_RESIZE_KEY][CONFIG_INPUT_RESIZE_HEIGHT_KEY]
        self.mean = model_parameters[CONFIG_MODEL_MEAN_KEY]
        self.std = model_parameters[CONFIG_MODEL_STD_KEY]
        self.label = configs[CONFIG_LABEL_KEY]

    def load_model(self):
        model_path = ROCKPAPERSCISSORS_MODEL
        if os.path.exists(model_path):
            if url_downloading.md5sum(model_path) == ROCKPAPERSCISSORS_MODEL_MD5:
                try:
                    self.model = rt.InferenceSession(model_path)
                    return
                except Exception as e:
                    print("加载 {} 模型失败!\n{}".format(ROCKPAPERSCISSORS_MODEL_FILE_NAME, e))
                    exit(1)
            else:
                os.remove(model_path)
        download_result = url_downloading.download_from_url(
            MODEL_DOWNLOAD_URL,
            ROCKPAPERSCISSORS_MODEL_FOLDER,
            ROCKPAPERSCISSORS_MODEL_FILE_NAME,
            ROCKPAPERSCISSORS_MODEL_MD5
        )
        if download_result == DOWNLOAD_SUCCEED:
            try:
                self.model = rt.InferenceSession(model_path)
            except Exception as e:
                print("加载 {} 模型失败!\n{}".format(ROCKPAPERSCISSORS_MODEL_FILE_NAME, e))
                exit(1)
        else:
            exit("下载 {} 模型失败！".format(ROCKPAPERSCISSORS_MODEL_FILE_NAME))

    def __normalize(self, im):
        im = im.astype(np.float32, copy=False) / 255.0
        im -= self.mean
        im /= self.std
        return im

    def matching_gesture(self, predict_results):
        rps_is = self.label[np.argmax(predict_results)]
        confidence = np.max(predict_results)
        return rps_is, confidence

    def run(self, image, confidence_threshold):
        image_resized = cv.resize(image, (self.input_resized_width, self.input_resized_height))
        image_normalized = self.__normalize(image_resized)
        image_transpose = np.transpose(image_normalized, TRANSPOSE_ORDER)
        input_feed = {self.model.get_inputs()[0].name: [image_transpose]}
        predict_results = self.model.run(None, input_feed)
        rps_is, confidence = self.matching_gesture(predict_results)
        return 'None' if confidence < confidence_threshold else rps_is

if __name__ == '__main__':
    rps = RockPaperScissors()
