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

FRUIT_CLASSIFICATION_MODEL_FILE_NAME = 'fruit_new.onnx'
FRUIT_CLASSIFICATION_MODEL_FOLDER = os.path.join(rospkg.RosPack().get_path('ros_fruit_node'), 'models')
FRUIT_CLASSIFICATION_MODEL = os.path.join(FRUIT_CLASSIFICATION_MODEL_FOLDER, FRUIT_CLASSIFICATION_MODEL_FILE_NAME)
FRUIT_CLASSIFICATION_MODEL_MD5 = "4470ba593e9aa90f23301d81ee0fb454"
MODEL_DOWNLOAD_URL = "https://roban.lejurobot.com/modols/fruit_new.onnx"
CONFIG_FILE = os.path.join(rospkg.RosPack().get_path('ros_fruit_node'), 'configs', 'config.json')

CONFIG_MODEL_PARAMETERS_KEY = 'model_parameters'
CONFIG_INPUT_RESIZE_KEY = 'input_resize'
CONFIG_INPUT_RESIZE_WIDTH_KEY = 'width'
CONFIG_INPUT_RESIZE_HEIGHT_KEY = 'height'
CONFIG_MODEL_MEAN_KEY = 'mean'
CONFIG_MODEL_STD_KEY = 'std'
TRANSPOSE_ORDER = (2, 0, 1)
CONFIG_LABEL_KEY = 'label'

class FruitClassification():
    def __init__(self):
        self.__model = None
        self.config = self.load_config()
        self.input_resized_width = self.config[CONFIG_MODEL_PARAMETERS_KEY][CONFIG_INPUT_RESIZE_KEY][CONFIG_INPUT_RESIZE_WIDTH_KEY]
        self.input_resized_height = self.config[CONFIG_MODEL_PARAMETERS_KEY][CONFIG_INPUT_RESIZE_KEY][CONFIG_INPUT_RESIZE_HEIGHT_KEY]
        self.mean = self.config[CONFIG_MODEL_PARAMETERS_KEY][CONFIG_MODEL_MEAN_KEY]
        self.std = self.config[CONFIG_MODEL_PARAMETERS_KEY][CONFIG_MODEL_STD_KEY]
        self.fruit_label = self.config[CONFIG_LABEL_KEY]

    def load_config(self):
        return json.loads(open(CONFIG_FILE, 'r').read())

    def is_model_loaded(self):
        return True if self.__model != None else False

    def load_model(self):
        if self.is_model_loaded():
            return
        if os.path.exists(FRUIT_CLASSIFICATION_MODEL):
            if url_downloading.md5sum(FRUIT_CLASSIFICATION_MODEL) == FRUIT_CLASSIFICATION_MODEL_MD5:
                self.__model = rt.InferenceSession(FRUIT_CLASSIFICATION_MODEL)
                return
            else:
                os.remove(FRUIT_CLASSIFICATION_MODEL)

        download_result = url_downloading.download_from_url(MODEL_DOWNLOAD_URL, FRUIT_CLASSIFICATION_MODEL_FOLDER, FRUIT_CLASSIFICATION_MODEL_FILE_NAME, FRUIT_CLASSIFICATION_MODEL_MD5)
        if download_result == True:
            self.__model = rt.InferenceSession(FRUIT_CLASSIFICATION_MODEL)
        else:
            exit("下载 {} 模型失败！".format(FRUIT_CLASSIFICATION_MODEL_FILE_NAME))

    def __normalize(self, im):
        im = im.astype(np.float32, copy=False) / 255.0
        im -= self.mean
        im /= self.std
        return im

    def matching_fruit(self, predict_results):
        fruit_is = self.fruit_label[np.argmax(predict_results)]
        confidence = np.max(predict_results)
        return fruit_is, confidence

    def run(self, image, confidence_threshold):
        image_resized = cv.resize(image, (self.input_resized_width, self.input_resized_height))
        image_normalized = self.__normalize(image_resized)
        image_transpose = np.transpose(image_normalized, TRANSPOSE_ORDER)
        input_feed = {self.__model.get_inputs()[0].name: [image_transpose]}
        predict_results = self.__model.run(None, input_feed)
        fruit_is, confidence = self.matching_fruit(predict_results)
        return 'None' if confidence < confidence_threshold else fruit_is

if __name__ == '__main__':
    fc = FruitClassification()
    fc.load_model()
