#!/usr/bin/env python3
# coding=utf-8

import rospkg
import onnxruntime as rt
import os, sys
import json
import numpy as np
import cv2 
from cv_bridge import CvBridge
sys.path.append(os.path.join(rospkg.RosPack().get_path("leju_lib_pkg"), "scripts"))  
import url_downloading 

FALL_DETECTION_MODEL_FILE_NAME = 'fall_model.onnx'
FALL_DETECTION_MODEL_FOLDER = os.path.join(rospkg.RosPack().get_path('ros_fall_detection'), 'models')
FALL_CLASSIFICATION_MODEL = os.path.join(FALL_DETECTION_MODEL_FOLDER, FALL_DETECTION_MODEL_FILE_NAME)
FALL_CLASSIFICATION_MODEL_MD5 = "4470ba593e9aa90f23301d81ee0fb454"
MODEL_DOWNLOAD_URL = "https://roban.lejurobot.com/modols/fall_model.onnx"
CONFIG_FILE = os.path.join(rospkg.RosPack().get_path('ros_fall_detection'), 'configs', 'config.json') 

CONFIG_MODEL_PARAMETERS_KEY = 'model_parameters'  # 配置文件中模型参数的键名
CONFIG_INPUT_RESIZE_KEY = 'input_resize'  # 配置文件中输入图像尺寸调整参数的键名
CONFIG_INPUT_RESIZE_WIDTH_KEY = 'width'  # 配置文件中输入图像宽度调整参数的键名
CONFIG_INPUT_RESIZE_HEIGHT_KEY = 'height'  # 配置文件中输入图像高度调整参数的键名
CONFIG_MODEL_MEAN_KEY = 'mean'  # 配置文件中模型均值参数的键名
CONFIG_MODEL_STD_KEY = 'std'  # 配置文件中模型标准差参数的键名
TRANSPOSE_ORDER = (2, 0, 1)  # 输入图像维度转置顺序
CONFIG_LABEL_KEY = 'label'  # 配置文件中标签的键名

class FallDetection():  # 定义跌倒检测类
    def __init__(self):  
        self.__model = None 
        self.config = self.load_config()
        self.bridge = CvBridge()
        self.input_resized_width = self.config[CONFIG_MODEL_PARAMETERS_KEY][CONFIG_INPUT_RESIZE_KEY][CONFIG_INPUT_RESIZE_WIDTH_KEY]  # 获取输入图像调整后的宽度
        self.input_resized_height = self.config[CONFIG_MODEL_PARAMETERS_KEY][CONFIG_INPUT_RESIZE_KEY][CONFIG_INPUT_RESIZE_HEIGHT_KEY]  # 获取输入图像调整后的高度
        self.mean = self.config[CONFIG_MODEL_PARAMETERS_KEY][CONFIG_MODEL_MEAN_KEY]  # 获取模型均值参数
        self.std = self.config[CONFIG_MODEL_PARAMETERS_KEY][CONFIG_MODEL_STD_KEY]  # 获取模型标准差参数
        self.fall_label = self.config[CONFIG_LABEL_KEY]  # 获取跌倒标签

    def load_config(self): 
        return json.loads(open(CONFIG_FILE, 'r').read())  # 以JSON格式读取配置文件内容，并返回解析后的字典对象

    def is_model_loaded(self): 
        return True if self.__model != None else False  # 判断私有变量__model是否为None，若不为None则返回True，否则返回False

    def load_image(self, image):
        # 将ROS图像消息转换为OpenCV图像
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        return img

    def load_model(self):
        if self.is_model_loaded():
            return
        if os.path.exists(FALL_CLASSIFICATION_MODEL):
            if url_downloading.md5sum(FALL_CLASSIFICATION_MODEL) == FALL_CLASSIFICATION_MODEL_MD5:
                self.__model = rt.InferenceSession(FALL_CLASSIFICATION_MODEL)
                return
            else:
                os.remove(FALL_CLASSIFICATION_MODEL)

        download_result = url_downloading.download_from_url(MODEL_DOWNLOAD_URL, FALL_DETECTION_MODEL_FOLDER, FALL_CLASSIFICATION_MODEL, FALL_CLASSIFICATION_MODEL_MD5)
        if download_result == True:
            self.__model = rt.InferenceSession(FALL_CLASSIFICATION_MODEL)
        else:
            exit("下载 {} 模型失败！".format(FALL_DETECTION_MODEL_FILE_NAME))

    def normalize(self, image):  # 归一化方法
        input_shape = (self.input_resized_width, self.input_resized_height)  # 输入图像形状
        image = cv2.resize(image, input_shape)  # 调整输入图像大小为指定尺寸
        image = image.astype(np.float32)/255.0  # 将图像转换为浮点型，并进行归一化处理
        image = (image - self.mean) / self.std  # 对图像进行均值归一化和标准差归一化
        image = np.transpose(image, (2, 0, 1))  # 将图像维度转置为指定顺序
        image = np.expand_dims(image, axis=0)  # 在第一维度上扩展维度
        return image 

    def postprocess_output(self, output, draw_threshold):  # 后处理模型输出方法
        label = self.config[CONFIG_LABEL_KEY]  # 获取标签
        fall_class_index = 1  # 摔倒类别的索引，根据你的输出来设置
        confidence = output[0][fall_class_index]  # 获取跌倒类别的置信度
        if confidence >= draw_threshold:  # 判断置信度是否大于等于指定阈值
            return label, confidence  # 返回标签和置信度
        else:
            return "normal", confidence  # 若置信度小于指定阈值，则返回"normal"和置信度

    def detect_fall(self, image, draw_threshold):  # 检测跌倒方法
        model_path = os.path.join(FALL_DETECTION_MODEL_FOLDER, FALL_DETECTION_MODEL_FILE_NAME)  # 定义模型文件路径
        session = rt.InferenceSession(model_path)  # 创建onnxruntime会话，加载模型

        image = self.load_image(image)  # 加载图像
        input_data = self.normalize(image)  # 对图像进行归一化处理
        input_data = np.float32(input_data)  # 转换输入数据类型为float32

        input_names = [input.name for input in session.get_inputs()]  # 获取模型输入名称列表
        im_shape_name = input_names[0]  # 输入图像形状名称
        input_data_name = input_names[1]  # 输入数据名称
        scale_factor_name = input_names[2]  # 缩放因子名称

        im_shape = np.array(input_data.shape[2:], dtype=np.int64)  # 输入图像形状
        scale_factor = np.array([[1.0, 1.0]], dtype=np.float32)  # 缩放因子
        
        input_dict = {
            im_shape_name: [im_shape],
            input_data_name: input_data,
            scale_factor_name: scale_factor
        }  # 构造输入字典

        output_names = [output.name for output in session.get_outputs()]  # 获取模型输出名称列表
        output = session.run(output_names, input_dict)  # 运行模型，获取输出结果
        
        predicted_label, confidence = self.postprocess_output(output[0], draw_threshold)  # 对输出结果进行后处理
        if predicted_label == self.fall_label:  # 判断预测标签是否为跌倒标签
            return "fall", confidence  # 若预测标签为跌倒标签，则返回"fall"和置信度
        else:
            return "normal", confidence  # 若预测标签不是跌倒标签，则返回"normal"和置信度


if __name__ == '__main__':
    fd = FallDetection()  # 创建跌倒检测对象
    draw_threshold = 0.3  # 阈值
    fd.load_model()  # 加载模型
