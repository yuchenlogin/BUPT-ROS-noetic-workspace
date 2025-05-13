#!/usr/bin/env python3
# coding=utf-8
import json
import random
import sys
import os
import face_recognition
import cv2
import time
import numpy as np
# from keras.models import load_model
from onnxruntime import InferenceSession
import rospy
import rospkg
sys.path.append(rospkg.RosPack().get_path('ros_vision_node'))
MODELS_PATH = os.path.split(sys.argv[0])[0]+"/models/"

genderList = ['Male', 'Female']


class Face_detecter():
    """人脸识别模块"""
    def __init__(self,display = False):
        self.recognized_face = {}  # 记录认识的人脸名字和encoding
        # self.recognize("fandes", "/home/lemon/testfiles/1.jpg")
        self.display = display
        self.current_image = np.zeros((480, 640, 3))
        # self.emotion_model = load_model("./model_v6_23.hdf5")
        # Load network
        self.age_gender_model = InferenceSession("/home/lemon/robot_ros_application/catkin_ws/src/ros_vision_node/src/models/age_gender_mobilenet_v2_onnx6.onnx")
        self.emotion_model = InferenceSession(
            "/home/lemon/robot_ros_application/catkin_ws/src/ros_vision_node/src/models/emotion_mobilenet_v2_onnx6.onnx")
        self.current_locations = []
        self.current_faces = []

    def recognize(self, name, faceimage):
        """记下人脸的encoding"""
        if type(faceimage) == str:
            faceimage = face_recognition.load_image_file(faceimage)
        elif type(faceimage) != np.ndarray:
            print("传入人脸数据错误")
            return 1
        self.recognized_face[name] = self.locate_faces(faceimage)[1][0]
        return 0

    def get_distance(self,face0_encoding,face1_encoding):
        face0_encoding = np.array([face0_encoding])
        face_distances = face_recognition.face_distance(face0_encoding, face1_encoding)
        return face_distances
        
    def locate_faces(self, rgb_frame,match_times=0):
        """获取图片中所有人脸位置以及对应的encoding"""
        face_locations = face_recognition.face_locations(rgb_frame,number_of_times_to_upsample=match_times)
        face_encodings = face_recognition.face_encodings(
            rgb_frame, face_locations, num_jitters=2)
        return face_locations, face_encodings

    def extract_faces(self, frame, face_locations,padding=0):
        """提取人脸区域的图像"""
        faces = []
        h, w, c = frame.shape
        for face_location in face_locations:
            top, right, bottom, left = face_location
            # 将脸部取出
            face_image = frame[top-padding if top-padding >= 0 else 0:bottom+padding if bottom+padding < h else h,
                               left-padding if left-padding > 0 else 0:right+padding if left+padding < w else w]
            faces.append(face_image)
        return faces

    def find_face_and_name(self, frame, match_tolerance=0.42):
        """查找人脸并尝试寻找名字"""
        face_locations, face_encodings = self.locate_faces(frame)
        face_names = []
        known_face_encodings = list(self.recognized_face.values())
        known_face_names = list(self.recognized_face.keys())
        for face_encoding in face_encodings:
            # 默认为unknown
            if len(known_face_encodings):
                self.get_distance(known_face_encodings,face_encoding)
                matches = face_recognition.compare_faces(
                    known_face_encodings, face_encoding, tolerance=match_tolerance)
                name = known_face_names[matches.index(
                    True)]if True in matches else "unknown"
            else:
                name = "unknown"
            face_names.append(name)
        self.current_locations = face_locations
        if self.display:
            for (top, right, bottom, left), name in zip(face_locations, face_names):
                # Scale back up face locations since the frame we detected in was scaled to 1/4 size
                # 矩形框
                cv2.rectangle(self.current_image, (left, top),
                              (right, bottom), (0, 0, 255), 1)
                # 加上标签
                # cv2.rectangle(self.current_image, (left, bottom-15), (right, bottom), (0, 0, 255), cv2.FILLED)
                cv2.putText(self.current_image, name, (right, top),
                            cv2.FONT_HERSHEY_DUPLEX, 0.7, (255, 255, 255), 2)
        return face_locations, face_encodings, face_names

    def detect_emotions(self, face_images):
        """检测人脸情绪"""
        result = []
        for i, face_image in enumerate(face_images):
            show_face = face_image.copy()
            face_image = cv2.cvtColor(face_image, cv2.COLOR_RGB2GRAY)
            # 调整到可以进入该模型输入的大小
            face_image = cv2.cvtColor(face_image, cv2.COLOR_GRAY2RGB)
            # 缩小使用cv2.INTER_AREA，放大使用cv2.INTER_CUBIC(较慢)和cv2.INTER_LINEAR(较快效果也不错)。
            face_image = cv2.resize(
                face_image, (224, 224), interpolation=cv2.INTER_CUBIC)
            face_image = face_image.transpose(2, 0, 1)
            face_image = face_image.astype('float32')
            face_image = face_image / 255.
            face_image = face_image * 2.0 - 1.0
            # print(face_image)

            input_data = np.array([face_image])
            res = self.emotion_model.run(
                output_names=None, input_feed={'image': input_data})
            # 载入分析结果
            predicted_class = np.argmax(res)
            # 分类情绪
            label_map = {0: 'Sad', 1: 'Disgust', 2: 'Happy',
                         3: 'Fear', 4: 'Surprise', 5: 'Neutral', 6: 'Angry'}
            predicted_label = label_map[predicted_class]
            # 根据情绪映射表输出情绪
            # print(predicted_label)
            result.append(predicted_label)
            if self.display:
                top, right, bottom, left = self.current_locations[i]
                cv2.rectangle(self.current_image, (left, top),
                              (right, bottom), (0, 0, 255), 2)
                # 加上标签
                # cv2.imwrite("image/{}{}.jpg".format(predicted_label[:2].upper(),random.randint(100000999999999, 999999999999999)),gray_image)
                # cv2.rectangle(self.current_image, (left, bottom-15), (right, bottom), (0, 0, 255), cv2.FILLED)
                cv2.putText(self.current_image, predicted_label, (right,
                            top+20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return result
    
    def label_to_gender_age(self,idlabel):
        """从模型输出转化为年龄性别"""
        # 分为42类,奇偶判断性别,数值判断年龄阶段,如label为4则性别Male,年龄10-15岁,以此类推
        age = str((idlabel//2)*5)+"-"+str((idlabel//2+1)*5)
        gender = genderList[idlabel%2]
        
        return age,gender
    
    def detect_age_gender(self, faces):
        """检测年龄性别"""
        results = []
        for i, face_image in enumerate(faces):
            # 预处理
            show_face = face_image.copy()
            face_image = cv2.cvtColor(face_image, cv2.COLOR_RGB2GRAY)
            # 调整到可以进入该模型输入的大小
            face_image = cv2.cvtColor(face_image, cv2.COLOR_GRAY2RGB)
            # 缩小使用cv2.INTER_AREA， 放大使用cv2.INTER_CUBIC(较慢)和cv2.INTER_LINEAR(较快效果也不错)。
            face_image = cv2.resize(
                face_image, (48, 48), interpolation=cv2.INTER_AREA)
            face_image = face_image.transpose(2, 0, 1)
            face_image = face_image.astype('float32')
            face_image = face_image / 255.
            face_image = face_image * 2.0 - 1.0
            
            input_data = np.array([face_image])
            res = self.age_gender_model.run(
                output_names=None, input_feed={'image': input_data})
            # 载入分析结果
            predicted_class = np.argmax(res)
            age,gender= self.label_to_gender_age(predicted_class)

            label = (gender, age)
            results.append(label)
            if self.display:
                top, right, bottom, left = self.current_locations[i]
                cv2.putText(self.current_image, str(label), (right, top+40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
        return results
    
    def detect_a_frame(self,frame):
        """检测一帧, 返回检测结果
        :param 
            frame : a frame
        :return 
            result_dict 
                face_token: 某个人脸唯一的enconding
                emotion: 人脸情绪判断,有七种,{0: 'Sad', 1: 'Disgust', 2: 'Happy',3: 'Fear', 4: 'Surprise', 5: 'Neutral', 6: 'Angry'}
                gender: 性别判断, ['Male', 'Female']
                age: 年龄判断, 每5年一个分段,(0-5)、(6-10)、...(101-), 共21段
                location:
                    width: 人脸宽度
                    height: 人脸高度
                    location: 具体位置, top, right, bottom, left
        """
        self.current_image = frame
        result = []
        frame = np.ascontiguousarray(self.current_image[:, :, ::-1])
        face_areas, face_encodings, face_names = self.find_face_and_name(frame)
        faces_images = self.extract_faces(frame, face_areas)
        self.current_faces = faces_images.copy()
        emotions = self.detect_emotions(faces_images)
        gender_ages = self.detect_age_gender(faces_images)
        for enconding,name,area,emotion,gender_age in zip(face_encodings,face_names,face_areas,emotions,gender_ages):
            face = {}
            top, right, bottom, left = area
            width = right - left
            height = bottom - top
            gender,age = gender_age
            # print("{}{}{}{}{}".format(len(enconding),name,area,emotion,gender_age))
            face["face_token"]=enconding.tolist()
            face["name"] = name
            face["gender"] = gender
            face["age"] = age
            face["emotion"] = emotion
            face["location"] = {"width":width, "height":height,"location":area}
            result.append(face)
        return json.dumps(result)
    
    def draw_result(self):
        for face in self.current_faces:
            cv2.imshow("faces",face)
        cv2.imshow('vision_node_drawer', self.current_image)
        cv2.waitKey(1)
        
    def detect_by_camera(self, id=2):
        """打开摄像头检测"""
        video_capture = cv2.VideoCapture(id)
        print("摄像头已开启")
        while True:
            # 读取摄像头画面
            ret, self.current_image = video_capture.read()

            if ret:
                stime = time.time()
                res = self.detect_a_frame(self.current_image)
                print(res)
                # frame = np.ascontiguousarray(self.current_image[:, :, ::-1])

                # face_areas, face_encodings, face_names = self.find_face_and_name(frame)
                # faces_images = self.extract_faces(frame, face_areas)
                # self.detect_emotions(faces_images)
                # self.detect_age_gender(faces_images)
                # print("detect time: {:.5f}".format(time.time()-stime))
                # Display
                cv2.imshow('monitor', self.current_image)
                # 按Q退出
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        video_capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    dt = Face_detecter()
    dt.detect_by_camera(2)
