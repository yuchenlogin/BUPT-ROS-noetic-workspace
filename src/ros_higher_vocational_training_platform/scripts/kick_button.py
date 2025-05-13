#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import sys
from sensor_msgs.msg import Image
from cv_bridge import *
import numpy as np
import cv2 as cv
from bodyhub.msg import JointControlPoint 
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg')+"/src")

from lejulib import *
from motion.example.action_example import Movement
import motion.bodyhub_client as bodycli
from digits_recognition import digits_recognition as digits_r
import sys
import os

WATCH_LEFT_ACTION = {"1":[[[2000,0],[0,0],[0,0]]],"2":[[[2000,0],[0,0],[0,0]]],"3":[[[2000,58],[0,0],[0,0]]],"4":[[[2000,-98],[0,0],[0,0]]],"5":[[[2000,-40],[0,0],[0,0]]],"6":[[[2000,0],[0,0],[0,0]]],"7":[[[2000,0],[0,0],[0,0]]],"8":[[[2000,0],[0,0],[0,0]]],"9":[[[2000,-58],[0,0],[0,0]]],"10":[[[2000,98],[0,0],[0,0]]],"11":[[[2000,40],[0,0],[0,0]]],"12":[[[2000,0],[0,0],[0,0]]],"13":[[[2000,0],[0,0],[0,0]]],"14":[[[2000,-49],[0,0],[0,0]]],"15":[[[2000,-24],[0,0],[0,0]]],"16":[[[2000,0],[0,0],[0,0]]],"17":[[[2000,49],[0,0],[0,0]]],"18":[[[2000,24],[0,0],[0,0]]],"19":[[[2000,0],[0,0],[0,0]]],"20":[[[2000,0],[0,0],[0,0]]],"21":[[[2000,13.4],[0,0],[0,0]]],"22":[[[2000,20.6],[0,0],[0,0]]]}

HEAD_IMAGE_TOPIC = '/camera/color/image_raw'
ROSNODE_NAME = 'kick_button'
HEAD_SERVO_TOPIC = 'MediumSize/BodyHub/HeadPosition'
SEARCH_LEFT = [15, 25]
SEARCH_RIGHT = [15, -35]
CONTROL_ID = 2
LEFT_MODEL_IMAGE_PATH = "/home/lemon/robot_ros_application/catkin_ws/src/ros_higher_vocational_training_platform/scripts/digits_model/left_model.png"
RIGHT_MODEL_IMAGE_PATH = "/home/lemon/robot_ros_application/catkin_ws/src/ros_higher_vocational_training_platform/scripts/digits_model/right_model.png"
WAIT_ACTION_DONE = 0.5

digit_match = {1:'one', 2:'two'}
current_working_directory = os.getcwd()
execution_file_path = sys.argv[0]
execution_file_dir_path = os.path.dirname(os.path.join(current_working_directory, execution_file_path))
left_button_kick_action = execution_file_dir_path + "/left_button_kick.py"
right_button_kick_action = execution_file_dir_path + "/right_button_kick.py"
kick_actions = {"right": right_button_kick_action, "left": left_button_kick_action}

class kick_button():
    def __init__(self):
        rospy.init_node(ROSNODE_NAME, anonymous=True)
        rospy.on_shutdown(self.rosShutdownHook)
        self.sub = rospy.Subscriber(HEAD_IMAGE_TOPIC, Image, self.image_callback)
        self.__cv_bridge = CvBridge()
        self.__img_origin = np.zeros((640, 480, 3), np.uint8)
        self.bodyhub = bodycli.BodyhubClient(CONTROL_ID)
        self.movement = Movement(self.bodyhub)
        self.debug = False

    def image_callback(self, msg):
        try:
            self.__img_origin = self.__cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as err:
            rospy.logerr(err)

    def set_head_servo(self, angles):
        keyframes = [
            ([0, -1, 16, -34, -17, -1, 0, 1, -16, 34, 17, 1, 0, -70, -15, 0, 70, 15, 0, 0, angles[1], angles[0]], 500, 0)
        ]
        self.movement.linearMove(keyframes)

    def load_model_image(self):
        return cv.imread(LEFT_MODEL_IMAGE_PATH)

    def frame_action(self, key_frame):
        try:
            client_action.custom_action([], key_frame)
        except Exception as err:
            serror(err)
        finally:
            pass

    def main(self, choose_digit):
        self.bodyhub.ready()
        self.frame_action(WATCH_LEFT_ACTION)
        # self.set_head_servo(SEARCH_LEFT)
        model_image = self.load_model_image()
        rospy.sleep(WAIT_ACTION_DONE)
        left_button_image = self.__img_origin
        recognize_digit = digits_r(left_button_image, model_image, debug=self.debug)
        recognize_left_result = recognize_digit.main()
        rospy.loginfo(digit_match[recognize_left_result])

        # self.set_head_servo(SEARCH_RIGHT)

        if recognize_left_result == choose_digit:
            choose_button = 'left'
        else:
            choose_button = 'right'
        os.system("python %s" % kick_actions[choose_button])

    def rosShutdownHook(self):
        self.bodyhub.reset()

if __name__ == "__main__":
    kb = kick_button()
    if len(sys.argv) > 1 and sys.argv[1] == 'debug':
        kb.debug = True
    choose_digit = int(input("请输入要敲击的按键编号(1或者2):"))
    if choose_digit in digit_match == False:
        rospy.logwarn("输入的按键编号有误")
    kb.main(choose_digit)
