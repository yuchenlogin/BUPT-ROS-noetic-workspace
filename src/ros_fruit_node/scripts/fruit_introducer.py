#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Empty
from ros_AIUI_node.srv import textToSpeakMultipleOptions
import os
import json
import sys, rospkg
sys.path.append(os.path.join(rospkg.RosPack().get_path('leju_test_nodes'), 'scripts'))
from healthy_checker import HealthChecker

FRUIT_RECOGNIZER_SET_CONFIDENCE_SERVER = '/fruit_recognizer_node/fruit_result'
TEXT_TO_SPEAK_MULTI_OPTIONS_SERVER = '/aiui/text_to_speak_multiple_options'
AIUI_PLAY_END = "/aiui/play_end"
FRUITS_INTRODUCER_CONFIG_RELATIVE_PATH = "../configs/fruits_introducer_config.json"
NO_IMAGE = -1
NO_IMAGE_MESSAGE = '摄像头异常，无法准确识别，请排查'

def relative_path_to_absolute_path(relative_path):
    abs_path = os.path.abspath(os.path.join(os.path.dirname(__file__), relative_path))
    return abs_path

class FruitResultSubscriber:
    def __init__(self):
        rospy.init_node('fruit_introducer', anonymous=True)
        self.subscriber = rospy.Subscriber(FRUIT_RECOGNIZER_SET_CONFIDENCE_SERVER, String, self.callback)
        self.result = 'None'
        self.timeout = 1.0
        self.rate = rospy.Rate(10)
        self.start_time = None
        self.tts_client = None
        self.is_play_end = True
        self.sub_aiui_play_end = rospy.Subscriber(AIUI_PLAY_END, Empty, self.sub_aiui_play_end_cb)
        self.fruits_introducer_config = self.load_config(relative_path_to_absolute_path(FRUITS_INTRODUCER_CONFIG_RELATIVE_PATH))
        self.healthchecker = HealthChecker()

    def load_config(self, file_path):
        return json.loads(open(file_path).read())

    def sub_aiui_play_end_cb(self, msg):
        self.is_play_end = True

    def callback(self, msg):
        if not self.is_play_end:
            self.result = 'None'
            return

        if msg.data != 'None' and msg.data != self.result:
            self.start_time = rospy.Time.now()
        self.result = msg.data

    def text_to_speak_multi_options(self, text, vcn='qige', speed=50, pitch=5, volume=20):
        try:
            rospy.wait_for_service(TEXT_TO_SPEAK_MULTI_OPTIONS_SERVER)
        except rospy.ROSException:
            rospy.logerr('wait for {} timeout!'.format(TEXT_TO_SPEAK_MULTI_OPTIONS_SERVER))
            return
        if self.tts_client is None:
            self.tts_client = rospy.ServiceProxy(TEXT_TO_SPEAK_MULTI_OPTIONS_SERVER, textToSpeakMultipleOptions)
        self.tts_client(text, vcn, speed, pitch, volume)

    def run(self):
        if self.healthchecker.check_camera() == NO_IMAGE:
            self.text_to_speak_multi_options(NO_IMAGE_MESSAGE)
            return
        while not rospy.is_shutdown():
            if self.result != 'None':
                if (rospy.Time.now() - self.start_time).to_sec() >= self.timeout:
                    rospy.loginfo('识别到水果：{}'.format(self.result))
                    self.is_play_end = False
                    self.text_to_speak_multi_options(**self.fruits_introducer_config[self.result])
                    self.result = 'None'
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = FruitResultSubscriber()
        node.run()
    except rospy.ROSInterruptException:
        pass
