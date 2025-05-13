#!/usr/bin/env python3
# coding=utf-8
import rospy
from std_msgs.msg import String, Empty
from ros_AIUI_node.srv import textToSpeakMultipleOptions
import os
import time
import threading
import json
import sys, rospkg
from head_control import *
sys.path.append(os.path.join(rospkg.RosPack().get_path('leju_test_nodes'), 'scripts'))
from healthy_checker import HealthChecker
# 添加 leju_lib_pkg 环境变量
sys.path.append(os.path.join(rospkg.RosPack().get_path('leju_lib_pkg'), 'src'))
from motion import bodyhub_client as bodycli
from lejulib import finishsend
from lejufunc.bezier import send_custom_bezier 
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))

FALL_DETECTION_CONFIDENCE_SERVER = '/fall_detection_node/fall_result'
TEXT_TO_SPEAK_MULTI_OPTIONS_SERVER = '/aiui/text_to_speak_multiple_options'
AIUI_PLAY_END = "/aiui/play_end"
FALL_DETECTION_CONFIG_RELATIVE_PATH = "../configs/fall_detection_config.json"

def relative_path_to_absolute_path(relative_path):
    abs_path = os.path.abspath(os.path.join(os.path.dirname(__file__), relative_path))
    return abs_path

class FallDetectionSubscriber:
    def __init__(self):
        rospy.init_node('fall_detection_node', anonymous=True)
        self.subscriber = rospy.Subscriber(FALL_DETECTION_CONFIDENCE_SERVER, String, self.callback)
        self.result = 'None'
        self.timeout = 1.0
        self.rate = rospy.Rate(1)
        self.start_time = None
        self.timer = None  # 定时器变量
        self.tts_client = None
        self.is_playing = False
        self.bodyhub = bodycli.BodyhubClient(2)
        self.fall_detection_config = self.load_config(relative_path_to_absolute_path(FALL_DETECTION_CONFIG_RELATIVE_PATH))
        self.healthchecker = HealthChecker()
        self.is_fallen = False
        self.stop_rotation = False

    def load_config(self, file_path):
        return json.loads(open(file_path).read())

    def callback(self, msg):
        if msg.data != 'None' and msg.data != self.result:
            self.start_time = rospy.Time.now()
        self.result = msg.data
        
        if msg.data is None:
            self.is_fallen = False
        elif msg.data == 'fall':
            self.is_fallen = True
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

    def play_sound(self, sound_path):
        os.system('aplay {}'.format(sound_path))

    def play_sound_periodically(self, sound_path, interval):
        # 播放提示音函数
        self.play_sound(sound_path)

        # 设置下一次播放的定时器
        self.timer = threading.Timer(interval, self.play_sound_periodically, args=(sound_path, interval))
        self.timer.start()
        
    def rotate_head(self):
        # self.stop_rotation = False
        self.bodyhub.ready()
        self.is_fallen = False
        current_frame_index = 0
        fallen_count = 0  # 记录连续检测到摔倒的次数
        while not rospy.is_shutdown():
            print("Is fallen:", self.is_fallen)  # 调试语句
            print(self.result)
            
            if not self.is_fallen:
                if fallen_count == 0:
                    if current_frame_index == 0:
                        send_custom_bezier(act_frames=head_control_right30_frames)
                    elif current_frame_index == 1:
                        send_custom_bezier(act_frames=head_control_right60_frames)
                    elif current_frame_index == 2:
                        send_custom_bezier(act_frames=head_control_30right_frames)
                    elif current_frame_index == 3:
                        send_custom_bezier(act_frames=head_control_right0_frames)
                    elif current_frame_index == 4:
                        send_custom_bezier(act_frames=head_control_left30_frames)
                    elif current_frame_index == 5:
                        send_custom_bezier(act_frames=head_control_left60_frames)
                    elif current_frame_index == 6:
                        send_custom_bezier(act_frames=head_control_30left_frames)
                    elif current_frame_index == 7:
                        send_custom_bezier(act_frames=head_control_left0_frames)
                        
                    current_frame_index = (current_frame_index + 1) % 8  # 更新当前帧索引，循环播放动作序列
                else:
                    fallen_count = 0  # 重置连续摔倒的计数

            time.sleep(5)
            
            if self.is_fallen:
                fallen_count += 1
                # 根据条件选择是否停留在当前位置
                if fallen_count >= 2:
                    self.is_fallen = False
                    self.text_to_speak_multi_options(**self.fall_detection_config["fall"])
                    rospy.sleep(5)  # 等待5秒
                    continue
            else:
                fallen_count = 0  # 如果没有检测到摔倒，则将计数器重置为0，从前一个位置继续循环


if __name__ == '__main__':
    try:
        node = FallDetectionSubscriber()
        rospy.on_shutdown(finishsend)
        node.text_to_speak_multi_options(**node.fall_detection_config["check"])
        rospy.sleep(5)  # 等待5秒，让头部开始旋转
        node.rotate_head()  # 启动头部旋转
    except rospy.ROSInterruptException:
        pass