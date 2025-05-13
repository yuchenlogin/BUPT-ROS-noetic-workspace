#!/usr/bin/python3
# -*- coding: utf-8 -*-

import json
import rospy
import rospkg
import os
import sys
import subprocess
import signal
from threading import Timer
from sensor_msgs.msg import ChannelFloat32
from bodyhub.srv import SrvInstWrite, SrvTLSstring
from ros_socket_node.srv import ModifyBtnsConfig
from play_tts_file import Player
sys.path.append(rospkg.RosPack().get_path('ros_AIUI_node')+"/scripts")
from  SocketClient import Socket_runner_client
ISEXIST = True
INVALID_VOICE_PATH = "/home/lemon/robot_ros_application/catkin_ws/src/keyboards/voice/invalid_demo.mp3"
BUTTON_VOICE_PATH = "/home/lemon/robot_ros_application/catkin_ws/src/keyboards/voice/button.mp3"
BACK_BUTTON_CONFIG_PATH = '/home/lemon/.lejuconfig/BackBtnsConfig.json'
SOURCE_PATH = 'source /home/lemon/robot_ros_application/catkin_ws/devel/setup.bash'
BUTTONREGISTERINFO = {'sensor_name': 'baseBoard', 'reg_addr': 24, 'data_len': 56}
BTNS_CONFIG_FILE_IS_UPDATE = True
NO_MATCHING_DEMO = 'illegal'

# this is for debugging
DEBUGGING_PHASE = True
DEFAULT_DEMO_PATH = r'/home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/'
EXECUTABLE_DEMO = {
    'detect_battery_level': DEFAULT_DEMO_PATH + '当前电量.py',
    'get_network_info': DEFAULT_DEMO_PATH + '网络状态.py',
    "check_servo": DEFAULT_DEMO_PATH + "舵机扫描.py"
}
DEBUGGING_PHASE_DEMO_LIST = [NO_MATCHING_DEMO,  EXECUTABLE_DEMO['detect_battery_level'], EXECUTABLE_DEMO['get_network_info'], EXECUTABLE_DEMO['check_servo'], NO_MATCHING_DEMO]
class BackButton():
    def __init__(self):
        self.demo_list = ['' for _ in range(5)]
        self.cnt = 0
        self.timer = None
        self.reset_button_cnt_delay = 0.4
        self.config_file_stamp_time = 0
        self.last_button = 0
        self.demo_script_is_running = False
        self.p = None
        self.buttonMap = {1:1, 2:2, 4:3, 8:4, 16:5}
        self.player=Player()
        self.client_runner=Socket_runner_client()
    
    def load_button_config(self):
        try:
            if os.path.exists(BACK_BUTTON_CONFIG_PATH) == ISEXIST:
                with open(BACK_BUTTON_CONFIG_PATH, 'r') as f:
                    back_button_config_data = json.load(f)
                for button, data in enumerate(back_button_config_data):
                    if os.path.exists(data['path']) == ISEXIST:
                        self.demo_list[button] = data['path']
                    else:
                        self.demo_list[button] = NO_MATCHING_DEMO
                return True
            else:
                rospy.logwarn("Missing config file for buttons!")
                return False
        except:
            rospy.logerr('load config file for buttons error!')

    def reset_button_count(self):
        self.cnt = 0
        self.timer.cancel()

    def check_config_update(self):
        btnsconfig_client = rospy.ServiceProxy('/ros_socket_node/modify_backbtnsconfig', ModifyBtnsConfig)
        response = btnsconfig_client('check_time_stamp')
        current_config_file_stamp_time = response.time
        if current_config_file_stamp_time == -1 or current_config_file_stamp_time == self.config_file_stamp_time:
            return False
        else:
            self.config_file_stamp_time = current_config_file_stamp_time
            return True

    def button_callback(self, msg):
        if msg.name == 'KeyStatus' and msg.values[0] != 0:
            if self.timer:
                self.timer.cancel()
            self.timer = Timer(self.reset_button_cnt_delay, self.reset_button_count)
            self.timer.start()
            self.cnt += 1
            if self.cnt == 2:
                # self.player.play(BUTTON_VOICE_PATH)
                if DEBUGGING_PHASE == False:
                    if self.check_config_update() == BTNS_CONFIG_FILE_IS_UPDATE:
                        self.load_button_config()
                if msg.values[0] == self.buttonMap[1]: return
                if msg.values[0] in self.buttonMap:
                    self.update_demo_state()
                    if self.demo_script_is_running == False:
                        if DEBUGGING_PHASE == True:
                            self.demo_list = DEBUGGING_PHASE_DEMO_LIST
                        if self.demo_list[self.buttonMap[msg.values[0]] - 1] != NO_MATCHING_DEMO:
                            self.run_demo(msg.values[0])
                        else:
                            self.player.play(INVALID_VOICE_PATH)
                            rospy.logwarn('There is an error in the button profile you have selected')
                    else:
                        if self.buttonMap[msg.values[0]] == self.last_button:
                            self.shutdown_demo(msg.values[0])
                        else:
                            rospy.loginfo('another demo is running!')

    def update_demo_state(self):
        self.demo_script_is_running=False if self.client_runner.get_execution_status()==False else True
        


    def run_demo(self, buttonValue):
        demo_path=self.demo_list[self.buttonMap[buttonValue] - 1]
        if not os.path.exists(demo_path):
            self.player.play(INVALID_VOICE_PATH)
            return
        self.client_runner.run(demo_path)
        self.demo_script_is_running = True
        self.last_button = self.buttonMap[buttonValue]

    def shutdown_demo(self, buttonValue):
        self.client_runner.stop()
        self.demo_script_is_running = False
        rospy.loginfo('shutdown button %d demo.' %self.buttonMap[buttonValue])

def regist_sensor(sensor_name, reg_addr, data_len):
    rospy.wait_for_service('/MediumSize/BodyHub/RegistSensor', 2)
    sensor_register = rospy.ServiceProxy('/MediumSize/BodyHub/RegistSensor', SrvInstWrite)
    sensor_register(sensor_name, reg_addr, data_len)

def delete_sensor(sensor_name):
    rospy.wait_for_service("/MediumSize/BodyHub/DeleteSensor", 2)
    sensor_deleter = rospy.ServiceProxy('/MediumSize/BodyHub/DeleteSensor', SrvTLSstring)
    sensor_deleter(sensor_name)

def shut_hook():
    delete_sensor(BUTTONREGISTERINFO['sensor_name'])
    rospy.loginfo('Delete sensor %s' % BUTTONREGISTERINFO['sensor_name'])

if __name__ == '__main__':
    rospy.init_node('keyboards_node')
    regist_sensor(BUTTONREGISTERINFO['sensor_name'], BUTTONREGISTERINFO['reg_addr'], BUTTONREGISTERINFO['data_len'])
    button = BackButton()
    rospy.on_shutdown(shut_hook)
    rospy.Subscriber('/MediumSize/SensorHub/sensor_CF1', ChannelFloat32, button.button_callback, queue_size=1)
    rospy.spin()
