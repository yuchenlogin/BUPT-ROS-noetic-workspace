#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import time
import sys
sys.path.append('/home/lemon/robot_ros_application/catkin_ws/src/keyboards/scripts')
from play_tts_file import Player
from bodyhub.srv import SrvServoScan
from lejulib import *

SCAN_TORSO = 'torso'
SCAN_FSR = 'fsr'
SCAN_BASEBOARD = 'baseboard'
SCAN_ALL = 'all'

class ServoScan(Player):

    def __init__(self):
        Player.__init__(self)
        self.voice_file_path = '/home/lemon/robot_ros_application/catkin_ws/src/keyboards/voice/'
        self.srv_name = '/MediumSize/BodyHub/ScanServo'
        rospy.wait_for_service(self.srv_name)
        self.ask_srv = rospy.ServiceProxy(self.srv_name, SrvServoScan)
        self.correct_id_num = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 111, 112, 200]
        self.scan_results = {"torso":[], "fsr":[], "baseboard":[], "all":[]}
        self.miss_ids = [1] * len(self.correct_id_num)

    def __torso_scan(self):
        response_torso = self.ask_srv(SCAN_TORSO)
        self.scan_results['torso'] = list(response_torso.getData)

    def __fsr_scan(self):
        response_fsr = self.ask_srv(SCAN_FSR)
        self.scan_results['fsr'] = list(response_fsr.getData)
    
    def __baseboard_scan(self):
        response_baseboard = self.ask_srv(SCAN_BASEBOARD)
        self.scan_results['baseboard'] = list(response_baseboard.getData)

    def __all_scan(self):
        response_all = self.ask_srv(SCAN_ALL)
        self.scan_results['all'] = list(response_all.getData)

    def screen_id(self, scan_list_name='', scan_list=[]):
        if scan_list_name == SCAN_TORSO:
            standard_ids_list = self.correct_id_num[:22]
        elif scan_list_name == SCAN_FSR:
            standard_ids_list = self.correct_id_num[22:24]
        elif scan_list_name == SCAN_BASEBOARD:
            standard_ids_list = self.correct_id_num[-1]
        elif scan_list_name == SCAN_ALL:
            standard_ids_list = self.correct_id_num
        for index, correct_id in enumerate(standard_ids_list):
            for scan_list_id in scan_list:
                if scan_list_id == correct_id:
                    scan_list.remove(scan_list_id)
                    self.miss_ids[index] = 1
                    break
                else:
                    self.miss_ids[index] = 0

    def play_illegal_ids(self):
        if len(self.scan_results['torso']):
            self.play_complete_voice(self.voice_file_path, "recognized_servo")  # 未识别到不合法的躯干舵机编号为
            for illegal_id in self.scan_results['torso']:
                self.play_single_voice(self.voice_file_path, str(illegal_id))
        if len(self.scan_results['fsr']):
            self.play_complete_voice(self.voice_file_path, "recognized_servo")  # 未识别到不合法的脚底压感编号为
            for illegal_id in self.scan_results['fsr']:
                self.play_single_voice(self.voice_file_path, str(illegal_id))
        if len(self.scan_results['baseboard']):
            self.play_complete_voice(self.voice_file_path, "recognized_servo")  # 未识别到不合法的主板编号为
            for illegal_id in self.scan_results['baseboard']:
                self.play_single_voice(self.voice_file_path, str(illegal_id))

    def play_miss_ids(self):
        miss_id_flag = False
        if self.miss_ids[-1] == 0:
            miss_id_flag = True
            self.play_complete_voice(self.voice_file_path, 'miss_baseboard')
        if 0 in self.miss_ids[22:24]:
            miss_id_flag = True
            self.play_complete_voice(self.voice_file_path, 'miss_fsr')
            for index, miss_id in enumerate(self.miss_ids[22:24]):
                if miss_id == 0:
                    self.play_single_voice(self.voice_file_path, str(self.correct_id_num[22:24][index]))
                    time.sleep(0.5)
        if 0 in self.miss_ids[:22]:
            miss_id_flag = True
            self.play_complete_voice(self.voice_file_path, 'miss_torso')
            for index, miss_id in enumerate(self.miss_ids[:22]):
                if miss_id == 0:
                    self.play_single_voice(self.voice_file_path, str(self.correct_id_num[:22][index]))
                    time.sleep(0.5)
        if miss_id_flag == False:
            print("work")
            self.play_complete_voice(self.voice_file_path, 'pass_scan')

    def servo_2_scan(self):
        self.play_complete_voice(self.voice_file_path, 'scan_servo')
        self.__all_scan()
        self.screen_id(SCAN_ALL, self.scan_results['all'])
        self.play_miss_ids()

def rosShutdownHook():
    print("shutdown")
    os.system('ps -aux | grep play | awk \'{print $2}\' | xargs kill -9')
    finishsend()

if __name__ == '__main__':
    rospy.init_node("check_servo")
    rospy.on_shutdown(rosShutdownHook)
    rospy.Subscriber('terminate_current_process', String, terminate)
    scan = ServoScan()
    scan.servo_2_scan()
