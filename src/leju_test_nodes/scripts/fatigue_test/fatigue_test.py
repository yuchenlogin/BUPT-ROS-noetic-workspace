#!/usr/bin/env python
# -*- coding: utf-8 -*-

import subprocess
import sys
import os
import math
import termios
import threading
import time
import numpy as np
import rospy
import rospkg
import rosnode
import yaml
from std_msgs.msg import *
from geometry_msgs.msg import *
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
sys.path.append(rospkg.RosPack().get_path('leju_test_nodes')+"/scripts")
import tty
import select
from keyboardlinstener import keyboardlinstener
from algorithm import pidAlgorithm as pidAlg
from Moving_Node import Slam_Moving_Node
from logger import Logger,TopicsRecorder
from std_msgs.msg import *
from lejulib import terminate, node_initial
from bodyhub.srv import SrvServoScan
from wx_bot import WX_Bot

SCRIPTS_PATH=os.path.split(sys.argv[0])[0]

with open(os.path.join(SCRIPTS_PATH,"config.cfg"),"r")as f:
    config = yaml.load(f)

WX_BOT_KEY = config["WX_BOT_KEY"]# 企业微信机器人的key
NOTICE_LIST = config["NOTICE_LIST"] # 要@的人的手机号
MAX_POINTS = config["MAX_POINTS"] # 标记的点数
SLAM_LOST_TIMEOUT = config["SLAM_LOST_TIMEOUT"] # SLAM定位丢失后n秒进行提醒
TOPIC_RECORD_DURATION = config["TOPIC_RECORD_DURATION"] #录制topic的分包时间


TEST_DATA = time.strftime("%Y-%m-%d-%H.%M.%S", time.localtime(time.time()))
ROOT_PATH = "/home/lemon/fatigue_test/"
map_path = ROOT_PATH + "map/"
map_path_name = map_path + "testmap.bin"
config_file=os.path.join(SCRIPTS_PATH,"slam_path_tracking.yaml")
os.system("mkdir -p {}".format(map_path))

Test_folder = "{}{}/".format(ROOT_PATH,TEST_DATA)
NODE_NAME = 'path_tracking_node'
CONTROL_ID = 2


class Tester(Slam_Moving_Node):
    def __init__(self):
        super(Tester, self).__init__('Slam_Tester', 2)
        self.notice_bot = WX_Bot(WX_BOT_KEY)
        self.notice_list = NOTICE_LIST
        self.slam_process = None
        self.keyboard = keyboardlinstener()
        SLAM_POINT = None
        if os.path.exists(config_file):
            with open(config_file,"r")as f:
                SLAM_POINT=yaml.load(f)
        self.slam_points = SLAM_POINT if SLAM_POINT is not None else {}
        self.slam_log_file = map_path+"{}{}.log".format(TEST_DATA,"maping")
        self.rosbag_path = Test_folder+TEST_DATA+".bag"

    def start_init(self):
        self.logger = Logger(Test_folder)
        self.recorder = TopicsRecorder(os.path.join(SCRIPTS_PATH,"record_topics.yaml"),duration=TOPIC_RECORD_DURATION,output=self.rosbag_path)
        self.recorder.start()
        self.ikstate_sub = rospy.Subscriber('/IK/state', Int8, self.IkFail_callback)
        self.logger.write("OPEN @" + time.strftime("%Y-%m-%d %H:%M:%S:\n", time.localtime(time.time())),False)
        self.notice_bot.send_message("开始老化测试: {}".format(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))),self.notice_list)

    def location_lost(self):
        self.logger.write("[warning]location not updated! ")
        self.logger.save_current_image("location_lost")
        if time.time() - self.no_locate_time > SLAM_LOST_TIMEOUT:
            self.no_locate_time = time.time()
            self.notice_bot.send_message("SLAM定位丢失,请检查!",self.notice_list)

    def IkFail_callback(self,msg):
        print(msg.data)
        msg = "机器人逆解失败: {}, 请检查！\n录制的topic保存于:{}\n".format(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())),self.rosbag_path)
        self.logger.write(msg)
        self.notice_bot.send_message(msg,self.notice_list)
        self.stop()
        self.ikstate_sub.unregister()
        rospy.signal_shutdown('IkFail_callback')

    def slam_start(self,preview_mode=True,location_mode=False):
        os.system("ps -aux | grep RGBD | awk '{{print $2}}' | xargs kill")
        self.slam_log_file = map_path+"{}{}.log".format(TEST_DATA,"locating" if location_mode else "maping")
        if location_mode and not os.path.exists(map_path_name):
            rospy.logwarn("找不到地图{},请先建图!".format(map_path_name))
            rospy.signal_shutdown("error")
        self.slam_process = subprocess.Popen("export DISPLAY=:0.0;source ~/robot_ros_application/catkin_ws/devel/setup.bash;rosrun SLAM RGBD {} {} {} | tee {}".format(str(preview_mode).lower(), str(location_mode).lower(), map_path_name, self.slam_log_file), shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        starttime = time.time()
        print("SLAM正在启动,请等候...")
        while not rospy.core.is_shutdown_requested() and self.slam_process.poll() is None:
            line = self.slam_process.stdout.readline().decode()
            if time.time() - starttime>30 or "fail"in line.lower():
                rospy.logwarn("slam启动失败")
                rospy.signal_shutdown("error")
                break
            elif "ORB Extractor" in line:
                print("SLAM已启动!")
                break

    def slam_stop(self):
        os.system("rosnode kill /RGBD")
        starttime=time.time()
        while self.slam_process is not None and self.slam_process.poll() is None:
            if time.time() - starttime>5:
                print("force kill slam!")
                os.system("ps -aux | grep RGBD | awk '{{print $2}}' | xargs kill")
                break

    def stop(self):
        if hasattr(self,"recorder"):
            self.recorder.stop()
        self.slam_stop()
        
    def start(self):
        self.start_init()
        self.slam_start(True,True)
        start_time = time.time()
        count = 0
        try:
            while not rospy.core.is_shutdown_requested():
                cur_st = time.time()
                self.circle_run()
                self.scan_servo()
                count += 1
                total_time = time.strftime("%H:%M:%S", time.gmtime(time.time() - start_time))
                current_time = time.strftime("%H:%M:%S", time.gmtime(time.time() - cur_st))
                log = "完成第 {} 轮测试,用时 {} , 累计用时 {} \n".format(count,current_time,total_time)
                self.notice_bot.send_message(log,self.notice_list)
                self.logger.write(log+"{}\n\n".format("-- -- "*15))
                time.sleep(1)
        except Exception as e:
            print(e)
            msg = "异常退出,原因:{}\n".format(e)
            self.logger.write(msg)
            self.notice_bot.send_message(msg,self.notice_list)
        finally:
            msg = "测试结束, 共完成 {} 轮测试, 累计用时 {}\n".format(count,time.strftime("%H:%M:%S", time.gmtime(time.time() - start_time)))
            log_msg = "\n日志保存路径:\n{}\nslam运行日志:\n{}\n".format(Test_folder,self.slam_log_file)
            msg += log_msg
            self.logger.write(msg)
            self.logger.stop()
            self.logger.join()
            self.notice_bot.send_message(msg,self.notice_list)
        rospy.signal_shutdown('exit')
    
    def scan_servo(self):
        try:
            srv_name = '/MediumSize/BodyHub/ScanServo'
            rospy.wait_for_service(srv_name)
            ask_srv = rospy.ServiceProxy(srv_name, SrvServoScan)
            response_servo = list(ask_srv("all").getData)
        except Exception as e:
            response_servo = [e]
        success_servo = range(1,23)+[111,112,200]
        if response_servo != success_servo:
            msg = "[error]扫描舵机异常,扫描到的id:{}\n".format(response_servo)
            self.logger.write(msg)
            self.notice_bot.send_message(msg+"请检查",self.notice_list)
        return response_servo==success_servo

    def circle_run(self):
        self.bodyhub_walk()
        points = []
        for k in self.slam_points:
            points.append(self.slam_points[k])
        self.path_tracking(points,ignore_angle = True, pos_wait_mode=1,wait_time=0.5,pos_exit_high_acc=False)
        self.path_tracking([self.slam_points[0]],ignore_angle = False, pos_wait_mode=1,wait_time=0.5,pos_exit_high_acc=False)# 最后一个点
        self.bodyhub_ready()

    def debug(self):
        self.slam_start(True,True)
        n = 1
        tips = "\n(按'q'退出,按数字键标记第n个点,按'c'键清除所有标记,按's'键马上保存配置文件)"
        print("\n{}\n开始标定slam位置\n请等待slam启动获取到坐标后,移动到第{}个点按下'f'键:{}".format("--"*10,n,tips))
        while not rospy.core.is_shutdown_requested() and  self.slam_process.poll() is None:
            cur_pos = list(self.get_pos())
            sys.stdout.write("\r{}".format(cur_pos))
            sys.stdout.flush()
            k = self.keyboard.getKey(0.1).lower()
            if k == "f":
                if cur_pos[1]:
                    self.slam_points[n-1]=cur_pos[0]
                    n+=1
                    if n > MAX_POINTS:
                        self.write_config()
                        break
                    print("\n\n请移动到第{}个点按下f键:{}".format(n,tips))
                else:
                    print("\n标记失败, 位置未更新\n请移动到第{}个点按下f键:{}".format(n,tips))
            elif k == "q":
                break
            elif k == "s":
                self.write_config()
            elif k == "c":
                print("已清除所有标记点")
                self.slam_points= {}
            elif k in "123456789":
                print("标记点{}".format(k))
                if cur_pos[1]:
                    self.slam_points[int(k)-1]=cur_pos[0]
                    self.write_config()
                else:
                    print("\n标记失败, 位置未更新\n请移动到第{}个点按下f键:(按q退出)".format(n))
            

    def write_config(self):
        with open(os.path.join(SCRIPTS_PATH,"slam_path_tracking.yaml"),"w")as f:
            yaml.dump(self.slam_points,f,encoding="utf-8")
        print("\n写入配置文件成功")

    def mapping(self):
        print("开始建图...")
        self.slam_start(True,False)
        while not rospy.core.is_shutdown_requested() and  self.slam_process.poll() is None:
            print(self.get_pos())
            time.sleep(0.1)
        print("建图结束")
        

if __name__ == '__main__':
    node_initial(name = "Slam_Tester")
    rospy.Subscriber('terminate_current_process', String, terminate)
    t = Tester()
    if len(sys.argv) > 1:
        if sys.argv[1]=="mapping":
            t.mapping()
        elif sys.argv[1] == "debug":
            t.debug()
    else:
        t.start()
    t.stop()
