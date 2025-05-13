#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import math
import time
import os
import rospy
import rospkg
import yaml
from std_msgs.msg import *
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
sys.path.append(rospkg.RosPack().get_path('leju_test_nodes')+"/scripts")

from lejulib import terminate,node_initial
import motion.bodyhub_client as bodycli
from multiARtag_detecter import MultiMarkerLocation
from keyboardlinstener import keyboardlinstener
from logger import Logger
from wx_bot import WX_Bot
from healthy_checker import HealthChecker
SCRIPTS_PATH = os.path.split(sys.argv[0])[0]
TEST_DATA = time.strftime("%Y-%m-%d-%H.%M.%S", time.localtime(time.time()))
ROOT_PATH = "/home/lemon/fatigue_test/walking_test/"
Test_folder = "{}{}/".format(ROOT_PATH,TEST_DATA)

WALK_SUCC = 0
with open(os.path.join(SCRIPTS_PATH,"config.yaml"),"r")as f:
    CONFIG = yaml.safe_load(f)
POINTS_START = CONFIG["POINTS_START"]
POINTS_END = CONFIG["POINTS_END"]
FAILURE_DISTANCE = CONFIG["FAILURE_DISTANCE"]
WX_BOT_KEY = CONFIG["WX_BOT_KEY"]
WX_NOTICE_LIST = CONFIG["WX_NOTICE_LIST"]
TEMPERATURE_LIMIT = CONFIG["temperature_protection_value"]
class Walker(bodycli.BodyhubClient,MultiMarkerLocation):
    def __init__(self,debug = False):
        super(Walker, self).__init__(id = 2)
        MultiMarkerLocation.__init__(self,self,debug)
        rospy.Subscriber('terminate_current_process', String, terminate)
        self.__step_len_max = [0.1, 0.05, 10.0]  # x, y ,theta
        self.notise_bot = WX_Bot(WX_BOT_KEY)
        self.healthy_checker = HealthChecker()
        # 更新地图xml文件
        self.generate_map(CONFIG["MAP_POINTS"],path =CONFIG["Map_xml_path"],marker_len=CONFIG["Marker_size"])
        self.config = CONFIG
        self.status = "None"
        
    def walk_s_route(self):
        self.walk()
        if self.walking_n_steps([0.0, 0.0, 10.0],6) != WALK_SUCC:
            return self.failure_active("s曲线行走异常")
        if self.walking_n_steps([0.08, 0.0, -8],15) != WALK_SUCC:
            return self.failure_active("s曲线行走异常")
        if self.walking_n_steps([0.08, 0.0, 8],15) != WALK_SUCC:
            return self.failure_active("s曲线行走异常")
        if self.walking_n_steps([0.0, 0.0, -10.0],6) != WALK_SUCC:
            return self.failure_active("s曲线行走异常")
        self.wait_walking_done()
        return 0
        
    def walk_straight_line(self,length = 2, steplen = 0.1):
        self.walk()
        stepcount = int(length/steplen)
        if self.walking_n_steps([steplen, 0.0, 0.0],stepcount) != WALK_SUCC:
            return self.failure_active("直线行走异常")
        self.wait_walking_done()
        return 0
        
    def notise_and_log(self,msg,print_color=37):
        self.logger.write(msg+"\n",print_color = print_color)
        self.notise_bot.send_message(msg,WX_NOTICE_LIST)

    def calculate_distance(self,taget_pos,timeout = 10):
        stime = time.time()
        while not rospy.is_shutdown():
            cur_pos = self.get_current_pos()
            if cur_pos and cur_pos["valid"]:
                distance = math.sqrt((cur_pos["pos"][0] - taget_pos[0])**2+(cur_pos["pos"][1] - taget_pos[1])**2)
                self.logger.write("当前位置:{},距离目标点距离:{}\n".format([cur_pos["pos"][0]**2,cur_pos["pos"][1]**2],distance))
                if distance > FAILURE_DISTANCE:
                    self.failure_active("位置过远,测试结束!")
                    return 1
                else:
                    self.logger.write("行走正常...\n")
                    return 0
            else:
                self.logger.write("无法获取到当前位置!\n")
                if time.time() - stime > timeout:
                    self.failure_active("[error]超时无法获取到当前位置,测试中止")
                    return -1
    
    def failure_active(self,reason):
        self.notise_and_log(reason,print_color = 31)
        rospy.signal_shutdown(str(reason))
        return 1
    
    def healthy_check(self):
        self.logger.write("正在检查摄像头...\n")
        check_result = self.healthy_checker.check_camera()
        if check_result:
            return self.failure_active("摄像头异常!")
        self.logger.write("摄像头正常\n")
        
        self.logger.write("正在检查舵机...\n")
        check_result = self.healthy_checker.check_servo_id()
        if check_result:
            return self.failure_active("舵机异常,异常id为{}".format(check_result))
        self.logger.write("舵机id正常\n")
        
        self.logger.write("正在检查舵机温度...\n")
        check_result = self.healthy_checker.check_servo_temperature(TEMPERATURE_LIMIT)
        if check_result:
            return self.failure_active("舵机温度异常,异常温度和id为{}".format(check_result))
        self.logger.write("舵机温度正常\n")
        
        self.logger.write("正在检查脚底压感...\n")
        check_result = self.healthy_checker.check_fsr()
        if check_result:
            if sum(check_result[0]) == 0 or sum(check_result[1]) == 0:
                return self.failure_active("脚底压感全部异常,{},终止测试!".format(check_result))
            self.notise_and_log("脚底压感异常,{}".format(check_result),print_color = 31)
        self.logger.write("脚底压感正常\n")
        
        self.logger.write("正在检查麦克风连接状态...\n")
        check_result = self.healthy_checker.check_mic()
        if check_result:
            self.notise_and_log("麦克风连接异常",print_color = 31)
        self.logger.write("麦克风正常\n")
        return 0
        
    def test_one_round(self):
        self.walk()
        
        # 直线行走测试
        self.logger.write("正在进行直线行走测试...\n")
        self.logger.write("移动到起点...\n")
        self.goto_multi_pose(POINTS_START)
        self.logger.write("开始行走(去程)...\n")
        if self.walk_straight_line(length=2):
            return self.failure_active("直线行走去程失败,请检查...")
        self.logger.write("计算当前位置中..\n")
        if self.calculate_distance(POINTS_END):
            return 1
        
        self.logger.write("移动到终点...\n")
        self.goto_multi_pose(POINTS_END)
        self.logger.write("开始直线行走(回程)...\n")
        if self.walk_straight_line(length=2):
            return self.failure_active("直线行走回程失败,请检查...")
        self.logger.write("计算当前位置中..\n")
        if self.calculate_distance(POINTS_START):
            return 1
        
        # 机器人状态检查
        if self.healthy_check():
            return 1

        # s曲线行走测试
        self.logger.write("正在进行曲线行走测试...\n")
        self.logger.write("移动到起点...\n")
        self.goto_multi_pose(POINTS_START)
        self.logger.write("开始s曲线行走(去程)...\n")
        if self.walk_s_route():
            return self.failure_active("s曲线行走去程失败,请检查...")
        self.logger.write("计算当前位置中..\n")
        if self.calculate_distance(POINTS_END):
            return 1
        
        self.logger.write("移动到终点...\n")
        self.goto_multi_pose(POINTS_END)
        self.logger.write("开始s曲线行走(回程)...\n")
        if self.walk_s_route():
            return self.failure_active("s曲线行走回程失败,请检查...")
        self.logger.write("计算当前位置中..\n")
        if self.calculate_distance(POINTS_START):
            return 1
        
        # 机器人状态检查
        if self.healthy_check():
            return 1
        
        return 0
                
    def start(self):
        self.logger = Logger(Test_folder)
        self.notise_and_log("开始行走测试...\n",print_color=32)
        try:
            self.multiMarkerstart()# 打开artag 识别
            start_time = time.time()
            count = 0
            while not rospy.core.is_shutdown_requested():
                cur_st = time.time()
                if self.test_one_round():
                    break
                count += 1
                total_time = time.strftime("%H:%M:%S", time.gmtime(time.time() - start_time))
                current_time = time.strftime("%H:%M:%S", time.gmtime(time.time() - cur_st))
                log = "完成第 {} 轮测试,用时 {} , 累计用时 {} \n".format(count,current_time,total_time)
                self.notise_and_log(log,print_color=32)
            self.reset()
            self.stopMultiMarker(timeout=5)
        except Exception as e:
            self.notise_and_log("测试异常退出,原因{}".format(e),print_color=31)
        finally:
            msg = "测试结束, 共完成 {} 轮测试, 累计用时 {}\n".format(count,time.strftime("%H:%M:%S", time.gmtime(time.time() - start_time)))
            log_msg = "\n日志保存路径:\n{}\n".format(Test_folder)
            msg += log_msg
            self.notise_and_log(msg,32)
            rospy.signal_shutdown('exit')
            self.logger.stop()
        
    def debug(self):
        print("正在启动ar_track_alvar...")
        self.multiMarkerstart()
        self.walk()
        keyboard = keyboardlinstener()
        c = 0
        tips = "(按F键->顺序标定,数字1->标定起始位置,数字2->结束位置,q->退出):"
        print("\n开始标定artag位置,请将机器人移动到开始位置,朝向结束位置:\n{}".format(tips))
        while not rospy.is_shutdown():
            posdata = self.get_multi_marker_pos()
            k = keyboard.getKey(0.1)
            if k.lower() == "q":
                print("退出")
                break
            pos,rot = posdata["pos"],posdata["rot"]
            cur_pos = [pos[0],pos[1],rot[2]]
            sys.stdout.write("\r\033[{}mx:{:8.5f} y:{:8.5f} a:{:8.5f},valid:{}   \033[0m".format(32 if posdata["valid"] else 31,pos[0],pos[1],rot[2], posdata["valid"]))
            sys.stdout.flush()
            if posdata is None or not posdata["valid"]:
                continue
            
            if k == "1":
                self.config["POINTS_START"] = cur_pos
                print("\n标定坐标{}".format(cur_pos))
            elif k == "2":
                self.config["POINTS_END"] = cur_pos
                print("\n标定坐标{}".format(cur_pos))
            elif k == "f":
                self.config["POINTS_START" if c==0 else "POINTS_END"] = cur_pos
                c += 1
                print("\n标定坐标{}".format(cur_pos))
                self.write_config()
                if c >=2:
                    print('校准完成，程序退出')
                    break
                print("\n请移动到结束位置,朝向开始位置:{}\n".format(tips))
        self.reset()
        self.stopMultiMarker(5)
        rospy.signal_shutdown('exit')
        
    def write_config(self):
        with open(os.path.join(SCRIPTS_PATH,"config.yaml"),"w")as f:
            yaml.dump(self.config,f,encoding="utf-8")
        print("\n写入配置文件成功")
        
    def test(self):
        while not rospy.is_shutdown():
            print(self.calculate_distance(POINTS_END))
    def shutdown_hook(self):
        self.reset()
        self.stopMultiMarker(5)

if __name__ == '__main__':
    rospy.init_node("walking_test_node", anonymous=False)
    w = Walker(debug=False)
    rospy.on_shutdown(w.shutdown_hook)
    # w.multiMarkerstart()
    if len(sys.argv) > 1:
        w.debug()
    else:
        w.start()
