#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, rospkg
import os, sys
sys.path.append(os.path.join(rospkg.RosPack().get_path("ros_socket_node"), "scripts"))
from nodes import Runner
import json
sys.path.append(os.path.join(rospkg.RosPack().get_path("leju_lib_pkg")))
import motion.motionControl as mtrcl
from bodyhub.srv import SrvServoScan, SrvPresentTemperature, SrvFSR
from ros_work_weixin_robot_node.srv import WorkWeixinTextSend
import subprocess
sys.path.append(os.path.join(rospkg.RosPack().get_path("leju_test_nodes"), "scripts"))
from logger import Logger
import time

CONFIG_FILE_PATH = "/home/lemon/robot_ros_application/catkin_ws/src/leju_test_nodes/scripts/action_test/config/config.json"
ABNORMAL_FSR_DATA = 0
SUCCESS = 0
ROOT_PATH = ROOT_PATH = "/home/lemon/fatigue_test/"
TEST_DATA = time.strftime("%Y-%m-%d-%H.%M.%S", time.localtime(time.time()))
TEST_LOG_FOLDER = "{}{}/".format(ROOT_PATH, TEST_DATA)

class ActionTest(object):
    def __init__(self):
        rospy.on_shutdown(self.rosShutdownHook)
        self.config = self.load_config()
        self.logger = Logger(TEST_LOG_FOLDER)

    def load_config(self):
        with open(CONFIG_FILE_PATH, "r") as f:
            rf = f.read()
        return json.loads(rf, encoding="utf-8")

    def servo_scan(self):
        try:
            rospy.wait_for_service("/MediumSize/BodyHub/ScanServo", timeout=2)
        except rospy.ROSException:
            rospy.logwarn("action test scan servo timeout!")
            return []
        servo_scan_client = rospy.ServiceProxy("/MediumSize/BodyHub/ScanServo", SrvServoScan)
        response = servo_scan_client("torso")
        return list(response.getData)

    def check_servo_id(self):
        current_servo_id_list = self.servo_scan()
        servo_lost_communication = list(set(self.config["servo_id_with_normal_communication"]).difference(set(current_servo_id_list)))
        return servo_lost_communication

    def get_servo_present_temperature(self):
        try:
            rospy.wait_for_service("/MediumSize/BodyHub/PresentTemperature", timeout=2)
        except rospy.ROSException:
            rospy.logwarn("action test get servo present temperature timeout!")
            return []
        present_temperature_client = rospy.ServiceProxy("/MediumSize/BodyHub/PresentTemperature", SrvPresentTemperature)
        response = present_temperature_client()
        return list(response.presentTemperature)

    def check_servo_temperature(self):
        over_temperature = {}
        current_servo_present_temperature = self.get_servo_present_temperature()
        for index, data in enumerate(current_servo_present_temperature):
            if data >= self.config["temperature_protection_value"]:
                over_temperature[index+1] = data
        return over_temperature

    def get_fsr(self):
        get_result = {"leftFSR": [], "rightFSR": []}
        try:
            rospy.wait_for_service("/MediumSize/BodyHub/FSR", timeout=2)
        except rospy.ROSException:
            rospy.logwarn("action test get fsr data timeout!")
            return get_result
        fsr_client = rospy.ServiceProxy("/MediumSize/BodyHub/FSR", SrvFSR)
        response = fsr_client()
        get_result["leftFSR"] = list(response.leftFSR)
        get_result["rightFSR"] = list(response.rightFSR)
        return get_result

    def send_test_process_and_check_abort(self, counter, single_round_execution_time, total_execution_time, servo_lost_communication, over_temperature, fsr_data, is_network_connected):
        if servo_lost_communication or over_temperature or self.is_fsr_data_abnormal(fsr_data) or is_network_connected == False:
            text_msg = "共执行了 {} 轮测试持续时间 {} 秒\n\n本轮测试时长 {} 秒，由于以下原因测试终止：".format(counter,\
                                total_execution_time,\
                                single_round_execution_time)
            if servo_lost_communication:
                text_msg = text_msg + "\n\n未扫描到的舵机 id:\n{}".format(" ".join(str(id) for id in servo_lost_communication))
            if over_temperature:
                text_msg = text_msg + "\n\n过温的舵机:\n{}".format("\n".join(["id {} 温度达到 {} ℃".format(item, over_temperature[item]) for item in over_temperature]))
            if self.is_fsr_data_abnormal(fsr_data):
                text_msg = text_msg + "\n\n脚底压感数值:\n左脚: {}\n右脚: {}".format(" ".join(str(data) for data in fsr_data["leftFSR"]),\
                                                                                    " ".join(str(data) for data in fsr_data["rightFSR"]))
            if is_network_connected == False:
                text_msg = text_msg + "\n\n网络失去连接"
            is_terminate = True
        else:
            text_msg = "已执行 {} 轮测试共 {} 秒\n\n本轮测试时长 {} 秒\n\n".format(counter,\
                                total_execution_time,\
                                single_round_execution_time)
            is_terminate = False
        self.work_weixin_robot_text_send(text_msg)
        return is_terminate, text_msg

    def work_weixin_robot_text_send(self, text_msg):
        try:
            rospy.wait_for_service("/work_weixin/text_send", timeout=2)
        except rospy.ROSException:
            rospy.logwarn("action test work weixin robot strike!")
            return
        text_send_client = rospy.ServiceProxy("/work_weixin/text_send", WorkWeixinTextSend)
        text_send_client(self.config["robotKey"], text_msg, self.config["phone"])

    def check_servo(self):
        over_temperature = {}
        servo_lost_communication = self.check_servo_id()
        if not servo_lost_communication:
            over_temperature = self.check_servo_temperature()
        fsr_data = self.get_fsr()
        return servo_lost_communication, over_temperature, fsr_data

    def is_fsr_data_abnormal(self, fsr_data):
        for data in fsr_data:
            if ABNORMAL_FSR_DATA in fsr_data[data]:
                return True
        return False

    def detect_network(self):
        if subprocess.call("timeout 2 ping www.baidu.com -c 1", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE) == SUCCESS:
            return True
        else:
            return False

    def stop_log(self):
        self.logger.stop()

    def loop_execute_action(self):
        counter = 0
        action_test_starting_time = rospy.get_time()
        while not self.is_node_shutdown:
            starting_time = rospy.get_time()
            for action in self.config["action_list"]:
                if self.is_node_shutdown: break
                self.runner = Runner(self.config[action]["action_file"])
                for _ in range(self.config[action]["times"]):
                    if self.is_node_shutdown: break
                    self.runner.start()
                    while self.runner.is_running() and not self.is_node_shutdown:
                        rospy.sleep(0.1)
                    self.runner.stop()
                    rospy.sleep(self.config[action]["second_of_break_time"])
                del self.runner
            end_time = rospy.get_time()
            counter += 1
            single_round_execution_time = end_time - starting_time
            total_execution_time = end_time - action_test_starting_time
            servo_lost_communication, over_temperature, fsr_data = self.check_servo()
            is_network_connected = self.detect_network()
            is_terminate, text_msg = self.send_test_process_and_check_abort(counter, single_round_execution_time, total_execution_time, servo_lost_communication, over_temperature, fsr_data, is_network_connected)
            if is_terminate:
                break
            rospy.sleep(self.config["interval_time_per_round"])
        if self.is_node_shutdown:
            text_msg = "\n共执行了 {} 轮测试持续时间 {} 秒\n\n本轮测试时长 {} 秒".format(counter,\
                                total_execution_time,\
                                single_round_execution_time)
            text_msg = text_msg + "\n"
            self.logger.write(text_msg, print_color=33)
        else:
            text_msg = text_msg + "\n"
            self.logger.write(text_msg, print_color=33)

    @property
    def is_node_shutdown(self):
        return rospy.core.is_shutdown_requested()

    def rosShutdownHook(self):
        try:
            if self.runner:
                self.runner.stop()
        except AttributeError:
            pass
        mtrcl.ResetBodyhub()

if __name__ == "__main__":
    rospy.init_node("ActionTest", anonymous=False)
    at = ActionTest()
    at.loop_execute_action()
    at.stop_log()
