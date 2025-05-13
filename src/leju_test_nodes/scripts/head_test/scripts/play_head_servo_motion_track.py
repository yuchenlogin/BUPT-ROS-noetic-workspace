#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from head_test import HeadTest
import json
import rospkg, os, sys
sys.path.append(os.path.join(rospkg.RosPack().get_path("leju_lib_pkg"), "src/"))
from lejufunc.bezier import get_bezier_frames
import threading
sys.path.append(os.path.join(rospkg.RosPack().get_path("leju_test_nodes"), "scripts"))
from healthy_checker import HealthChecker
from ros_work_weixin_robot_node.srv import WorkWeixinTextSend
import time
from logger import Logger

FILE_PATH_SAVE_TRACK = "/home/lemon/robot_ros_application/catkin_ws/src/leju_test_nodes/scripts/head_test/config/servo_motion_track.json"
FILE_PATH_CONFIG = "/home/lemon/robot_ros_application/catkin_ws/src/leju_test_nodes/scripts/head_test/config/config.json"
PER_SECOND = 1
SECOND_TO_MILLISECOND = 1000
ROOT_PATH = "/home/lemon/fatigue_test/head_test/"
TEST_DATA = time.strftime("%Y-%m-%d-%H.%M.%S", time.localtime(time.time()))
TEST_LOG_FOLDER = "{}{}/".format(ROOT_PATH, TEST_DATA)

class PlayMotionTrack:
    def __init__(self):
        self.headtest = HeadTest()
        self.healthchecker = HealthChecker()
        self.logger = Logger(TEST_LOG_FOLDER)
        self.track_list = self.load_json(FILE_PATH_SAVE_TRACK)
        self.config = self.load_json(FILE_PATH_CONFIG)
        self.play_stop = False
        self.keyboard_terminate = False
        rospy.on_shutdown(self.rosShutdownHook)

    def load_json(self, file_path):
        try:
            with open(file_path, "r") as f:
                rf = f.read()
        except IOError:
            rospy.logwarn("{} 文件不存在".format(file_path))
            exit(1)
        return json.loads(rf)

    def healthy_check(self):
        servo_lost_communication = self.healthchecker.check_servo_id(nomalids=self.config["servo_id_with_normal_communication"])
        overheated_servo = self.healthchecker.check_servo_temperature(temperature_limit=self.config["temperature_protection_value"])
        network_connect_status = self.healthchecker.check_network()
        camera_connect_status = self.healthchecker.check_camera()
        return servo_lost_communication, overheated_servo, network_connect_status, camera_connect_status

    def work_weixin_robot_text_send(self, text_msg):
        try:
            rospy.wait_for_service("/work_weixin/text_send", timeout=2)
        except rospy.ROSException:
            rospy.logwarn("action test work weixin robot strike!")
            return
        text_send_client = rospy.ServiceProxy("/work_weixin/text_send", WorkWeixinTextSend)
        text_send_client(self.config["robotKey"], text_msg, self.config["phone"])

    def send_test_process(self, counter, single_round_execution_time, total_execution_time, servo_lost_communication, overheated_servo, network_connect_status, camera_connect_status):
        error_list = []
        if servo_lost_communication:
            error_list.append("\n未扫描到的舵机 id:\n{}\n".format(" ".join(str(id) for id in servo_lost_communication)))
        if overheated_servo:
            error_list.append("\n过温的舵机 id 及温度:\n{}\n".format("\n".join(str(item) for item in overheated_servo)))
        if network_connect_status:
            error_list.append("\n{}\n".format(network_connect_status))
        if camera_connect_status:
            error_list.append("\n摄像头数据异常\n")
        if self.keyboard_terminate:
            error_list.append("\n用户停止\n")
        if len(error_list) or self.keyboard_terminate:
            text_msg = "共执行了 {} 轮测试持续时间 {} 秒\n\n本轮测试时长 {} 秒，由于以下原因测试终止：\n".format(counter, total_execution_time, single_round_execution_time)
            for err_msg in error_list:
                text_msg += err_msg
        else:
            text_msg = "已执行 {} 轮测试共 {} 秒\n\n本轮测试时长 {} 秒\n\n".format(counter,\
                                total_execution_time,\
                                single_round_execution_time)
        self.work_weixin_robot_text_send(text_msg)
        return text_msg

    def play_track(self):
        play_rate = rospy.Rate(95)
        action_time = SECOND_TO_MILLISECOND * self.config["time_spent_once_catch_track"]
        action_frames = []
        counter = 0
        head_test_starting_time = rospy.get_time()
        while not self.play_stop and not rospy.core.is_shutdown_requested():
            initial_head_position = self.headtest.get_head_servo_present_angle()
            if initial_head_position != None:
                p0 = initial_head_position
                self.track_list.append(initial_head_position)
                break
        for track in self.track_list:
            move_frames = get_bezier_frames(p0, track, None, action_time)[1:]
            p0 = track
            for frame in move_frames: action_frames.append(frame)
        while not rospy.core.is_shutdown_requested() or self.play_stop == False:
            starting_time = rospy.get_time()
            for _ in range(self.config["times"]):
                for frame in action_frames:
                    if rospy.core.is_shutdown_requested() or self.play_stop: break
                    play_rate.sleep()
                    self.headtest.set_head_servo_angle(frame)
                if rospy.core.is_shutdown_requested() or self.play_stop: break
                rospy.sleep(self.config["second_of_break_time"])
            end_time = rospy.get_time()
            single_round_execution_time = end_time - starting_time
            total_execution_time = end_time - head_test_starting_time
            counter += 1
            servo_lost_communication, overheated_servo, network_connect_status, camera_connect_status = self.healthy_check()
            if servo_lost_communication or overheated_servo or network_connect_status or camera_connect_status: self.play_stop = True
            text_msg = self.send_test_process(counter, single_round_execution_time, total_execution_time, servo_lost_communication, overheated_servo, network_connect_status, camera_connect_status)
            if self.play_stop == True: break
            rospy.sleep(self.config["interval_time_per_round"])
        text_msg = text_msg + "\n"
        self.logger.write(text_msg, print_color=33)

    def stop_log(self):
        self.logger.stop()

    def rosShutdownHook(self):
        self.play_stop = True
        self.headtest.set_bodyhub_to_reset()

    def main(self):
        play_thread = threading.Thread(target=self.play_track)
        play_thread.start()
        while not rospy.core.is_shutdown_requested():
            if raw_input("如果停止案例，请输入 q 并回车：\n"):
                self.play_stop = True
                self.keyboard_terminate = True
                break
        play_thread.join()

if __name__ == "__main__":
    rospy.init_node("PlayMotionTrack_node")
    PMT = PlayMotionTrack()
    PMT.main()
    PMT.stop_log()
