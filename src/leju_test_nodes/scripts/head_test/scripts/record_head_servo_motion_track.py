#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from head_test import HeadTest
import threading
import json

TORQUE_LOCK = 1
TORQUE_UNLOCK = 0
HEAD_SERVO_ID = [21, 22]
HEAD_SERVO_TORQUE_LOCK = {HEAD_SERVO_ID[0]: TORQUE_LOCK, HEAD_SERVO_ID[1]: TORQUE_LOCK}
HEAD_SERVO_TORQUE_UNLOCK = {HEAD_SERVO_ID[0]: TORQUE_UNLOCK, HEAD_SERVO_ID[1]: TORQUE_UNLOCK}
PER_SECOND = 1
FILE_PATH_SAVE_TRACK = "/home/lemon/robot_ros_application/catkin_ws/src/leju_test_nodes/scripts/head_test/config/servo_motion_track.json"
FILE_PATH_CONFIG = "/home/lemon/robot_ros_application/catkin_ws/src/leju_test_nodes/scripts/head_test/config/config.json"

class RecordMotionTrack:
    def __init__(self):
        self.headtest = HeadTest()
        self.stop_record = False
        self.angle_list = []
        self.config = self.load_config()
        rospy.on_shutdown(self.rosShutdownHook)

    def load_config(self):
        with open(FILE_PATH_CONFIG, "r") as f:
            rf = f.read()
        return json.loads(rf)

    def record_head_servo_motion_track(self):
        record_rate = rospy.Rate(PER_SECOND / self.config["time_spent_once_catch_track"])
        while not self.stop_record and not rospy.core.is_shutdown_requested():
            present_angle = self.headtest.get_head_servo_present_angle()
            if present_angle != None:
                self.angle_list.append(present_angle)
            else:
                continue
            record_rate.sleep()
        self.save_track()

    def save_track(self):
        try:
            with open(FILE_PATH_SAVE_TRACK, "w+") as wf:
                json.dump(self.angle_list, wf)
        except Exception as e:
            rospy.logerr("头部运动轨迹录制异常，请重新录制！")

    def rosShutdownHook(self):
        self.stop_record = True
        self.headtest.head_servo_torque_enable(HEAD_SERVO_TORQUE_LOCK)
        self.headtest.set_bodyhub_to_reset()

    def main(self):
        self.headtest.head_servo_torque_enable(HEAD_SERVO_TORQUE_UNLOCK)
        raw_input("头部舵机已解锁，敲击 回车键 开始录制头部舵机运动轨迹...")
        record_thread = threading.Thread(target=self.record_head_servo_motion_track)
        record_thread.start()
        while not rospy.core.is_shutdown_requested():
            if raw_input("如果录制结束，请输入 q 并回车:\n") == "q":
                self.stop_record = True
                break
        record_thread.join()

if __name__ == "__main__":
    rospy.init_node("RecordMotionTrack_node", anonymous=False)
    RMT = RecordMotionTrack()
    RMT.main()
