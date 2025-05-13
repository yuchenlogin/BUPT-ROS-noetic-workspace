#!/usr/bin/env python
# -*- coding: utf-8 -*-
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import subprocess
from bodyhub.srv import SrvServoScan, SrvPresentTemperature, SrvFSR
import time
import rospy
import rospkg
import os
import sys
sys.path.append(os.path.join(rospkg.RosPack().get_path("leju_lib_pkg")))
sys.path.append(os.path.join(
    rospkg.RosPack().get_path("leju_test_nodes"), "scripts"))

ROS_SERVICES_RETRY_TIMES = 3


class HealthChecker(object):
    def servo_scan(self, timeout=5):
        try:
            rospy.wait_for_service(
                "/MediumSize/BodyHub/ScanServo", timeout=timeout)
            servo_scan_client = rospy.ServiceProxy(
                "/MediumSize/BodyHub/ScanServo", SrvServoScan)
            response = servo_scan_client("torso")
        except Exception as e:
            rospy.logwarn("servo_scan error!{}".format(e))
            return []
        return list(response.getData)

    def get_servo_temperature(self, timeout=5):
        try:
            rospy.wait_for_service(
                "/MediumSize/BodyHub/PresentTemperature", timeout=timeout)
            present_temperature_client = rospy.ServiceProxy(
                "/MediumSize/BodyHub/PresentTemperature", SrvPresentTemperature)
            response = present_temperature_client()
        except Exception as e:
            rospy.logwarn("get servo temperature error!{}".format(e))
            return []
        return list(response.presentTemperature)

    def get_fsr(self, timeout=5):
        get_result = {}
        try:
            rospy.wait_for_service("/MediumSize/BodyHub/FSR", timeout=timeout)

            fsr_client = rospy.ServiceProxy("/MediumSize/BodyHub/FSR", SrvFSR)
            response = fsr_client()
            get_result["leftFSR"] = list(response.leftFSR)
            get_result["rightFSR"] = list(response.rightFSR)
        except Exception as e:
            rospy.logwarn("get fsr value error!{}".format(e))
            return get_result
        return get_result

    def check_fsr(self, timeout=5):
        for i in range(ROS_SERVICES_RETRY_TIMES):
            fsr_data = self.get_fsr(timeout)
            if len(fsr_data) or rospy.core.is_shutdown_requested():
                break
        res = [0 if len(fsr_data) and 0 not in fsr_data["leftFSR"] else 1, 0 if len(fsr_data) and 0 not in fsr_data["rightFSR"] else 1]
        return 0 if res == [0, 0] else [fsr_data["leftFSR"] if len(fsr_data) else None, fsr_data["rightFSR"] if len(fsr_data) else None]

    def check_servo_id(self, timeout=5, nomalids=list(range(1, 23))):
        for i in range(ROS_SERVICES_RETRY_TIMES):
            cur_servo_ids = self.servo_scan(timeout)
            if len(cur_servo_ids) or rospy.core.is_shutdown_requested():
                break
        servo_lost_communication = list(
            set(nomalids).difference(set(cur_servo_ids)))
        return servo_lost_communication if len(servo_lost_communication) else 0

    def check_servo_temperature(self, temperature_limit=65, timeout=5):
        for i in range(ROS_SERVICES_RETRY_TIMES):
            temperature_list = self.get_servo_temperature(timeout)
            if len(temperature_list) or rospy.core.is_shutdown_requested():
                break
        overheated_list = [[id+1, cur_temperature] for id, cur_temperature in enumerate(
            temperature_list) if cur_temperature > temperature_limit]
        return 0 if len(overheated_list) == 0 and len(temperature_list) != 0 else overheated_list

    def check_mic(self):
        if os.system("arecord -l | grep -qE \"Bothlent\"") == 0:
            return 0
        return 1

    def check_camera(self, timeout=5):
        try:
            rgb_image = rospy.wait_for_message(
                "/camera/color/image_raw", Image, timeout=timeout)
            cv_image = CvBridge().imgmsg_to_cv2(rgb_image, "bgr8")
            if not cv_image.shape[0]:
                raise Exception("摄像头数据异常")
            rgb_image = rospy.wait_for_message(
                "/chin_camera/image", Image, timeout=timeout)
            cv_image = CvBridge().imgmsg_to_cv2(rgb_image, "bgr8")
            if not cv_image.shape[0]:
                raise Exception("下巴摄像头数据异常")
        except Exception as e:
            rospy.logwarn(e)
            return -1
        return 0

    def check_network_connection(self):
        return subprocess.check_output(["iwgetid", "-r"])

    def check_network(self):
        if len(self.check_network_connection()):
            if subprocess.call("timeout 2 ping www.baidu.com -c 1", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE) == 0:
                return 0
            else:
                return "已连接wifi但无法访问网络"
        else:
            return "未连接到wifi"


if __name__ == "__main__":
    rospy.init_node("healthy_checker")
    h = HealthChecker()
    while not rospy.is_shutdown():
        time.sleep(0.1)
        print(h.check_fsr())
