#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, rospkg
import sys, os
sys.path.append(os.path.join(rospkg.RosPack().get_path("leju_lib_pkg")))
from motion.motionControl import SetBodyhubTo_setStatus, ResetBodyhub
from bodyhub.srv import SrvServoAllRead, SrvServoAllWrite

BODYHUB_CONTROL_ID = 2
HEAD_SERVO_ID = [21, 22]

class HeadTest:
    def __init__(self):
        self.__set_bodyhub_to_ready()

    def __set_bodyhub_to_ready(self):
        SetBodyhubTo_setStatus(BODYHUB_CONTROL_ID)

    def set_bodyhub_to_reset(self):
        ResetBodyhub()

    def head_servo_torque_enable(self, state={}):
        set_torque_id = []
        set_torque_value = []
        try:
            rospy.wait_for_service("/MediumSize/BodyHub/DirectMethod/GetServoLockStateAll", timeout=2)
        except rospy.ROSException:
            rospy.logwarn("wait for get head servo lock state timeout!")
            return
        get_head_servo_lock_state_client = rospy.ServiceProxy("/MediumSize/BodyHub/DirectMethod/GetServoLockStateAll", SrvServoAllRead)
        response = get_head_servo_lock_state_client(HEAD_SERVO_ID, len(HEAD_SERVO_ID))
        get_head_servo_lock_state = dict(zip(HEAD_SERVO_ID, list(response.getData)))
        for item in state:
            if int(get_head_servo_lock_state[item]) != state[item]:
                set_torque_id.append(item)
        if set_torque_id:
            for id in set_torque_id:
                set_torque_value.append(state[id])
            try:
                rospy.wait_for_service("/MediumSize/BodyHub/DirectMethod/SetServoLockStateAll", timeout=2)
            except rospy.ROSException:
                rospy.logwarn("wait for set head servo lock state timeout!")
                return
            set_head_servo_lock_state_client = rospy.ServiceProxy("/MediumSize/BodyHub/DirectMethod/SetServoLockStateAll", SrvServoAllWrite)
            set_result = set_head_servo_lock_state_client(set_torque_id, len(set_torque_id), set_torque_value)
            if set_result.complete != True:
                rospy.logerr("set head servo lock state fail!")

    def get_head_servo_present_angle(self):
        try:
            rospy.wait_for_service("/MediumSize/BodyHub/DirectMethod/GetServoPositionAll", timeout=2)
        except rospy.ROSException:
            rospy.logwarn("wait for get head servo present angle timeout")
            return
        get_head_servo_present_angle_client = rospy.ServiceProxy("/MediumSize/BodyHub/DirectMethod/GetServoPositionAll", SrvServoAllRead)
        try:
            response = get_head_servo_present_angle_client(HEAD_SERVO_ID, len(HEAD_SERVO_ID))
            head_servo_present_angle = list(response.getData)
            return head_servo_present_angle
        except rospy.ServiceException:
            return None

    def set_head_servo_angle(self, servo_angle=[]):
        try:
            rospy.wait_for_service("/MediumSize/BodyHub/DirectMethod/SetServoTarPositionAll", timeout=2)
        except rospy.ROSException:
            rospy.logwarn("wait for set head servo angle timeout!")
            return
        set_head_servo_angle_client = rospy.ServiceProxy("/MediumSize/BodyHub/DirectMethod/SetServoTarPositionAll", SrvServoAllWrite)
        response = set_head_servo_angle_client(HEAD_SERVO_ID, len(HEAD_SERVO_ID), servo_angle)
        if response.complete != True:
            rospy.logerr("set head servo angle fail!")

if __name__ == "__main__":
    rospy.init_node("HeadTest", anonymous=False)
    HT = HeadTest()
