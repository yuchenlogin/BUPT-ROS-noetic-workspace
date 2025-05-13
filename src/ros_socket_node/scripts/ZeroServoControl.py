#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy
import time
from bodyhub.srv import *

FRAME_TIME = 10

def GetCurrentValue():
    id_array = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
    id_count = 22
    rospy.wait_for_service('/MediumSize/BodyHub/DirectMethod/GetServoPositionValAll', 2)
    moto_client = rospy.ServiceProxy('/MediumSize/BodyHub/DirectMethod/GetServoPositionValAll', SrvServoAllRead)
    result = moto_client(id_array, id_count)
    current_value = result.getData

    return current_value


def GetFrameWithSpeed(start, end, time):
    frame_number = time // FRAME_TIME + 1
    move_sequence = []
    for i in range(len(start)):
        move_sequence.append(numpy.linspace(start[i], end[i], num=frame_number, endpoint=True))
    target_frames = []
    for i in range(frame_number):
        cur_frame = []
        for j in range(len(start)):
            cur_frame.append(move_sequence[j][i])
        target_frames.append(cur_frame)

    return target_frames


def ZeroServoTransfer(arraySend):
    rospy.loginfo("ZeroServoTransfer begin...")    
    diff = []      
    ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
    cur_moto_value = GetCurrentValue()
    servo_value = cur_moto_value
    for i in range(len(ids)):
        diff.append(abs(cur_moto_value[i]-arraySend[i]))
    maxValue = int(max(diff)) 
    time = maxValue if maxValue > 20 else 20
    rospy.loginfo("ZeroServoTransfer time : %d ms", time)
    move_frames = GetFrameWithSpeed(cur_moto_value, arraySend, 10*time)
    rospy.wait_for_service("/MediumSize/BodyHub/DirectMethod/SetServoTarPositionValAll",2)
    servomoto = rospy.ServiceProxy("/MediumSize/BodyHub/DirectMethod/SetServoTarPositionValAll",SrvServoAllWrite)
    for frame in move_frames:
        servomoto(ids,len(frame),frame)

       





