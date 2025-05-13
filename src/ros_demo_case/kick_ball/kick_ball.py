#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
import rospy
import rospkg
import numpy as np
import cv2
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))

from ball_detecter import ball_detecter
from kickball_frame import kick_ball_points_frames
from lejufunc import client_action
import algorithm.pidAlgorithm as pidAlg
import motion.bodyhub_client as bodycli
from lejulib import finishsend

BALL_GOAL_POS = [240.0, 260.0, 260.0] # 期望球落于画面中的位置[x, y, y]

class KickBall(bodycli.BodyhubClient):
    """继承bodyhubclient获得运动控制能力"""
    def __init__(self, debug=False):
        super(KickBall, self).__init__(id=2)
        self.__debug = debug
        self.detecter = ball_detecter(debug=debug)

        self.__pid_x = pidAlg.PositionPID(p=0.0008)
        self.__pid_y = pidAlg.PositionPID(p=0.0003)
        self.__pid_a = pidAlg.PositionPID(p=0.09)
        self.__err_threshold = [20.0, 20.0, 20.0]

    def goto_ball(self, goal_pos):
        """走向小球的位置: 通过运动控制将小球落于画面中的goal_pos位置"""
        self.walk() #  状态跳转到walking, 允许行走
        while not rospy.is_shutdown():
            result = self.detecter.detect() # 获取颜色识别结果, 返回小球位置
            if result['find'] != False:
                xError = goal_pos[0] - result['Cy'] # 计算机器人前进方向上小球在画面中的位置与期望位置的差值, 注意此处前进方向为图像中的y轴方向
                yError = goal_pos[1] - result['Cx'] # 计算图像x方向的差值
                aError = goal_pos[2] - result['Cx'] # 使用图像x方向的位置差值作为转角差值, 能让机器人更快朝向小球
                if (abs(xError) < self.__err_threshold[0]) and (abs(yError) < self.__err_threshold[1]) and (abs(aError) < self.__err_threshold[2]):
                    break # 误差小于阈值则退出
                xLength = self.__pid_x.run(xError) # 将差值传入pid控制器获得x\y\a方向的控制量
                yLength = self.__pid_y.run(yError)
                aLength = self.__pid_a.run(aError)
                self.walking_the_distance(xLength, yLength, aLength) # 执行动作进行校正, 往前:x+, 往左:y+, 逆时针:a+
                self.wait_walking_done()# 等待动作执行完毕
            else:
                rospy.logwarn('no ball found!')
                time.sleep(0.5)

    def kick(self):
        self.ready()# 状态跳转到ready, 允许执行动作
        client_action.custom_action(
            music=[], act_frames=kick_ball_points_frames) # 调用执行动作的接口, 传入踢球动作帧

    def start(self):
        self.goto_ball(BALL_GOAL_POS)
        self.kick()
        self.reset()


if __name__ == '__main__':
    rospy.init_node("kick_ball", anonymous=True)
    rospy.on_shutdown(finishsend)
    time.sleep(0.2)
    KickBall(debug = True if len(sys.argv) > 1 else False).start()
