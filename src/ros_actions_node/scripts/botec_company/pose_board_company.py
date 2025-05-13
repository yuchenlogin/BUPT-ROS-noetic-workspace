#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import time
import numpy as np
import rospy
import rospkg
import bodyhub_action as bodyact
sys.path.append(os.path.split(os.path.split(sys.argv[0])[0])[0])
import botec.navigation as nav
from std_msgs.msg import String

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *


class PoseBoard(bodyact.Action, nav.Navigation):
    def __init__(self, body_client):
        bodyact.Action.__init__(self, body_client)
        nav.Navigation.__init__(self, body_client, False)
        self.bodyhub = body_client
        self.marker_debug_print_on = 0b01
        self.slam_debug_print_on = 0b10

        # self.use_slam = False
        self.use_slam = True
        self.slam_path1_point = [[0.03,0.0,0.00], [0.261,0.015,0.031]]
        self.slam_path2_point = [[0.72,0.025,0.079], [0.956,0.027,0.107]]

    def set_head_rot(self, head_rot):
        keyframes = [
            ([0, -1, 16, -34, -17, -1, 0, 1, -16, 34, 17, 1, 0, -70, -15, 0, 70, 15, 0, 0, head_rot[0], head_rot[1]], 500, 0)
        ]
        self.movement.linearMove(keyframes)
        self.head_rot = [0, head_rot[1], head_rot[0]]

    def board_act(self):
        try:
            board_frames = {"1": [[[1000, 0], [0, 0], [0, 0]]], "2": [[[1000, -1.5], [0, 0], [0, 0]]], "3": [[[1000, 0], [0, 0], [0, 0]]], "4": [[[1000, 0], [0, 0], [0, 0]]], "5": [[[1000, 0], [0, 0], [0, 0]]], "6": [[[1000, -1.5], [0, 0], [0, 0]]], "7": [[[1000, 0], [0, 0], [0, 0]]], "8": [[[1000, 1.5], [0, 0], [0, 0]]], "9": [[[1000, 0], [0, 0], [0, 0]]], "10": [[[1000, 0], [0, 0], [0, 0]]], "11": [[[1000, 0], [0, 0], [0, 0]]], "12": [[[1000, 1.5], [0, 0], [0, 0]]], "13": [[[1000, 0], [0, 0], [0, 0]]], "14": [[[1000, -35], [0, 0], [0, 0]]], "15": [[[1000, 0], [0, 0], [0, 0]]], "16": [[[1000, 0], [0, 0], [0, 0]]], "17": [[[1000, -5], [0, 0], [0, 0]]], "18": [[[1000, 85], [0, 0], [0, 0]]], "19": [[[1000, 0], [0, 0], [0, 0]]], "20": [[[1000, 0], [0, 0], [0, 0]]], "21": [[[1000, 0], [0, 0], [0, 0]]], "22": [[[1000, 0], [0, 0], [0, 0]]]}
            board_musics = []
            client_action.custom_action(board_musics, board_frames)
        except Exception as err:
            serror(err)
        finally:
            pass

    def debug(self):
        self.bodyhub_ready()
        self.set_debug_print(self.marker_debug_print_on + self.slam_debug_print_on)
        while not rospy.is_shutdown():
            time.sleep(0.01)

    def start(self):
        if self.use_slam:
            self.bodyhub_ready()
            # self.set_head_rot([0, 10])
            # self.set_arm_mode(0)
            self.bodyhub_walk()
            self.path_tracking(self.slam_path1_point,jump_list=[0])
            self.bodyhub.wait_walking_done()
            # self.bodyhub_ready()
            # self.set_head_rot([0, 10])
            # self.set_arm_mode(1)
        else:
            self.bodyhub_walk()
            self.bodyhub.walking_n_steps([0.08, 0.0, 0.0], 6)
            self.bodyhub.wait_walking_done()

        self.bodyhub_walk()
        self.goto_pose(0, [0.094,-0.236,0.615])

        self.bodyhub_ready()
        self.board_act()
        time.sleep(8)
        self.bodyhub.reset()

        if self.use_slam:
            self.bodyhub_ready()
            # self.set_head_rot([0, 10])
            # self.set_arm_mode(0)
            self.bodyhub_walk()
            self.path_tracking(self.slam_path2_point)
            self.bodyhub.wait_walking_done()
            # self.bodyhub_ready()
            # self.set_head_rot([0, 0])
            # self.set_arm_mode(1)
        
        else:
            self.bodyhub_walk()
            self.bodyhub.walking_n_steps([0.08, 0.0, 0.0], 6)
            self.bodyhub.wait_walking_done()

        self.bodyhub_walk()
        # self.goto_rot(1, 0.0)
        self.goto_pose(1, [0.088,-0.214,0.575])
        self.bodyhub_walk()

def terminate(data):
    rospy.loginfo(data.data)
    rospy.signal_shutdown("kill")

if __name__ == '__main__':
    from motion import bodyhub_client as bodycli
    rospy.init_node('botech_pose_board_node', anonymous=True)
    rospy.Subscriber('terminate_current_process', String, terminate)
    time.sleep(0.2)
    obj = PoseBoard(bodycli.BodyhubClient(2))
    if len(sys.argv) >= 2 and (sys.argv[1] == 'debug'):
        obj.debug()
    else:
        obj.start()
