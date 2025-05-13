#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import math
import time
import rospy
import rospkg
import numpy as np
import cv2 as cv
from cv_bridge import *
from std_msgs.msg import *
from sensor_msgs.msg import *

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *

sys.path.append(rospkg.RosPack().get_path('ros_color_node') + '/scripts')
from color_filter import rgb_to_hsv

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.bodyhub_client as bodycli
import motion.TrajectoryPlan as tPlan
import vision.imageProcessing as imgPrcs
import algorithm.pidAlgorithm as pidAlg


BALL_GOAL_POS = [240.0, 260.0, 260.0]

# low_rgb = [132, 91, 86]
# up_rgb = [179, 107, 99]
# low_ball_hsv = rgb_to_hsv(low_rgb)
# up_ball_hsv = rgb_to_hsv(up_rgb)
rgb_ball = [163, 51, 42]
rgb_ball_thred = 16


# HSV阈值
lowerOrange = np.array([15, 100, 100])
upperOrange = np.array([25, 255, 255])
lowerCyan = np.array([80, 100, 100])
upperCyan = np.array([95, 255, 255])
lowerRed = np.array([0, 83, 115])
upperRed = np.array([10, 255, 193])


class Movement:
    '''
    生成、发送动作轨迹
    '''

    def __init__(self, bodyhub_cli):
        self.bodyhub = bodyhub_cli

    def jointMoveToPosition(self, duration, ids, current_pos, goal_pos):
        sequence = []
        joint_pos = [float(0) for i in range(len(ids))]
        for i in range(len(ids)):
            sequence.append(list(np.linspace(current_pos[i], goal_pos[i], int(duration/0.01))))
        for y in range(len(sequence[0])):
            for x in range(len(ids)):
                joint_pos[x] = sequence[x][y]
            self.bodyhub.set_joint_position(ids, joint_pos)

    def linearMove(self, keyframes):
        ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
        keyframe = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 1000, 0]
        keyframe[0] = self.bodyhub.get_joint_position(ids)
        keyframes.insert(0, keyframe)

        for idx in range(1, len(keyframes)):
            self.jointMoveToPosition(keyframes[idx][1]/1000.0, ids, keyframes[idx-1][0], keyframes[idx][0])
            if keyframes[idx][2] > 0:
                self.bodyhub.wait_joint_done(True)
                time.sleep(keyframes[idx][2]/1000.0)
            else:
                self.bodyhub.wait_joint_done()
        self.bodyhub.wait_joint_done(True)

    def sendBezierTraj(self, trajectoryPoint):
        ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
        for m in range(len(trajectoryPoint[0])):
            jointPosition = []
            for n in range(len(trajectoryPoint)):
                jointPosition.append(trajectoryPoint[n][m].y)
            self.bodyhub.set_joint_position(ids, jointPosition)

    def bezierMove(self, keyframes):
        ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
        keyframe = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 1000, 0]
        keyframe[0] = self.bodyhub.get_joint_position(ids)
        keyframes.insert(0, keyframe)

        tpObject = tPlan.TrajectoryPlanning(22, 10.0)
        tpObject.setInterval(keyframes[1][1])
        tpObject.planningBegin(keyframes[0][0], keyframes[1][0])

        for idx in range(2, len(keyframes)):
            tpObject.setInterval(keyframes[idx][1])
            trajectoryPoint = tpObject.planning(keyframes[idx][0])
            sendBezierTraj(trajectoryPoint)
            if keyframes[idx][2] > 0:
                self.bodyhub.wait_joint_done(True)
                time.sleep(keyframes[idx][2]/1000.0)
            else:
                self.bodyhub.wait_joint_done()

        trajectoryPoint = tpObject.planningEnd()
        sendBezierTraj(trajectoryPoint)
        self.bodyhub.wait_joint_done(True)


class Action(object):
    '''
    robot action
    '''

    def __init__(self, name, ctl_id):
        # rospy.init_node(name, anonymous=True)
        # time.sleep(0.2)
        rospy.on_shutdown(self.__ros_shutdown_hook)

        self.bodyhub = bodycli.BodyhubClient(ctl_id)
        self.movement = Movement(self.bodyhub)

    def __ros_shutdown_hook(self):
        if self.bodyhub.reset() == True:
            rospy.loginfo('bodyhub reset, exit')
        else:
            rospy.loginfo('exit')

    def bodyhub_ready(self):
        if self.bodyhub.ready() == False:
            rospy.logerr('bodyhub to ready failed!')
            rospy.signal_shutdown('error')
            time.sleep(1)
            exit(1)

    def bodyhub_walk(self):
        if self.bodyhub.walk() == False:
            rospy.logerr('bodyhub to walk failed!')
            rospy.signal_shutdown('error')
            time.sleep(1)
            exit(1)

    def start(self):
        '''
        action example
        '''
        # 步态
        self.bodyhub_walk()
        self.bodyhub.walking_the_distance(1.0, 0.0, 0.0)
        self.bodyhub.wait_walking_done()
        # 动作
        self.bodyhub_ready()
        keyframes = [
            ([0, -1, 16, -34, -17, -1, 0, 1, -16, 34, 17, 1, 0, -70, -15, 0, 70, 15, 0, 0, 0, 25], 500, 100),
            ([0, -12, 16, -34, -17, -12, 0, -11, -16, 34, 17, -11, 0, -70, -15, 0, 70, 15, 0, 0, 0, 25], 800, 0),
            ([-5, -14, 35, -70, -35, -14, -5, -10, -16, 34, 17, -12, 0, -70, -15, 0, 70, 15, 0, 0, 0, 25], 500, 0),
            ([-10, -14, 30, -75, -30, -14, -10, -10, -16, 34, 17, -12, 0, -70, -15, 0, 70, 15, 0, 0, 0, 25], 500, 200),
            ([5, -14, 55, -40, 15, -14, 5, -10, -16, 34, 17, -12, 0, -70, -15, 0, 70, 15, 0, 0, 0, 25], 400, 200),
            ([-5, -14, 35, -70, -35, -14, -5, -10, -16, 34, 17, -12, 0, -70, -15, 0, 70, 15, 0, 0, 0, 25], 500, 0),
            ([0, -14, 18, -36, -18, -14, 0, -11, -16, 34, 17, -11, 0, -70, -15, 0, 70, 15, 0, 0, 0, 25], 1000, 0),
            ([0, -1, 16, -34, -17, -1, 0, 1, -16, 34, 17, 1, 0, -70, -15, 0, 70, 15, 0, 0, 0, 25], 1000, 100)
        ]
        self.movement.linearMove(keyframes)


class KickBall(Action):
    def __init__(self, argv):
        super(KickBall, self).__init__('kick_ball', 2)

        if len(argv) > 1 and argv[1] == 'sim':
            self.__run_type = 0
            self.__ball = imgPrcs.ColorObject(lowerOrange, upperOrange)
            self.__hole = imgPrcs.ColorObject(lowerCyan, upperCyan)
            image_topic = '/sim/camera/UVC/colorImage'
        else:
            self.__run_type = 1
            self.__ball = imgPrcs.ColorObject(lowerRed, upperRed)
            self.__hole = imgPrcs.ColorObject(lowerCyan, upperCyan)
            image_topic = '/chin_camera/image'

        self.__cv_bridge = CvBridge()
        self.__img_origin = np.zeros((640, 480, 3), np.uint8)
        self.__fps_time = 0
        rospy.Subscriber(image_topic, Image, self.__image_callback)

        self.__pid_x = pidAlg.PositionPID(p=0.0008)
        self.__pid_y = pidAlg.PositionPID(p=0.0003)
        self.__pid_a = pidAlg.PositionPID(p=0.09)
        self.__err_threshold = [20.0, 20.0, 20.0]

    def __image_callback(self, msg):
        try:
            self.__img_origin = self.__cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as err:
            rospy.logerr(err)

        if False:
            t0 = time.time()
            imgPrcs.putVisualization(self.__img_origin, self.__ball.detection(self.__img_origin))
            t1 = time.time()
            fps = 1.0/(time.time() - self.__fps_time)
            self.__fps_time = time.time()
            imgPrcs.putTextInfo(self.__img_origin, fps, (t1-t0)*1000)
            cv.imshow("image window", self.__img_origin)
            cv.waitKey(1)

    def goto_ball(self, goal_pos):
        while not rospy.is_shutdown():
            result = self.__ball.detection(self.__img_origin)
            if result['find'] != False:
                xError = goal_pos[0] - result['Cy']
                yError = goal_pos[1] - result['Cx']
                aError = goal_pos[2] - result['Cx']
                if (abs(xError) < self.__err_threshold[0]) and (abs(yError) < self.__err_threshold[1]) and (abs(aError) < self.__err_threshold[2]):
                    break
                xLength = self.__pid_x.run(xError)
                yLength = self.__pid_y.run(yError)
                aLength = self.__pid_a.run(aError)
                self.bodyhub.walking_the_distance(xLength, yLength, aLength)
                self.bodyhub.wait_walking_done()
            else:
                rospy.logwarn('no ball found!')
                time.sleep(0.5)

    def prepare_kick(self, goal_pos):
        while not rospy.is_shutdown():
            result1 = self.__ball.detection(self.__img_origin)
            result2 = self.__hole.detection(self.__img_origin)
            if (result1['find'] != False) and (result2['find'] != False):
                xError = goal_pos[0] - result1['Cy']
                yError = goal_pos[1] - result1['Cx']
                aError = goal_pos[2] - result2['Cx']
                if (abs(xError) < self.__err_threshold[0]) and (abs(yError) < self.__err_threshold[1]) and (abs(aError) < self.__err_threshold[2]):
                    break
                xLength = self.__pid_x.run(xError)
                yLength = self.__pid_y.run(yError)
                aLength = self.__pid_a.run(aError)
                self.bodyhub.walking_the_distance(xLength, yLength, aLength)
                self.bodyhub.wait_walking_done()

    def kick(self):
        try:
            u8e22u743u4feeu6539u54e1_frames = {"1":[[[490,0],[0,0],[390,0]],[[1790,7],[-390,0],[150,0]],[[2290,3.5],[-150,1.05],[150,-1.05]],[[2790,0],[-150,0],[243,0]],[[3600,7],[-243,0],[480,0]],[[5200,0],[-480,0],[0,0]]],"2":[[[490,-1.5],[0,0],[390,0]],[[1790,-13],[-390,0],[150,0]],[[2290,-13],[-150,0],[150,0]],[[2790,-13],[-150,0],[243,0]],[[3600,-13],[-243,0],[480,0]],[[5200,-1.5],[-480,0],[0,0]]],"3":[[[490,16.5],[0,0],[390,0]],[[1790,16.5],[-390,0],[150,0]],[[2290,28.3],[-150,-3.525],[150,3.525]],[[2790,40],[-150,0],[243,0]],[[3600,16.5],[-243,0],[480,0]],[[5200,16.5],[-480,0],[0,0]]],"4":[[[490,-34.1],[0,0],[390,0]],[[1790,-43.1],[-390,0],[150,0]],[[2290,-43],[-150,-6.464999999999999],[150,6.464999999999999]],[[2790,0],[-150,0],[243,0]],[[3600,-43.1],[-243,0],[480,0]],[[5200,-34.1],[-480,0],[0,0]]],"5":[[[490,-17.6],[0,0],[390,0]],[[1790,-23.6],[-390,0],[150,0]],[[2290,-15.8],[-150,-10.290000000000001],[150,10.290000000000001]],[[2790,45],[-150,0],[243,0]],[[3600,-23.6],[-243,0],[480,0]],[[5200,-17.6],[-480,0],[0,0]]],"6":[[[490,-1.5],[0,0],[390,0]],[[1790,-13],[-390,0],[150,0]],[[2290,-13],[-150,0],[150,0]],[[2790,-13],[-150,0],[243,0]],[[3600,-13],[-243,0],[480,0]],[[5200,-1.5],[-480,0],[0,0]]],"7":[[[490,0],[0,0],[390,0]],[[1790,7],[-390,0],[150,0]],[[2290,3.5],[-150,1.05],[150,-1.05]],[[2790,0],[-150,0],[243,0]],[[3600,7],[-243,0],[480,0]],[[5200,0],[-480,0],[0,0]]],"8":[[[490,1.5],[0,0],[390,0]],[[1790,-7],[-390,0],[150,0]],[[2290,-5],[-150,-0.6],[150,0.6]],[[2790,-3],[-150,0],[243,0]],[[3600,-7],[-243,0],[480,0]],[[5200,1.5],[-480,0],[0,0]]],"9":[[[490,-16.5],[0,0],[390,0]],[[1790,-16.5],[-390,0],[150,0]],[[2290,-10.2],[-150,-1.875],[150,1.875]],[[2790,-4],[-150,0],[243,0]],[[3600,-16.5],[-243,0],[480,0]],[[5200,-16.5],[-480,0],[0,0]]],"10":[[[490,34.1],[0,0],[390,0]],[[1790,34.1],[-390,0],[150,0]],[[2290,23.6],[-150,3.1650000000000005],[150,-3.1650000000000005]],[[2790,13],[-150,0],[243,0]],[[3600,34.1],[-243,0],[480,0]],[[5200,34.1],[-480,0],[0,0]]],"11":[[[490,17.6],[0,0],[390,0]],[[1790,17.6],[-390,0],[150,0]],[[2290,10.8],[-150,2.04],[150,-2.04]],[[2790,4],[-150,0],[243,0]],[[3600,17.6],[-243,0],[480,0]],[[5200,17.6],[-480,0],[0,0]]],"12":[[[490,1.5],[0,0],[390,0]],[[1790,-13],[-390,0],[150,0]],[[2290,-13],[-150,0],[150,0]],[[2790,-13],[-150,0],[243,0]],[[3600,-13],[-243,0],[480,0]],[[5200,1.5],[-480,0],[0,0]]],"13":[[[490,0],[0,0],[390,0]],[[1790,0],[-390,0],[150,0]],[[2290,-13.5],[-150,4.050000000000001],[150,-4.050000000000001]],[[2790,-27],[-150,0],[243,0]],[[3600,0],[-243,0],[480,0]],[[5200,0],[-480,0],[0,0]]],"14":[[[490,-70],[0,0],[390,0]],[[1790,-37],[-390,0],[150,0]],[[2290,-47.3],[-150,3.5999999999999996],[150,-3.5999999999999996]],[[2790,-61],[-150,2.0267175572519087],[243,-3.2832824427480922]],[[3600,-65],[-243,0.9074688796680498],[480,-1.79253112033195]],[[5200,-70],[-480,0],[0,0]]],"15":[[[490,-15],[0,0],[390,0]],[[1790,-18],[-390,0],[150,0]],[[2290,-12],[-150,-1.7999999999999998],[150,1.7999999999999998]],[[2790,-6],[-150,0],[243,0]],[[3600,-18],[-243,0],[480,0]],[[5200,-15],[-480,0],[0,0]]],"16":[[[490,0],[0,0],[390,0]],[[1790,0],[-390,0],[150,0]],[[2290,-13.5],[-150,4.050000000000001],[150,-4.050000000000001]],[[2790,-27],[-150,0],[243,0]],[[3600,0],[-243,0],[480,0]],[[5200,0],[-480,0],[0,0]]],"17":[[[490,70],[0,0],[390,0]],[[1790,37],[-390,0],[150,0]],[[2290,47.3],[-150,-3.5999999999999996],[150,3.5999999999999996]],[[2790,61],[-150,-2.0267175572519087],[243,3.2832824427480922]],[[3600,65],[-243,-0.9074688796680498],[480,1.79253112033195]],[[5200,70],[-480,0],[0,0]]],"18":[[[490,15],[0,0],[390,0]],[[1790,18],[-390,0],[150,0]],[[2290,18],[-150,0],[150,0]],[[2790,18],[-150,0],[243,0]],[[3600,18],[-243,0],[480,0]],[[5200,15],[-480,0],[0,0]]],"19":[[[490,0],[0,0],[390,0]],[[1790,0],[-390,0],[150,0]],[[2290,0],[-150,0],[150,0]],[[2790,0],[-150,0],[243,0]],[[3600,0],[-243,0],[480,0]],[[5200,0],[-480,0],[0,0]]],"20":[[[490,0],[0,0],[390,0]],[[1790,0],[-390,0],[150,0]],[[2290,0],[-150,0],[150,0]],[[2790,0],[-150,0],[243,0]],[[3600,0],[-243,0],[480,0]],[[5200,0],[-480,0],[0,0]]],"21":[[[490,0],[0,0],[390,0]],[[1790,0],[-390,0],[150,0]],[[2290,0],[-150,0],[150,0]],[[2790,0],[-150,0],[243,0]],[[3600,0],[-243,0],[480,0]],[[5200,0],[-480,0],[0,0]]],"22":[[[490,0],[0,0],[390,0]],[[1790,0],[-390,0],[150,0]],[[2290,0],[-150,0],[150,0]],[[2790,0],[-150,0],[243,0]],[[3600,0],[-243,0],[480,0]],[[5200,0],[-480,0],[0,0]]]}
            u8e22u743u4feeu6539u54e1_musics = []
            client_action.custom_action(u8e22u743u4feeu6539u54e1_musics,u8e22u743u4feeu6539u54e1_frames)


        except Exception as err:
            serror(err)
        finally:
            finishsend()

    def start(self):
        if self.__run_type == 0:
            self.bodyhub_walk()
            self.goto_ball([330.0, 320.0, 320.0])
            self.prepare_kick([360.0, 280.0, 285.0])
            self.bodyhub_ready()
            self.kick()
        else:
            self.bodyhub_walk()
            self.goto_ball(BALL_GOAL_POS)
            self.bodyhub_ready()
            self.kick()


if __name__ == '__main__':
    rospy.init_node("kick_ball_node", anonymous=True)

    KickBall(sys.argv).start()
