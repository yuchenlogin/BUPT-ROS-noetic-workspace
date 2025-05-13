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
from lejulib import terminate

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.bodyhub_client as bodycli
import vision.imageProcessing as imgPrcs
import algorithm.pidAlgorithm as pidAlg
from lejufunc import client_action
from motion.motionControl import SetBodyhubTo_setStatus, ResetBodyhub
from action import kickball_frame


BALL_GOAL_POS = [240.0, 260.0, 260.0]

# HSV阈值
lowerOrange = np.array([15, 100, 100])
upperOrange = np.array([25, 255, 255])
lowerCyan = np.array([80, 100, 100])
upperCyan = np.array([95, 255, 255])
lowerRed = np.array([0, 192, 96])
upperRed = np.array([20, 255, 240])

class Action(object):
    '''
    robot action
    '''

    def __init__(self, name, ctl_id):
        rospy.init_node(name, anonymous=True)
        time.sleep(0.2)
        rospy.on_shutdown(self.__ros_shutdown_hook)

        self.bodyhub = bodycli.BodyhubClient(ctl_id)

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

class KickBall(Action):
    def __init__(self, argv):
        super(KickBall, self).__init__('kick_ball', 2)
        rospy.Subscriber('terminate_current_process', String, terminate)

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
        id = 2
        SetBodyhubTo_setStatus(id)
        kick_ball_points_musics, kick_ball_points_frames = kickball_frame.kick_ball()
        client_action.custom_action(kick_ball_points_musics, kick_ball_points_frames)
        ResetBodyhub()

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
    KickBall(sys.argv).start()
