#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import math
import time
import numpy as np
import rospy
import rospkg
import tf
import numpy.matlib
from std_msgs.msg import *
from geometry_msgs.msg import *
import bodyhub_action as bodact
from algorithm import pidAlgorithm as pidAlg
from kick_ball_practice import KickBall 

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *

NODE_NAME = 'race_node'
# POSE_TOPIC = "/sim/torso/pose"
POSE_TOPIC = "/initialpose"
CONTROL_ID = 2
STEP_LEN_MAX = [0.06, 0.02, 10]

TASK1_PATH_POINT1 = [[0.45, -0.12, 0]]
TASK1_PATH_POINT2 = [[1.00, -0.20, 0],[1.43, -0.64, -80],[1.45, -1.00, -82]]
TASK2_PATH_POINT1 = [[1.63, -1.53, -71],[1.37, -2.19, -96],[1.58, -2.61, -63]]
TASK2_PATH_POINT2 = [[0.34, -2.39, 91],[0.24, -1.74, 91]]
TASK3_PATH_POINT1 = [[0.32, -1.36, 97]]



class RaceNode(bodact.Action):
    def __init__(self):
        super(RaceNode, self).__init__(NODE_NAME, CONTROL_ID)

        self.__gait_cmd_pub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=2)
        rospy.Subscriber(POSE_TOPIC, PoseWithCovarianceStamped, self.pose_callback)

        self.__debug = False
        self.current_pose = {0, 0, 0}
        self.pose_update = False
        self.err_threshold = [0.15, 0.15, 10]


    def toRPY(self, pose):
        return tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])


    def pose_callback(self, msg):
        p = msg.pose.pose.position
        x, y = p.x, p.y
        _, _, yaw = self.toRPY(msg.pose.pose)
        
        self.current_pose = [p.x, p.y, yaw * 180.0 / math.pi]
        self.pose_update = True
        if self.__debug:
            print( 'pose:', self.current_pose)

    def rot_mat(self, theta):
        theta = theta * math.pi / 180.0
        return np.array([[math.cos(theta), -math.sin(theta)], 
                        [math.sin(theta), math.cos(theta)]], dtype=np.float32)


    def path_tracking(self, path_point, mode=0):
        step_len = [0, 0, 0]
        rot_adjust = False
        path_marker, marker_num = 0, len(path_point)
        while not rospy.is_shutdown():
            rospy.wait_for_message('/requestGaitCommand', Bool, 10)

            if self.pose_update == False:
                print( 'location not updated!')
                time.sleep(0.5)
                continue

            if (mode == 1) and (path_marker == marker_num-1):
                time.sleep(0.5)

            for i in range(3):
                step_len[i] = path_point[path_marker][i] - self.current_pose[i]
            v = np.dot(np.linalg.inv(self.rot_mat(self.current_pose[2])), np.array([step_len[0], step_len[1]])).tolist()
            if (mode == 1) and (path_marker == marker_num-1):
                w = step_len[2]
            else:
                w = (math.atan2(step_len[1], step_len[0]) * 180.0 / math.pi) - self.current_pose[2]
            w = (w-360.0) if w >= 180.0 else w
            w = (w+360.0) if w <= -180.0 else w
            step_len = [v[0], v[1], w]
            self.pose_update = False

            pos_err_scale, rot_err_scale = 1.0, 10.0
            if (mode == 1) and (path_marker == marker_num-1):
                pos_err_scale, rot_err_scale = 0.4, 0.2
            if (abs(step_len[0]) < (self.err_threshold[0]*pos_err_scale)) and (abs(step_len[1]) < (self.err_threshold[1]*pos_err_scale)) and (abs(step_len[2]) < (self.err_threshold[2]*rot_err_scale)):
                path_marker = path_marker + 1
                print( 'path marker', path_marker, '/', marker_num)
                if path_marker >= marker_num:
                    self.bodyhub.wait_walking_done()
                    break

            if abs(step_len[2]) > 30:
                rot_adjust = True
            else:
                rot_adjust = False

            for i in range(3):
                step_len[i] = STEP_LEN_MAX[i] if step_len[i] > STEP_LEN_MAX[i] else step_len[i]
                step_len[i] = -STEP_LEN_MAX[i] if step_len[i] < -STEP_LEN_MAX[i] else step_len[i]
            if rot_adjust:
                self.__gait_cmd_pub.publish(data=[0.01, 0, step_len[2]])
            else:
                self.__gait_cmd_pub.publish(data=step_len)
        self.bodyhub.wait_walking_done()

    
    def set_head_rot(self, head_rot):
        keyframes = [
            ([0, -1, 16, -34, -17, -1, 0, 1, -16, 34, 17, 1, 0, -70, -15, 0, 70, 15, 0, 0, head_rot[0], head_rot[1]], 500, 0)
        ]
        self.movement.linearMove(keyframes)
        self.head_pitch = head_rot[1]
    
    def debug(self):
        self.set_arm_mode(0)
        self.bodyhub_ready()
        self.set_head_rot([0, 10])
        self.__debug = True
        while not rospy.is_shutdown():
            time.sleep(0.01)
        self.bodyhub_ready()
        self.set_head_rot([0, 0])
        self.set_arm_mode(1)

    def start(self,act_ball):
        
        self.set_arm_mode(0)                            
        self.bodyhub_ready()
        self.set_head_rot([0, 10])
        self.bodyhub_walk()
        self.path_tracking(TASK1_PATH_POINT1)           #走进地雷阵
    
        self.bodyhub_walk()
        self.bodyhub.walking_n_steps([0, -0.03, 0], 6)  
        self.bodyhub.wait_walking_done()                #往右走六步避开地雷

        self.bodyhub_walk()
        self.path_tracking(TASK1_PATH_POINT2)           #走出地雷阵，来到s弯道前
    
        self.bodyhub_ready()
        self.set_head_rot([0, 10])
        self.bodyhub_walk()
        self.path_tracking(TASK2_PATH_POINT1)           #走出s通道，来到彩虹弯道前
        
        self.bodyhub_walk()
        self.bodyhub.walking_n_steps([-0.03, 0, -8], 4)               
        self.bodyhub.wait_walking_done()                #以一定角度后退来调整方向

        self.bodyhub_walk()
        self.bodyhub.walking_n_steps([0.04, 0, -4.2], 32)
        self.bodyhub.wait_walking_done()  
        self.path_tracking(TASK2_PATH_POINT2)           #走彩虹弯道，来到直线通道前
           
        self.bodyhub_walk()      
        self.path_tracking(TASK3_PATH_POINT1)           #走直线通道，然后靠近小球，注意让下巴摄像头看到小球
    
        self.set_arm_mode(1)

        # 抵达目标点后，开始找球和踢球
        KickBall(sys.argv).start()

        rospy.signal_shutdown('exit')                   #恢复站立姿态并退出


if __name__ == '__main__':
    print("I'm starting")

    if len(sys.argv) >= 2 and (sys.argv[1] == 'debug'):
        RaceNode().debug()
    else:
        RaceNode().start('')
