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
import yaml
from std_msgs.msg import *
from rosgraph_msgs.msg import Log
from geometry_msgs.msg import *
from algorithm import pidAlgorithm as pidAlg

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/src')
import lejufunc.bodyhub_action as bodact
from lejulib import *
SCRIPTS_PATH=os.path.split(sys.argv[0])[0]

# POSE_TOPIC = "/sim/torso/pose"
POSE_TOPIC = "/initialpose"
STEP_LEN_MAX = [0.06, 0.02, 10]


class Slam_Moving_Node(bodact.Action):
    def __init__(self,NODE_NAME,CONTROL_ID):
        super(Slam_Moving_Node, self).__init__(NODE_NAME, CONTROL_ID, init_node=False)
        self.__gait_cmd_pub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=2)
        rospy.Subscriber(POSE_TOPIC, PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber("/rosout", Log, self.rosout_callback)
        self.__debug = False
        self.current_pose = [0, 0, 0]
        self.pose_update = False
        self.err_threshold = [0.12, 0.12, 10]

    def rosout_callback(self,data):
        if "ik failed" in data.msg and hasattr(self,"logger"):
            self.logger.write("[error]"+data.msg+"\n")

    def get_pos(self):
        return self.current_pose,self.pose_update

    def quart_to_rpy(self, w, x, y, z):
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
        return [r, p, y]


    def toRPY(self, pose):
        return tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    def pose_callback(self, msg):
        p = msg.pose.pose.position
        x, y = p.x, p.y
        _, _, yaw = self.toRPY(msg.pose.pose)
        old_pos=self.current_pose
        self.current_pose = [p.x, p.y, yaw * 180.0 / math.pi]
        if self.__debug:
            print ('pose:', self.current_pose)
        if ((old_pos[0]-self.current_pose[0])**2+(old_pos[1]-self.current_pose[1])**2)>=1 or abs(old_pos[2]-self.current_pose[2])>50:
            return#滤掉高频跳点
        self.pose_update = True

    def rot_mat(self, theta):
        theta = theta * math.pi / 180.0
        return np.array([[math.cos(theta), -math.sin(theta)], 
                        [math.sin(theta), math.cos(theta)]], dtype=np.float32)
    def location_lost(self):
        pass
    def path_tracking(self, path_point, pos_wait_mode=0,ignore_angle=False,pos_exit_high_acc=True,wait_time=0.4):
        '''
        pos_wait_mode=1:等待机器人站稳后才进行定位,精度高但慢
        ignore_angle模式:用于回程运动,忽略标记点的方向,直接沿着两点之间连线走
        wait_time:等待机器人定位稳定的时间
        pos_exit_high_acc:确保终点精度高,调节时间会增加
        '''
        step_len = [0, 0, 0]
        rot_adjust = False
        path_marker_index, marker_num = 0, len(path_point)
        step = 0 # 1:允许根据标定位置调整机器人方向，0不允许调整方向,直接往目标点走,-1保持当前角度
        self.no_locate_time = time.time()
        while not rospy.core.is_shutdown_requested():
            rospy.wait_for_message('/requestGaitCommand', Bool, 10)

            if self.pose_update == False:
                print ('location not updated!')
                self.location_lost()
                time.sleep(0.5)
                continue
            self.no_locate_time = time.time()
            current_pose=self.current_pose
            for i in range(3):
                step_len[i] = path_point[path_marker_index][i] - current_pose[i]
            print("\n\ncurrent pos{} \ntarget{} \ndifflen{}".format(current_pose,path_point[path_marker_index],step_len))
            if (pos_wait_mode == 1):
                time.sleep(wait_time)
            # time.sleep(1)
            v = np.dot(np.linalg.inv(self.rot_mat(current_pose[2])), np.array([step_len[0], step_len[1]])).tolist()

            if_close = v[0]**2+v[1]**2<=(self.err_threshold[0]*1.2)**2 #快要到达目标点了
            if if_close :
                if ignore_angle:#靠很近并且忽略角度,才根据标定点转动
                    step = -1
                else:#靠很近并且没有忽略角度,不能转动只做微调
                    step = 1
            else:
                step = 0

            if (step==1):# use the calibrated direction
            # if not ignore_angle and v[0]**2+v[1]**2<=(self.err_threshold[0]*2)**2: # 
                w = step_len[2]
            elif step == 0: # use the direction along the line between two points
                w = (math.atan2(step_len[1], step_len[0]) * 180.0 / math.pi) - current_pose[2]
            else:
                w = 0
            w = (w-360.0) if w >= 180.0 else w
            w = (w+360.0) if w <= -180.0 else w
            step_len = [v[0], v[1], w]

            pos_err_scale, rot_err_scale = 1.0, 1.0
            if (pos_wait_mode == 1) or (pos_exit_high_acc and (path_marker_index == marker_num-1)):# high acc in end pos or pos_wait_mode
                pos_err_scale, rot_err_scale = 0.5, 0.5
            if (abs(step_len[0]) < (self.err_threshold[0]*pos_err_scale)) and \
            (abs(step_len[1]) < (self.err_threshold[1]*pos_err_scale)) and \
            (abs(step_len[2]) < (self.err_threshold[2]*rot_err_scale)):
                path_marker_index = path_marker_index + 1
                print ('path point done', path_marker_index, '/', marker_num)
                if path_marker_index >= marker_num:
                    self.bodyhub.wait_walking_done()
                    break
            if abs(step_len[2]) > 20:
                rot_adjust = True
            else:
                rot_adjust = False

            for i in range(3):
                step_len[i] = STEP_LEN_MAX[i] if step_len[i] > STEP_LEN_MAX[i] else step_len[i]
                step_len[i] = -STEP_LEN_MAX[i] if step_len[i] < -STEP_LEN_MAX[i] else step_len[i]
            if rot_adjust:
                self.__gait_cmd_pub.publish(data=[0.0, 0, step_len[2]])
            else:
                self.__gait_cmd_pub.publish(data=step_len)
            self.pose_update = False
        self.bodyhub.wait_walking_done()
 
    
    def set_head_rot(self, head_rot):
        keyframes = [
            ([0, -1, 16, -34, -17, -1, 0, 1, -16, 34, 17, 1, 0, -70, -15, 0, 70, 15, 0, 0, head_rot[0], head_rot[1]], 500, 0)
        ]
        self.movement.linearMove(keyframes)
        self.head_pitch = head_rot[1]

    def stand_straight(self, head_rot):
        keyframes = [
            ([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -70, -15, 0, 70, 15, 0, 0, head_rot[0], head_rot[1]], 500, 0)
        ]
        self.movement.linearMove(keyframes)
        self.head_pitch = head_rot[1]

    def frame_action(self,key_frame):
        try:
            client_action.custom_action([],key_frame)
        except Exception as err:
            serror(err)
        finally:
            pass 

    def turn_around(self,d_right=False,step=14):
        self.bodyhub.walking_n_steps([0,0,10 if d_right else -10],step)


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

    
    