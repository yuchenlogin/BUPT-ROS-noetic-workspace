#!/usr/bin/env python
# -*- coding: utf-8 -*-

import subprocess
import sys
import os
import math
import time
import threading
import copy
import yaml
import numpy as np
try:
    import scipy.linalg as linalg
except ImportError:
    print(sys.exc_info())
    os.system("pip install scipy==1.2.2")
    import scipy.linalg as linalg
import rospy
import rospkg
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from MapGenerator import MapGenerator
SCRIPTS_PATH=os.path.split(sys.argv[0])[0]
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
from lejulib import terminate
import motion.bodyhub_client as bodycli
class MarkerInfo(object):
    def __init__(self, _debug=False):
        rospy.Subscriber("/chin/visualization_marker_chin", Marker, self.chin_marker_callback)
        rospy.Subscriber("/head/visualization_marker_head", Marker, self.head_marker_callback)
        self.max_id = 20
        self.chin_marker_info = [{'valid': False, 'pos': [0, 0, 0], 'rot':[0, 0, 0]} for i in range(20)]
        self.head_marker_info = [{'valid': False, 'pos': [0, 0, 0], 'rot':[0, 0, 0]} for i in range(20)]
        self.multi_marker_info = {'valid': False, 'pos': [0, 0, 0], 'rot':[0, 0, 0], "id":0}
        self.head_rot = [0, 0, 0]
        self.chin_marker_lock = threading.Lock()
        self.head_marker_lock = threading.Lock()
        self.multi_marker_lock = threading.Lock()
        self.__debug_print = _debug

    def set_debug_print(self, _debug):
        self.__debug_print = _debug

    def set_head_rot(self, rot):
        self.head_rot = rot

    def quart_to_rpy(self, w, x, y, z):
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
        return [r, p, y]

    def rotate_mat(self, axis, radian):
        rot_matrix = linalg.expm(np.cross(np.eye(3), axis / linalg.norm(axis) * radian))
        # np.eye(3) 三维对角阵
        # cross 向量叉乘。
        return rot_matrix

    def chin_marker_callback(self, msg):
        if msg.id >= self.max_id:
            rospy.logwarn("unavailable marker id")
            return
        rpy = self.quart_to_rpy(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        rot = [-rpy[1] * 180.0 / math.pi, rpy[0] * 180.0 / math.pi, -rpy[2] * 180.0 / math.pi]
        cam_rot = self.rotate_mat([0, 1, 0], -(self.head_rot[1] - 10) * math.pi / 180.0)  # 绕y轴旋转
        pos_in_torso = np.dot(cam_rot, np.array([[-msg.pose.position.y], [-msg.pose.position.x], [msg.pose.position.z]])).tolist()
        pos = [pos_in_torso[0][0], pos_in_torso[1][0], pos_in_torso[2][0]]
        if msg.ns =="main_shapes":
            self.multi_marker_lock.acquire()
            self.multi_marker_info = {'valid': True, 'pos': pos, 'rot':rot, "id":msg.id}
            self.multi_marker_lock.release()
        else:
            self.chin_marker_lock.acquire()
            self.chin_marker_info[msg.id]['rot'] = rot
            self.chin_marker_info[msg.id]['pos'] = pos
            self.chin_marker_info[msg.id]['type'] = msg.ns
            self.chin_marker_info[msg.id]['valid'] = True
            self.chin_marker_lock.release()#获得机器人坐标系下标签的位置

        if self.__debug_print == True and msg.ns == "main_shapes":
            print ("chin marker:{}\n type:{}\n pos:{}\n rot:{}\n\n".format(msg.id, msg.ns, pos, rot))

    def head_marker_callback(self, msg):
        rpy = self.quart_to_rpy(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        self.head_marker_lock.acquire()
        self.head_marker_info[msg.id]['pos'] = [msg.pose.position.z, -msg.pose.position.x, -msg.pose.position.y]
        self.head_marker_info[msg.id]['rot'] = [rpy[2] * 180.0 / math.pi, rpy[0] * 180.0 / math.pi, -rpy[1] * 180.0 / math.pi]

        cam_rot = self.rotate_mat([0, 1, 0], -(self.head_rot[1]) * math.pi / 180.0)  # 绕y轴旋转
        pos_in_torso = np.dot(cam_rot, np.array([[msg.pose.position.z], [-msg.pose.position.x], [-msg.pose.position.y]])).tolist()
        self.head_marker_info[msg.id]['pos'] = [pos_in_torso[0][0], pos_in_torso[1][0], pos_in_torso[2][0]]

        self.head_marker_info[msg.id]['valid'] = True
        self.head_marker_lock.release()

        if self.__debug_print == True:
            print ("head marker:",  msg.id, self.head_marker_info[msg.id])
            
    def get_multi_marker_pos(self):
        self.multi_marker_lock.acquire()
        tmp = copy.deepcopy(self.multi_marker_info)
        self.multi_marker_info["valid"] = False
        self.multi_marker_lock.release()
        return tmp

    def get_current_pos(self,timeout = 10):
        cur_data=None
        stime = time.time()
        self.multi_marker_info["valid"] = False
        while not rospy.is_shutdown():
            self.multi_marker_lock.acquire()
            cur_data = copy.deepcopy(self.multi_marker_info)
            self.multi_marker_lock.release()
            if cur_data["valid"]:
                self.multi_marker_info["valid"] = False
                break
            else:
                time.sleep(0.01)
            if time.time() -stime > timeout:
                rospy.logwarn("获取artag码位置超时!")
                break
        return cur_data
    
    def get_chin_marker_pose(self, mk_id):
        self.chin_marker_lock.acquire()
        temp = copy.deepcopy(self.chin_marker_info[mk_id])
        self.chin_marker_info[mk_id]['valid'] = False
        self.chin_marker_lock.release()
        return temp

    def get_head_marker_pose(self, mk_id):
        self.head_marker_lock.acquire()
        temp = copy.deepcopy(self.head_marker_info[mk_id])
        self.chin_marker_info[mk_id]['valid'] = False
        self.head_marker_lock.release()
        return temp
    
class MarkerLocation(MarkerInfo):
    def __init__(self, body_client, _debug=False):
        MarkerInfo.__init__(self, _debug)
        self.bodyhub = body_client
        self.marker_err_threshold = [0.02, 0.03, 3.0]

    def goto_rot(self, mk_id,  goal_rot):
        while not rospy.is_shutdown():
            self.get_chin_marker_pose(mk_id)
            time.sleep(0.4)
            marker_pose = self.get_chin_marker_pose(mk_id)
            if marker_pose['valid']:
                err = marker_pose['rot'][2] - goal_rot
                if (abs(err) < self.marker_err_threshold[2]):
                    break
                theta = err * 0.9
                self.bodyhub.walking_the_distance(0.0, 0.0, theta)
                self.bodyhub.wait_walking_done()
            else:
                rospy.logwarn('%d marker no found!', mk_id)
                self.bodyhub.walking_n_steps([0.0, 0.0, 10.0], 2)
                self.bodyhub.wait_walking_done()

    def goto_pose(self, mk_id, goal_pose):
        while not rospy.is_shutdown():
            self.get_chin_marker_pose(mk_id)
            time.sleep(0.4)
            marker_pose = self.get_chin_marker_pose(mk_id)
            print("mark_p",marker_pose)
            if marker_pose['valid']:
                x_err = marker_pose['pos'][0] - goal_pose[0]
                y_err = marker_pose['pos'][1] - goal_pose[1]
                a_err = marker_pose['rot'][2] - goal_pose[2]
                if (abs(x_err) < self.marker_err_threshold[0]) and (abs(y_err) < self.marker_err_threshold[1]) and (abs(a_err) < self.marker_err_threshold[2]):
                    break
                x_len = x_err * 0.5
                y_len = y_err * 0.6
                a_len = a_err * 0.5
                self.bodyhub.walking_the_distance(x_len, y_len, a_len)
                self.bodyhub.wait_walking_done()
            else:
                rospy.logwarn('%d marker no found!', mk_id)
                self.bodyhub.walking_n_steps([-0.04, 0.0, 0.0], 1)
                self.bodyhub.wait_walking_done()

class MultiMarkerLocation(MarkerLocation):
    def __init__(self, body_client, _debug=False):
        MarkerLocation.__init__(self, body_client, _debug)
        self.multiLauncher = None
        self.__step_len_max = [0.1, 0.05, 10.0]  # x, y ,theta
        self.marker_err_threshold = [0.025, 0.03, 5.0]
        
    def limit_step(self,steplen):
        new_step =copy.deepcopy(steplen) 
        for i,va in enumerate(steplen):
            if abs(self.__step_len_max[i])<abs(va):
                new_step[i] = abs(va)/va*self.__step_len_max[i]
        return new_step
    
    def generate_map(self,pointsdict,path= os.path.join(SCRIPTS_PATH,"launch/MultiMarkerMap.xml"),marker_len=5):
        generator = MapGenerator(marker_len)
        xmlstr = generator.points2map(pointsdict)
        with open(path,"w") as f:
            xmlstr.encode("utf-8")
            f.write(xmlstr)
        print("generate map to {}".format(path))
        
    def multiMarkerstart(self,lanuch_map = os.path.join(SCRIPTS_PATH,"launch/roban_chin_camera_multi.launch")):
        cmd = "roslaunch {}".format(lanuch_map)
        self.multiLauncher = subprocess.Popen("{}".format(cmd), shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("multiMarker start")
            
    def stopMultiMarker(self,timeout=5):
        os.system("rosnode kill /chin/ar_track_alvar")
        starttime=time.time()
        while self.multiLauncher is not None and self.multiLauncher.poll() is None:
            if time.time() - starttime>timeout:
                print("force kill ar_track_alvar!")
                os.system("ps -aux | grep ar_track_alvar | awk '{{print $2}}' | xargs kill")
                break
            
    def goto_multi_rot(self,  goal_rot):
        while not rospy.is_shutdown():
            self.get_multi_marker_pos()
            time.sleep(0.4)
            marker_pose = self.get_multi_marker_pos()
            if marker_pose['valid']:
                err = self.calculate_diff_angle(marker_pose['rot'][2],goal_rot)
                
                print("\nmark_p:\ncur_rot{}\ngoal_pos{}\nerr{}".format( marker_pose['rot'][2],goal_rot,[err]))
                if (abs(err) < self.marker_err_threshold[2]):
                    break
                theta = err * 0.5
                x_len, y_len, theta = self.limit_step([0, 0, theta])
                print(x_len, y_len, theta)
                self.bodyhub.walking(0.0, 0.0, theta)
                self.bodyhub.wait_walking_done()
            else:
                rospy.logwarn('multi marker no found!')
                self.bodyhub.walking_n_steps([-0.04, 0.0, 0.0], 1)
                self.bodyhub.wait_walking_done()
               
    def calculate_diff_angle(self,cur_angle,goal_angle):
        err = cur_angle - goal_angle
        if err >180:
            err -= 360
        elif err < -180:
            err += 360
        return err
    
    def goto_pos_nearby(self,goal_pose):
        while not rospy.is_shutdown():
            self.get_multi_marker_pos()
            time.sleep(0.4)
            marker_pose = self.get_multi_marker_pos()
            if marker_pose['valid']:
                cur_pos = [marker_pose['pos'][0],marker_pose['pos'][1],marker_pose['rot'][2]]
                x_err = cur_pos[0] - goal_pose[0]
                y_err = cur_pos[1] - goal_pose[1]
                # if x_err**2 + y_err**2 > 0.05**2:#当相隔太远时
                a_err = (math.atan2(y_err, x_err) * 180.0 / math.pi)
                # else:    
                #     a_err = cur_pos[2] - goal_pose[2]
                print("\nmark_p:\ncur_pos{}\ngoal_pos{}\nerr{}".format(cur_pos,goal_pose,[x_err,y_err,a_err]))
                if (abs(x_err) < self.marker_err_threshold[0]*2) and (abs(y_err) < self.marker_err_threshold[1]*2):
                    break
                
                if abs(a_err) > 20:
                    x_err = y_err = 0
                x_len = x_err * 0.55
                y_len = y_err * 0.6
                a_len = a_err * 0.5
                print(x_len, y_len, a_len)
                x_len, y_len, a_len = self.limit_step([x_len, y_len, a_len])
                print(x_len, y_len, a_len)
                self.bodyhub.walking(x_len, y_len, a_len)
                self.bodyhub.wait_walking_done()
            else:
                rospy.logwarn('multi marker no found!')
                self.bodyhub.walking_n_steps([-0.04, 0.0, 0.0], 1)
                self.bodyhub.wait_walking_done()
                
    def goto_multi_pose(self, goal_pose):
        self.goto_pos_nearby(goal_pose)
        self.goto_multi_rot(goal_pose[2])
        while not rospy.is_shutdown():
            self.get_multi_marker_pos()
            time.sleep(0.4)
            marker_pose = self.get_multi_marker_pos()
            if marker_pose['valid']:
                cur_pos = [marker_pose['pos'][0],marker_pose['pos'][1],marker_pose['rot'][2]]
                x_err = cur_pos[0] - goal_pose[0]
                y_err = cur_pos[1] - goal_pose[1]
                rospy.logwarn("{} : {}".format(x_err**2 + y_err**2,0.1**2))
                a_err = self.calculate_diff_angle(cur_pos[2], goal_pose[2])
                
                print("\nmark_p:\ncur_pos{}\ngoal_pos{}\nerr{}".format(cur_pos,goal_pose,[x_err,y_err,a_err]))
                if (abs(x_err) < self.marker_err_threshold[0]) and (abs(y_err) < self.marker_err_threshold[1]) and (abs(a_err) < self.marker_err_threshold[2]):
                    break
                
                if abs(a_err) > 20:
                    x_err = y_err = 0
                x_len = x_err * 0.4
                y_len = y_err * 0.6
                a_len = a_err * 0.5
                print(x_len, y_len, a_len)
                x_len, y_len, a_len = self.limit_step([x_len, y_len, a_len])
                print(x_len, y_len, a_len)
                self.bodyhub.walking(x_len, y_len, a_len)
                self.bodyhub.wait_walking_done()
            else:
                rospy.logwarn('multi marker no found!')
                self.bodyhub.walking_n_steps([-0.04, 0.0, 0.0], 1)
                self.bodyhub.wait_walking_done()
                
        
if __name__ == '__main__':
    rospy.init_node("artag_tester", anonymous=False) # 实例化aiui节点
    t = MultiMarkerLocation(None,True)
    SCRIPTS_PATH=os.path.split(sys.argv[0])[0]
    with open(os.path.join(SCRIPTS_PATH,"config.yaml"),"r")as f:
        config = yaml.safe_load(f)
        print(config)
        
    points = config["MAP_POINTS"]
    t.generate_map(points)
    t.multiMarkerstart()
    # time.sleep(2)
    # t.stopMultiMarker()
    rospy.spin()
