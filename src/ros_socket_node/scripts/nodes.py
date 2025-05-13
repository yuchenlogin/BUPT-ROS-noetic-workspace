#!/usr/bin/env python
# -*- coding: utf-8 -*-
import subprocess
import sys
if sys.version<'3':
    reload(sys) 
    sys.setdefaultencoding( "utf-8" )
from sys import path
import os
import rospy
import array
import signal

from std_msgs.msg import UInt16
from std_msgs.msg import String
from bodyhub.msg import JointControlPoint
from bodyhub.srv import *
from actexecpackage.srv import *
from ros_msg_node.srv import ProcessMsg
import rospkg
import time
from  threading import Thread
SOURCE_PATH = '. /home/lemon/robot_ros_application/catkin_ws/devel/setup.bash'

MASTER_ID = 2
class Runner():
    def __init__(self, node_file = None, logout = False):
        self.__path = node_file
        self.p = None
        self.__logout = logout
        if logout:
            log_path = "/home/lemon/.ros/socket_runner/"
            if not os.path.exists(log_path):
                os.system("mkdir -p /home/lemon/.ros/socket_runner")
            self.__logfile = log_path + os.path.basename(node_file).split(".")[0] + ".log" if node_file is not None else "None"

            

    def start(self):
        log = "> '{}' 2>&1".format(self.__logfile) if self.__logout else ""
        cmd = "python '{}' {}".format(self.__path, log)
        self.p = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
    def is_running(self):
        return True if self.p and self.p.poll() is None else False
    
    def read_log(self):
        if self.__logout:
            return subprocess.check_output("cat '{}'".format(self.__logfile), shell=True)
        return "LOGOUT not set"
        
    def get_demo_path(self):
        return self.__path

    def stop(self):
        if not self.is_running():
            return
        print("force killed demo process")
        os.killpg(os.getpgid(self.p.pid), signal.SIGKILL)



class Node(object):
    def __init__(self):
        self.masterID = rospy.get_param("masterID", MASTER_ID)
        self.walkflag = False

        self.ros_app_dir = rospkg.RosPack().get_path("ros_actions_node") + "/scripts/"

        self.srv_master_node = '/MediumSize/BodyHub/GetMasterID'
        self.srv_current_status = '/MediumSize/BodyHub/GetStatus'
        self.srv_body_state = '/MediumSize/BodyHub/StateJump'
        self.error_process_srv = '/ros_msg_node/process_msg_process'
        self.buttonPub = rospy.Publisher('/VirtualPanel', String, queue_size=2)
        self.terminate_pub = rospy.Publisher('terminate_current_process', String, queue_size=10)
        self.runner=Runner()

    def _srv_client(self, srv_name, args_fmt):
        """ros service client function

        :param srv_name:
        :param args_fmt:
        :return:
        """
        print("rosservice call {}".format(srv_name))
        rospy.wait_for_service(srv_name)
        client = rospy.ServiceProxy(srv_name, args_fmt)
        return client


    def get_master_id(self):
        """get current master id of bodyhub node
        """
        exec_client = self._srv_client(self.srv_master_node, SrvTLSstring)
        master_id = exec_client('Get master id').data
        return master_id

    def node_run(self, node_file, logout = False):
        """run script

        :param node_file:
        :return:
        """
        result = os.system("chmod +x '{}'".format(node_file))
        if result == 0:
            self.runner = Runner(node_file, logout = logout)
            self.runner.start()
            # Thread(target=os.system, args=[node_file]).start()
            # self.status_info.is_running = True
            # self.status_info.path = node_file
        else:
            error_client = self._srv_client(self.error_process_srv, ProcessMsg)
            error_client("error","Permission denied, can't execute download file")
        return None

    def node_force_stop(self):
        self.runner.stop()
    
    @property
    def is_executing(self):
        return self.runner.is_running()


    def node_break(self):
        """stop script by force

        :return:
        """
        self.masterID_guarantee()
        current_state = self.get_current_status()
        if current_state == "error":
            rospy.logerr("bodyhub state error")
            raise
        elif current_state in ["walking", "running", "pause"]:
            self.state_jump("stop")
            self.wait_for_ready()
        elif current_state == "directOperate":
            self.state_jump("reset")
            self.state_jump("setStatus")


    def set_act_state(self, state):
        """set bodyhub status by force

        :param state:
        :return:
        """
        self.node_break()
        act_client = self._srv_client(self.srv_body_state, SrvState)
        act_client(self.masterID, state)
        print('set bodyhub', state)
        return None

    def list_app(self):
        app_list = []
        for file in sorted(os.listdir(self.ros_app_dir)):    ## 使用sorted函数对文件按首字母进行排序
            if file == 'lejulib.py':
                continue
            if file.endswith('.py'):
                app_list.append(os.path.join(self.ros_app_dir, file))
        return app_list

    def walk_node_stop(self):
        """stop walking
        """
        exec_client = self._srv_client(self.srv_body_state, SrvState)
        exec_client(self.masterID, 'stop')
        return None

    def walk_node_reset(self):
        """release occupied bodyhub node from walking mode
        """
        exec_client = self._srv_client(self.srv_body_state, SrvState)
        exec_client(self.masterID, 'reset')
        return None


    def get_current_status(self):
        """get bodyhub node current status
        :return:
        """
        status_client = self._srv_client(self.srv_current_status,SrvString)
        status = status_client('current_status').data
        return status

    def state_jump(self, state, masterID=None):
        """
        :param state:
        :return:
        """
        if not masterID:
            masterID = self.masterID
        state_client = self._srv_client(self.srv_body_state, SrvState)
        state_client(masterID, state)


    def wait_for_ready(self):
        """
        :return:
        """
        while self.get_current_status() != "ready":
            time.sleep(0.01)


    def masterID_guarantee(self):
        """  guarantee masterID : self.masterID
        :return:
        """
        current_mssterID = self.get_master_id()
        if current_mssterID != self.masterID:
            if current_mssterID == 0:
                self.state_jump("setStatus")
            else:
                current_state = self.get_current_status()
                if current_state in ["walking", "running", "pause"]:
                    self.state_jump("stop", current_mssterID)
                    self.wait_for_ready()
                self.state_jump("reset", current_mssterID)
                self.state_jump("setStatus")





