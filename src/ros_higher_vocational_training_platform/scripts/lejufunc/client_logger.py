#!/usr/bin/env python
# coding=utf-8

import threading
import rospy
import rospkg
import rosservice
from ros_msg_node.srv import *
import os
import sys
from std_msgs.msg import String
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg')+"/src/")
import motion.bodyhub_client as bodycli
finish_pub = rospy.Publisher("/Finish",String,queue_size = 2)
finishend_lock = threading.Lock()

def service_alive(process_service):
	service_list = rosservice.get_service_list()
	if process_service in service_list:
		return True
	else:
		return False

def wait_for_processMsgservice():
	if service_alive("/ros_msg_node/process_msg_process"):
		rospy.wait_for_service("/ros_msg_node/process_msg_process")
		client = rospy.ServiceProxy("/ros_msg_node/process_msg_process", ProcessMsg)
		return client
	else:
		rospy.logerr("serevice not alive")
		return False


def sprint(*msg):
	print_msg = ""
	client = wait_for_processMsgservice()
	if client:
		for i in msg:
			if not isinstance(i,str):
				i = str(i)
			print_msg = print_msg + i + " "
		client("print",print_msg)

def sdebug(debug_msg):
	if rospy.is_shutdown():
		return
	client = wait_for_processMsgservice()
	if client:
		client("debug",debug_msg)

def sinfo(info_msg):
	if rospy.is_shutdown():
		return
	client = wait_for_processMsgservice()
	if client:
		client("info",info_msg)

def swarn(warn_msg):
	if rospy.is_shutdown():
		return
	client = wait_for_processMsgservice()
	if client:
		client("warn",warn_msg)

def serror(error_msg):
	rospy.logerr(error_msg)
	if rospy.is_shutdown():
		return
	client = wait_for_processMsgservice()
	if client:
		client("error",str(error_msg))

def reset_bodyhub(control_id=2):
    bodycli.BodyhubClient(control_id).reset()

def finishsend():
	if finishend_lock.locked():
		return
	finishend_lock.acquire()
	rospy.logwarn("Finished sending")
	os.system('ps -aux | grep -w play | awk \'{print $2}\' | xargs kill -9')

	if rospy.is_shutdown():
		finishend_lock.release()
		return
	reset_bodyhub()
	if service_alive("/ros_msg_node/process_msg_process"):
		finish_pub.publish("finish")
		rospy.wait_for_service("/ros_msg_node/finish_msg_process", 2)
		client = rospy.ServiceProxy("/ros_msg_node/finish_msg_process", RunningFinish)
		client()
	finishend_lock.release()
