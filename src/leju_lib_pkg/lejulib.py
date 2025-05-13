#!/usr/bin/env python
# coding=utf-8

import os
import sys
import signal
import rospy
import rospkg
import rosservice

from std_msgs.msg import String
from ros_actions_node.msg import DeviceList
from ros_msg_node.srv import *

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg')+"/src/")
import motion.bodyhub_client as bodycli
from motion.motionControl import SetBodyhubTo_setStatus

from lejufunc import client_action
from lejufunc import client_face
from lejufunc import client_color
from lejufunc import client_audio
from lejufunc import client_video
from lejufunc import client_sensor
from lejufunc import client_controller
from lejufunc import client_wakeup
from lejufunc import client_speech
from lejufunc import client_gripper
from lejufunc import client_button
from lejufunc import client_walk
from lejufunc import client_label
from lejufunc import slam
from lejufunc.client_logger import *
from lejufunc import bezier

finish_pub = rospy.Publisher("/Finish",String,queue_size = 2)
device_pub = rospy.Publisher("/ActRunner/DeviceList", DeviceList, queue_size=2)


# terminate current process
def terminate(data):
	rospy.loginfo(data.data)
	bezier.stop_SendJointCommand = True
	rospy.signal_shutdown("kill")


# check device list in demo
def report_device_status(camera_status, controller_status):
	device_pub.publish(camera_status, controller_status)

def set_status(control_id=2):
    SetBodyhubTo_setStatus(control_id)
     
def node_initial(name = "act_runner", anonymous = True, stateJump = True):
	# initial node
	rospy.init_node(name, anonymous=anonymous)
	rospy.sleep(0.2)
	rospy.on_shutdown(finishsend)
	rospy.Subscriber('terminate_current_process', String, terminate)
	if stateJump:
		set_status()
	rospy.loginfo("action node running")
	
 
if __name__ == '__main__':
	node_initial()
