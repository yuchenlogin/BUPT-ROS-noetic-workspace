#!/usr/bin/env python
# coding=utf-8
import os
import rospy
from lejulib import *
import rosnode

JY901_NODE = "/jy901Module_node"
ANTI_DISTURBANCE_NODE = "/anti_disturbance_node"

def rosShutdownHook():
    os.system("rosnode kill {}".format(ANTI_DISTURBANCE_NODE))
    os.system("rosnode kill {}".format(JY901_NODE))

def main():
    rospy.init_node("anti_action_node")
    rospy.on_shutdown(rosShutdownHook)
    rospy.Subscriber('terminate_current_process', String, terminate)

    try:
        os.system(". ~/robot_ros_application/catkin_ws/devel/setup.sh")
        if rosnode.rosnode_ping(JY901_NODE, 1) == True:
            os.system("rosrun walking_controller walking_controller_node &")
            pass
        else:
            os.system("roslaunch walking_controller anti_disturbance.launch &")
            pass
        rospy.spin()

    except Exception as err:
        serror(err)

if __name__ == '__main__':
    main()
