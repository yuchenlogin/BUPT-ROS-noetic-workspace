#!/usr/bin/env python
# coding=utf-8
import os
import rospy
from lejulib import *
import rosnode

MAIN_NODE = "/ros_sound_source_localization"

def rosShutdownHook():
    os.system("rosnode kill {}".format(MAIN_NODE))

def main():
    rospy.init_node("run_sound_source_localization")
    rospy.on_shutdown(rosShutdownHook)
    rospy.Subscriber('terminate_current_process', String, terminate)

    try:
        os.system(". ~/robot_ros_application/catkin_ws/devel/setup.sh")
        os.system("roslaunch ros_sound_source_localization sound_source_localization.launch")
        rospy.spin()

    except Exception as err:
        serror(err)

if __name__ == '__main__':
    main()
