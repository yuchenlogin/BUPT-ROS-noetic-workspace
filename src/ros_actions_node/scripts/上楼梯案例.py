#!/usr/bin/env python
# coding=utf-8
import os
import rospy
from lejulib import *


def main():
    node_initial()

    try:
       os.system("cd ~/robot_ros_application/catkin_ws/src/ros_actions_node/scripts && ./run_stair.sh")

    except Exception as err:
        serror(err)

    finally:
        finishsend()

if __name__ == '__main__':
    main()
