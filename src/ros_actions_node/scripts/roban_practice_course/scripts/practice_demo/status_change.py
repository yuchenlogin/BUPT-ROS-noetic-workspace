#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time

import rospy
import rospkg

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
from motion.motionControl import SetBodyhubTo_walking, WaitForWalkingDone, SetBodyhubTo_setStatus
import time
import sys
from lejufunc import client_action
import os
import motion.bodyhub_client as bodycli
import NXIUGAI0429


if __name__ == '__main__':
    rospy.init_node("status_change")
    bodyhub = bodycli.BodyhubClient(2)
    
    SetBodyhubTo_walking(2)

    # 前进
    bodyhub.walking_the_distance(0.20, 0, 0)
    bodyhub.wait_walking_done()
    
    # 执行动作
    NXIUGAI0429.kick_redball()

    # 切换状态
    bodyhub.ready()
    bodyhub.walk()

    # 继续行走
    bodyhub.walking_the_distance(0.20, 0.02, 30)
    bodyhub.wait_walking_done()

    # 释放bodyhub占用
    bodyhub.reset()

    sys.exit()