#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import rospkg
import bodyhub_action as bodyact
from std_msgs.msg import String
from motion import bodyhub_client as bodycli

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *

def terminate(data):
    rospy.loginfo(data.data)
    rospy.signal_shutdown("kill")

class CheckGait(bodyact.Action):
    def __init__(self, body_client):
        bodyact.Action.__init__(self, name="check_gait_node", ctl_id=2, init_node=False)
        self.bodyhub = body_client
        rospy.Subscriber('terminate_current_process', String, terminate)

    def start(self):
        self.bodyhub_walk()
        # self.bodyhub.walking_n_steps([0.00, 0.0, 0.0], 3)
        self.bodyhub.walking_n_steps([0.08, 0.0, 0.0], 9)
        self.bodyhub.wait_walking_done()

if __name__ == '__main__':
    obj = CheckGait(bodycli.BodyhubClient(2))
    obj.start()
