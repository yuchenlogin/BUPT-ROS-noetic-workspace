#!/usr/bin/env python
# -*- coding: utf-8 -*-.
import rospkg
import bodyhub_action as bodyact
import time
from std_msgs.msg import *
import threading

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *

class march_on_the_spot(bodyact.Action):
    def __init__(self, body_client):
        bodyact.Action.__init__(self, body_client)
        self.bodyhub = body_client

    def march_on_the_spot(self):
        global timeout
        while not timeout:
            self.bodyhub_walk()
            self.bodyhub.walking(0, 0, 0)

    def pause(self):
        global timeout
        while True:
            time.sleep(walk_time)
            timeout = True
            time.sleep(stand_time)
            timeout = False
    
if __name__ == '__main__':
    from motion import bodyhub_client as bodycli
    walk_time = 60      # 原地踏步持续时间
    stand_time = 60     # 原地站立持续时间

    try:
        node_initial()

        timeout = False
        obj = march_on_the_spot(bodycli.BodyhubClient(2))
        
        t = threading.Thread(target=obj.pause)
        t.start()

        while True:
            obj.march_on_the_spot()

    except Exception as err:
        serror(err)
