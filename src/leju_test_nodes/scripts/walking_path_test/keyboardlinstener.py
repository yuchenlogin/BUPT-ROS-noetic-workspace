#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import math
import termios
import time
import rospkg
from std_msgs.msg import *
from geometry_msgs.msg import *
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import tty
import select

class keyboardlinstener(object):
    def __init__(self):
        super(keyboardlinstener,self).__init__()
        self.key_val = ""
        self.update = False

    def getKey(self, key_timeout):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = "None"
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
