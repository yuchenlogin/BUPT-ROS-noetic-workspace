#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import time
from ik_lib.ikmodulesim import IkModuleSim
from ik_lib.ikmodulesim.CtrlType import CtrlType as C

class Centroid_controller(IkModuleSim):
    def __init__(self):
        super(Centroid_controller, self).__init__()
        self.running=False
        
        self.toInitPoses()
        
    def squa_and_stand(self,times=1):
        self.running=True
        try:
            for i in range(times):
                self.body_motion(C.Torso, [0, 0, 0, 0, 0.0, -0.08])  
                self.body_motion(C.Torso, [0, 0, 0, 0, 0.0, 0.08]) 
                if not self.running:
                    break
        except:
            print(sys.exc_info(),28)

    def on_shutdown(self):
        self.running = False
        self.waitPostureDone()
        self.reset()

if __name__ == '__main__':
    times=1
    if len(sys.argv)>1 and sys.argv[1].isdigit():
        times=int(sys.argv[1])
    rospy.init_node('squa_and_stand', anonymous=True)
    controller=Centroid_controller()
    rospy.on_shutdown(controller.on_shutdown)
    controller.squa_and_stand(times)
