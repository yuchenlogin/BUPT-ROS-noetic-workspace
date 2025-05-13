#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time

import rospy
import rospkg

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.motionControl as mCtrl


NodeControlId = 2

def rosShutdownHook():
    mCtrl.ResetBodyhub()

if __name__ == '__main__':
    print( 'node runing...')
    rospy.init_node('demo_walking_node', anonymous=True)
    time.sleep(0.2)
    rospy.on_shutdown(rosShutdownHook)

    while not rospy.is_shutdown():

        
        if mCtrl.SetBodyhubTo_walking(NodeControlId) == False:
            rospy.logerr('bodyhub to walking fail!')
            rospy.signal_shutdown('error')
            exit(1)
        
        # 侧移
        mCtrl.SendGaitCommand(0.0, -0.06, 0.0)
        mCtrl.WaitForWalkingDone()
        
        # 前进
        mCtrl.SendGaitCommand(0.1, 0.0, 0.0)
        mCtrl.WaitForWalkingDone()

        # 前进并旋转
        mCtrl.SendGaitCommand(0.1, 0.0, 10.0)
        mCtrl.WaitForWalkingDone()
        
        mCtrl.ResetBodyhub()   

        if mCtrl.SetBodyhubTo_setStatus(NodeControlId) == False:
            rospy.logerr('bodyhub to setStatus fail!')
            rospy.signal_shutdown('error')
            exit(1)

        mCtrl.ResetBodyhub()
        print("end")
        rospy.signal_shutdown('exit')
        