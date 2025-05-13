#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time

import rospy
import rospkg

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.motionControl as mCtrl
import motion.TrajectoryPlan as tPlan
import motion.bodyhub_client as bClient
sys.path.append(rospkg.RosPack().get_path('ros_actions_node'))
from scripts.action.demo_frame import DemoFrame as DF

import termios
import tty

NodeControlId = 2
tpObject = tPlan.TrajectoryPlanning(22,10.0)
BODYHUB_WALKING = bClient.BodyhubClient(NodeControlId)
idList = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22]
angleList = [[0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0, 0,0,0, 0,0, 0,0],1000,0]

def SendTrajectory(controlId, trajectoryPoint):
    for m in range(len(trajectoryPoint[0])):
        jointPosition = []
        for n in range(len(trajectoryPoint)):
            jointPosition.append(trajectoryPoint[n][m].y)

        mCtrl.SendJointPsition(posList=jointPosition, controlId=controlId)

def actionExecute(poseList):
    angleList[0] = mCtrl.GetJointAngle(idList)
    poseList.insert(0, angleList)

    tpObject.setInterval(poseList[0][1])
    tpObject.planningBegin(poseList[0][0], poseList[1][0])

    for poseIndex in range(2, len(poseList)):
        tpObject.setInterval(poseList[poseIndex-1][1])
        trajectoryPoint = tpObject.planning(poseList[poseIndex][0])
        SendTrajectory(NodeControlId, trajectoryPoint)
        if poseList[poseIndex-1][2] > 0:
            mCtrl.WaitForActionExecute(True)
            time.sleep(poseList[poseIndex-1][2]/1000.0)
        else:
            mCtrl.WaitForActionExecute()
            
    trajectoryPoint = tpObject.planningEnd()
    SendTrajectory(NodeControlId, trajectoryPoint)
    mCtrl.WaitForActionExecute(True)

def action(action_name):
    mCtrl.SetBodyhubTo_setStatus(NodeControlId)

    action_frames = action_name
    actionExecute(action_frames)

    mCtrl.ResetBodyhub()

def go_square():
    if mCtrl.SetBodyhubTo_walking(NodeControlId) == False:
        rospy.logerr('bodyhub to setStatus fail!')
        rospy.signal_shutdown('error')
        exit(1)

    move_forward = [0.1, 0.0, 0.0]
    move_forward_step = 8
    turn_right_in_place = [0.0, 0.0, -10.0]
    turn_right_in_place_step = 8
    
    for _ in range(4):
        BODYHUB_WALKING.walking_n_steps(move_forward, move_forward_step)
        BODYHUB_WALKING.walking_n_steps(turn_right_in_place, turn_right_in_place_step)

    mCtrl.WaitForWalkingDone()
    mCtrl.ResetBodyhub()

def rosShutdownHook():
    mCtrl.ResetBodyhub()

demo_bind_key = {'w': 'squat',
'a': 'squat_to_stand', 's': 'shake_head', 'd': 'head_toward_face',
'z': 'touch_leg', 'x': 'snicker', 'c': 'bow',
'h': 'anthropomorphic_speech', 'j': 'hug', 'k': 'bend_over', 'l': 'pick_up_trash',
'n': 'go_square',
'q': 'quit'
}

gait_examples = {'n': go_square}

def output_demoinfo(key):
    return '{}--{}'.format(key, demo_bind_key[key])

def demoInfo():
    print("\n%-30s%s" % ('', output_demoinfo('w')))
    print("%-30s%-30s%-30s" % (output_demoinfo('a'), output_demoinfo('s'), output_demoinfo('d')))
    print("%-30s%-30s%-30s" % (output_demoinfo('z'), output_demoinfo('x'), output_demoinfo('c')))
    print("%-30s%-30s%-30s%-30s" % (output_demoinfo('h'), output_demoinfo('j'), output_demoinfo('k'), output_demoinfo('l')))
    print("%-30s%s" % ('', output_demoinfo('n')))
    print("%-30s%s" % (' ', output_demoinfo('q')))
    print("")

def getch(str=''):
    print(str)
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    print(ch)
    return ch

def keyboard_poll():
    while not rospy.is_shutdown():
        cmd = getch('key:')
        if cmd == 'q':
            return
        elif cmd in gait_examples:
            gait_examples[cmd]()
        elif cmd in demo_bind_key:
            action_frames = getattr(DF, demo_bind_key[cmd])
            action(action_frames)

def main():
    demoInfo()
    keyboard_poll()

if __name__ == '__main__':
    rospy.init_node('demo_lessons_plan', anonymous=True)
    main()
        