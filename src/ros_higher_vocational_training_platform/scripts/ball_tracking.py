#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import os
import cv2
import signal
if sys.version>'3':
    import queue as Queue
else:
    import Queue 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import array
import time
import threading
import numpy as np
from bodyhub.srv import *  # for SrvState.srv
from bodyhub.msg import JointControlPoint
from lejulib import *
import motion.bodyhub_client as bodycli
import vision.imageProcessing as imgPrcs
import algorithm.pidAlgorithm as pidAlg
import rospkg
from motion.motionControl import ResetBodyhub

SERVO = client_action.SERVO
QUEUE_IMG = Queue.Queue(maxsize=2)
bridge = CvBridge()

# Horizontal and vertical limit value
H_limit=70
V_limit=25

# fps threshold:Only when the frame rate is greater than this value, the head actuator will work
FPS_Threshold=20

# HSV阈值
lowerRed = np.array([0, 160, 150])
upperRed = np.array([50, 220, 200])


class Action(object):
    '''
    robot action
    '''

    def __init__(self, ctl_id):
        rospy.on_shutdown(self.__ros_shutdown_hook)

        self.bodyhub = bodycli.BodyhubClient(ctl_id)

    def __ros_shutdown_hook(self):
        if self.bodyhub.reset() == True:
            rospy.loginfo('bodyhub reset, exit')
        else:
            rospy.loginfo('exit')

    def bodyhub_ready(self):
        if self.bodyhub.ready() == False:
            rospy.logerr('bodyhub to ready failed!')
            rospy.signal_shutdown('error')
            time.sleep(1)
            exit(1)

    def bodyhub_walk(self):
        if self.bodyhub.walk() == False:
            rospy.logerr('bodyhub to walk failed!')
            rospy.signal_shutdown('error')
            time.sleep(1)
            exit(1)
            
class Ball_tracker(Action):
    def __init__(self, debug=False):
        super(Ball_tracker, self).__init__( 2)
        image_topic = '/camera/color/image_raw'
        rospy.Subscriber('terminate_current_process', String, terminate)
        self.__cv_bridge = CvBridge()
        self.img_origin = np.zeros((640, 480, 3), np.uint8)
        self.img_update=False
        self.running =False
        self.target_lost=True
        rospy.Subscriber(image_topic, Image, self.__image_callback)
        self.detect_fps=0
        self.img_size=[640,480]
        self.ball_pos=[320,240]
        self.detecter=Ball_detecter(self)
        self.head_joint_puber=HeadJointControl_Thread(self)
        self.debug =debug
    def start_tracking(self):
        print("Start tracking")
        self.running=True
        self.bodyhub_ready()
        self.detecter.start()
        self.head_joint_puber.start()
    def stop(self):
        self.running=False
        self.head_joint_puber.join()
        self.detecter.join()
        
    def __image_callback(self, msg):
        try:
            self.img_origin = self.__cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.img_size=[self.img_origin.shape[1],self.img_origin.shape[0]]
            self.img_update=True
        
        except CvBridgeError as err:
            rospy.logerr(err)

class Ball_detecter(threading.Thread):
    def __init__(self,parent):
        super(Ball_detecter, self).__init__()
        self.parent = parent
        self.__fps_time = 0
        self.__ball = imgPrcs.ColorObject(lowerRed, upperRed)

    def pre_process(self):
        return cv2.cvtColor(self.parent.img_origin,cv2.COLOR_BGR2HSV)

    def run(self):
        def getpos(event,x,y,flags,param):
            if event==cv2.EVENT_LBUTTONDOWN:
                print(HSV[y,x])#点击图片时返回hsv的值
        while not rospy.is_shutdown() and self.parent.running:
            if not self.parent.img_update:
                time.sleep(0.01)
                continue
            self.parent.img_update =False
            t0 = time.time()
            result=self.__ball.detection(self.parent.img_origin,minisize=150)
            if result['find'] != False:
                self.parent.target_lost=False
                self.parent.ball_pos=[result["Cx"],result["Cy"]]
                # print(result)
            else:
                self.parent.target_lost=True

            imgPrcs.putVisualization(self.parent.img_origin, result)
            t1 = time.time()
            fps = 1.0/(time.time() - self.__fps_time)
            self.parent.detect_fps=fps
            # rospy.logwarn("{} {}".format(fps,self.parent.target_lost))
            self.__fps_time = time.time()
            if self.parent.debug:
                HSV=self.pre_process()
                imgPrcs.putTextInfo(self.parent.img_origin, fps, (t1-t0)*1000)
                cv2.imshow("image window", self.parent.img_origin)
                cv2.setMouseCallback('image window',getpos)
                cv2.waitKey(1)
                
class HeadJointControl_Thread(threading.Thread):
    def __init__(self, parent):
        super(HeadJointControl_Thread, self).__init__()
        self.parent = parent
        self.__pid_x = pidAlg.PositionPID(p=0.005,d=0.001)
        self.__pid_y = pidAlg.PositionPID(p=0.005,d=0.001)
        self.__err_threshold = [20.0, 20.0]
        self.controlID= 2
        self.HeadJointPub = rospy.Publisher('MediumSize/BodyHub/HeadPosition', JointControlPoint, queue_size=100)
        self.update_ctlid()
        self.target_pos=[0,0]

    def run(self):
        self.set_head_servo([0,0])
        while (self.parent.running and not rospy.is_shutdown()) and (self.parent.detect_fps < FPS_Threshold or self.parent.target_lost):
            time.sleep(0.1)
        while self.parent.running  and not rospy.is_shutdown():
            errx=-self.parent.ball_pos[0]+self.parent.img_size[0]/2
            erry=self.parent.ball_pos[1]-self.parent.img_size[1]/2
            rspx=0
            rspy=0
            # print(self.parent.img_update,self.parent.target_lost,errx,erry)
            # 执行阈值
            if abs(errx)>self.__err_threshold[0]:
                rspx=self.__pid_x.run(errx)
            
            if abs(erry)>self.__err_threshold[1]:
                rspy=self.__pid_y.run(erry)

            if self.parent.target_lost:
                # print("target lost")
                time.sleep(0.1)
                continue   
            self.target_pos[0]+=rspx
            self.target_pos[1]+=rspy
            
            if self.target_pos[0]>=H_limit:
                self.target_pos[0]=H_limit
            elif self.target_pos[0]<=-H_limit:
                self.target_pos[0]=-H_limit
                
            if self.target_pos[1]>=V_limit:
                self.target_pos[1]=V_limit
            elif self.target_pos[1]<=-V_limit:
                self.target_pos[1]=-V_limit
                    
            # print(self.target_pos)
            self.set_head_servo(self.target_pos)
            time.sleep(0.01)
        # self.set_head_servo([0,0])
       
    def set_head_servo(self,angles):
        if self.parent.debug:
            rospy.logwarn("pub angles:{}".format(angles))
        if not rospy.is_shutdown():
            self.update_ctlid()
            self.HeadJointPub.publish(positions=angles, mainControlID=self.controlID)
        
    def update_ctlid(self):#get the newest Main_controlID
        try:
            rospy.wait_for_service('MediumSize/BodyHub/GetMasterID',1)
        except:
            print ('error: wait_for_service GetMasterID!',sys.exc_info())
            return 0
        client = rospy.ServiceProxy('MediumSize/BodyHub/GetMasterID', SrvTLSstring)
        response = client('get')
        self.controlID= response.data

def rosShutdownHook():
    tracker.stop()
    finishsend()
    ResetBodyhub() # reset bodyhub while on_shutdown,https://www.lejuhub.com/Talos/robot_ros_application/-/issues/978#note_821639
        
if __name__ == '__main__':
    debug= True if len(sys.argv) > 1 else False
    rospy.init_node("ball_tracking", anonymous=True)
    rospy.on_shutdown(rosShutdownHook)
    tracker=Ball_tracker(debug)
    tracker.start_tracking()
    rospy.spin()
 
