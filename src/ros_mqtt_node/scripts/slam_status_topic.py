#!/usr/bin/python3

import threading
import paho.mqtt.client as mqtt
import rospy
import rospkg
import json
import os
import time
from std_msgs.msg import UInt32MultiArray
from SLAM.srv import *
Topic_name="/slam_status"
class Slam_fp_pub(threading.Thread):
    def __init__(self,client):
        super(Slam_fp_pub, self).__init__()
        self.client = client
        self.slam_config_path=rospkg.RosPack().get_path('ros_mqtt_node')+"/config/slam_mapping.json"
        self.rate = rospy.Rate(4)

    def run(self):
        self.running= True
        rospy.loginfo("Slam_fp_pub starting")
        mtime=0
        config={}
        while self.running and not rospy.is_shutdown():
            try:
                sub_result = rospy.wait_for_message("/SLAM/FeaturePoint/Quantity", UInt32MultiArray, 2)
                feature_points_counts = sub_result.data
            except Exception as e:
                if "timeout" not in e.args[0]:
                    rospy.logwarn("cannot subscribe to message: /SLAM/FeaturePoint/Quantity {}".format(e))
                time.sleep(1)
                continue
            else:
                if os.path.getmtime(self.slam_config_path)!=mtime:
                    rospy.loginfo("reload config file")
                    mtime=os.path.getmtime(self.slam_config_path)
                    with open(self.slam_config_path,"r",encoding="utf8") as f:
                        config=json.load(f)

            status="unknown"
            mapping_mode=True
            progress=-1
            try:
                rospy.wait_for_service('/SLAM/slam_status', 2)
                smc = rospy.ServiceProxy('/SLAM/slam_status', slam_status)
                result = smc("get",0)
                if result.return_code==0:
                    mapping_mode=bool(result.mapping_mode)
                    status=result.status
                    progress=result.progress
            except Exception as e:
                rospy.logwarn(e)
                continue

            msgs = {
                'topic': 'slam_status',
                'total': feature_points_counts[0]+feature_points_counts[1],
                "Identified": feature_points_counts[0],
                "config": config,
                "status":status,
                "mapping_mode":mapping_mode,
                "progress":progress
            }

            # print(msgs,mtime)
            self.rate.sleep()
            
            self.client.publish(Topic_name,json.dumps(msgs),0)
    def set_pub_rate(self,rate):
        self.rate= rospy.Rate(rate)
