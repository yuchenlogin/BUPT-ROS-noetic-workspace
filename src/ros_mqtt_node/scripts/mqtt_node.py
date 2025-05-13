#!/usr/bin/python3

import threading
import paho.mqtt.client as mqtt
import rospy
import rospkg
import json
import time
from std_msgs.msg import UInt32MultiArray
from SLAM.srv import *
from slam_status_topic import Slam_fp_pub

HOST = '0.0.0.0'
PORT = 1883
KEEPALIVE = 60
CLIENT_ID = 'Roban_client'
USERNAME = 'admin'
PASSWORD = 'publish'
ROS_NODE_NAME = 'mqtt_node'
COMMAND_KEY = 'cmd'

rospy.init_node(ROS_NODE_NAME, anonymous=True, log_level=rospy.INFO)

def resume_slam_mapping(cli):
    msgs = {
            'cmd': "resume_slam_mapping",
            "process_msg":"",
            'process_result': 1 ,# response error
        }
    try:
        rospy.wait_for_service('/SLAM/slam_mapping_control', 2)
        smc = rospy.ServiceProxy('/SLAM/slam_mapping_control', slam_mapping_control)
        result = smc("resume")
    except Exception as e:
        rospy.logwarn(e)
        msgs["process_result"]=1
        msgs["process_msg"]="slam control srv call fail"
    else:
        msgs["process_result"]=result.return_code
        msgs["process_msg"]=result.msg

    
    cli.publish('/commands_result', process_dict_to_mqtt_payload(msgs), 2)

def set_slam_mapping_status(cli,status):
    msgs = {
            'cmd': "set_slam_mapping_status",
            'process_result': False ,
        }
    try:
        rospy.wait_for_service('/SLAM/slam_status', 2)
        smc = rospy.ServiceProxy('/SLAM/slam_status', slam_status)
        result = smc("set",status)
    except Exception as e:
        rospy.logwarn(e)
        msgs["process_result"]=False
    else:
        msgs["process_result"]=False if result.return_code else True

    cli.publish('/commands_result', process_dict_to_mqtt_payload(msgs), 2)

def get_slam_mapping_status(cli):
    msgs = {
            'cmd': "get_slam_mapping_status",
            'process_result': False ,
            'status':"None",
            'mapping_mode':-1, 
            'progress':-1
        }
    try:
        rospy.wait_for_service('/SLAM/slam_status', 2)
        smc = rospy.ServiceProxy('/SLAM/slam_status', slam_status)
        result = smc("get",0)
    except Exception as e:
        rospy.logwarn(e)
        msgs["process_result"]=False
    else:
        msgs["process_result"]=False if result.return_code else True
        msgs["status"]=result.status
        msgs["mapping_mode"]=result.mapping_mode
        msgs["progress"]=result.progress
    cli.publish('/commands_result', process_dict_to_mqtt_payload(msgs), 2)

def slam_set_config(cli,configjs):
    slam_config_path=rospkg.RosPack().get_path('ros_mqtt_node')+"/config/slam_mapping.json"
    msgs = {
            'cmd': "slam_set_config",
            'process_result': False ,
        }
    try:
        with open(slam_config_path,"w")as f:
            json.dump(configjs, f,indent=4)
            f.flush()
    except Exception as e:
        rospy.logwarn(e)
    else:
        msgs["process_result"]=True
    cli.publish('/commands_result', process_dict_to_mqtt_payload(msgs), 2)

def set_fp_pub_rate(cli,rate):
    msgs = {
            'cmd': "set_fp_pub_rate",
            'process_result': False ,
        }
    try:
        fp_puber.set_pub_rate(rate)
    except Exception as e:
        rospy.logwarn(e)
    else:
        msgs["process_result"]=True
    cli.publish('/commands_result', process_dict_to_mqtt_payload(msgs), 2)

def slam_mapping_move(cli,vector):
    msgs = {
            'cmd': "slam_mapping_move",
            'process_result': False ,
        }
    if len(vector)==3:
        try:
            rospy.wait_for_service('/SLAM/slam_mapping_walk', 2)
            smc = rospy.ServiceProxy('/SLAM/slam_mapping_walk', slam_mapping_walk)
            result = smc(vector)
        except Exception as e:
            rospy.logwarn(e)
            msgs["process_result"]=False
        else:
            msgs["process_result"]=False if result.return_code else True

    cli.publish('/commands_result', process_dict_to_mqtt_payload(msgs), 2)

def stop_slam_mapping(cli):
    msgs = {
            'cmd': "stop_slam_mapping",
            'process_result': False ,
        }
    try:
        rospy.wait_for_service('/SLAM/slam_mapping_control', 2)
        smc = rospy.ServiceProxy('/SLAM/slam_mapping_control', slam_mapping_control)
        result = smc("stop")
    except Exception as e:
        rospy.logwarn(e)
        msgs["process_result"]=False
    else:
        msgs["process_result"]=False if result.return_code else True

    cli.publish('/commands_result', process_dict_to_mqtt_payload(msgs), 2)
    
def start_slam_mapping(cli):
    msgs = {
            'cmd': "start_slam_mapping",
            'process_result': False ,
        }
    try:
        rospy.wait_for_service('/SLAM/slam_mapping_control', 5)
        smc = rospy.ServiceProxy('/SLAM/slam_mapping_control', slam_mapping_control)
        result = smc("start")
    except Exception as e:
        rospy.logwarn(e)
        msgs["process_result"]=False
    else:
        msgs["process_result"]=False if result.return_code else True

    cli.publish('/commands_result', process_dict_to_mqtt_payload(msgs), 2)

def get_slam_feature_points(cli):
    try:
        sub_result = rospy.wait_for_message("/SLAM/FeaturePoint/Quantity", UInt32MultiArray, 2)
        feature_points_counts = sub_result.data
        process_success = True
    except:
        rospy.logwarn("cannot subscribe to message: /SLAM/FeaturePoint/Quantity")
        process_success = False
        feature_points_counts = []

    msgs = {
        'cmd': 'get_slam_feature_points',
        'process_result': process_success,
        'data': feature_points_counts
    }
    cli.publish('/commands_result', process_dict_to_mqtt_payload(msgs), 2)

def async_do_job(func, args=None):
    """unblock do func task

    :param func:
    :param args:
    :return:
    """
    threading.Thread(target=func, args=args).start()


def process_mqtt_payload_to_dict(payload):
    return json.loads(payload.decode('utf-8'))

def process_dict_to_mqtt_payload(dict):
    return json.dumps(dict)

def on_message(client, userdata, msg):
    recivce_data = process_mqtt_payload_to_dict(msg.payload)
    rospy.loginfo(recivce_data)
    cmd=recivce_data[COMMAND_KEY]
    if cmd == 'get_slam_feature_points':
        async_do_job(get_slam_feature_points, args=(client,))
    elif cmd=="start_slam_mapping":
        async_do_job(start_slam_mapping, args=(client,))
    elif cmd=="stop_slam_mapping":
        async_do_job(stop_slam_mapping, args=(client,))
    elif cmd == "slam_mapping_move":
        async_do_job(slam_mapping_move, args=(client,recivce_data["vector"]))
    elif cmd == "slam_set_config":
        async_do_job(slam_set_config, args=(client,recivce_data["config"]))
    elif cmd == "set_fp_pub_rate":
        async_do_job(set_fp_pub_rate, args=(client,recivce_data["rate"]))
    elif cmd == "get_slam_mapping_status":
        async_do_job(get_slam_mapping_status, args=(client,))
    elif cmd == "set_slam_mapping_status":
        async_do_job(set_slam_mapping_status, args=(client,recivce_data["status"]))
    elif cmd == "resume_slam_mapping":
        async_do_job(resume_slam_mapping, args=(client,))
    else:
        msgs = {
        'cmd': cmd,
        'process_result': False,
        'data': "unknown cmd"
        }
        client.publish('/commands_result', process_dict_to_mqtt_payload(msgs), 1)


def on_connect(client, userdata, flags, rc):
    if rc != 0:
        rospy.logerr("Connection Fail! Returned code=",str(rc))

    client.subscribe("/commands")

def rosShutdownHook():
    client.disconnect()

rospy.on_shutdown(rosShutdownHook)
client = mqtt.Client(client_id=CLIENT_ID, clean_session=None)
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set(USERNAME, PASSWORD)
client.connect(HOST, PORT, KEEPALIVE)
fp_puber=Slam_fp_pub(client)
fp_puber.start()
client.loop_forever()
