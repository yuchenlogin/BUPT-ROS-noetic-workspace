#!/usr/bin/python3
# coding=utf-8

import random
import threading
import paho.mqtt.client as mqtt
import rospy
import rospkg
import json
import time
from std_msgs.msg import UInt32MultiArray

HOST = '0.0.0.0'
PORT = 1883
KEEPALIVE = 60
CLIENT_ID = 'roban_speech_test'
USERNAME = 'admin'
PASSWORD = 'publish'
COMMAND_KEY = 'cmd'

def async_do_job(func, args=None):
    """unblock do func task

    :param func:
    :param args:
    :return:
    """
    threading.Thread(target=func, args=args).start()

def payload_to_dict(payload):
    return json.loads(payload.decode('utf-8'))

def dict_to_payload(dict):
    return json.dumps(dict)


class MqttClient(mqtt.Client):
    def __init__(self, client_id=CLIENT_ID,host = HOST,port = PORT,username = USERNAME, psw = PASSWORD):
        super().__init__(client_id=client_id)
        self.username_pw_set(username, psw)
        self.connect(host, port, KEEPALIVE)
    
    def on_connect(self,client, userdata, flags, rc):
        if rc != 0:
            print("Connection Fail! Returned code=",str(rc))
        client.subscribe("/test_commands")
                
    def publish(self,topic,msgs):
        super().publish(topic,json.dumps(msgs),2)
    
    def on_message(self, client, userdata, msg):
        rec_topic, rec_data = msg.topic,payload_to_dict(msg.payload)
        print("{}: {}".format(rec_topic,rec_data))
    
    def rosShutdownHook(self):
        self.disconnect()
        
if __name__ == "__main__":
    client_id = "mqtt-tester-{}".format(random.randint(0,10000))
    t = MqttClient(client_id=client_id,host=HOST,port=PORT)
    MQTT_result_topic = "/speech_test_result"
    t.subscribe(MQTT_result_topic)
    t.loop_forever()

