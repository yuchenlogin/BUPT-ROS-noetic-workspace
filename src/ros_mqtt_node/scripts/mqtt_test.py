#!/usr/bin/python3


import paho.mqtt.client as mqtt
import time
import json
import random 
import sys

HOST = "127.0.0.1"
PORT = 1883
keepalive = 60
client_id = "pyMQpub-{}".format(random.randint(0,10000))
username = "admin"
password = "publish"
Debug=False
def on_publish(client, userdata, mid):
    # print("mid: "+str(mid)+f" {timestamp} ---> Running...")
    print("pub: "+str(mid))
    pass

def on_message(client, userdata, msg):
    if msg.topic !="/slam_status" or Debug:
        print(msg.topic+" "+":"+str(msg.payload))
    pass

def on_connect(client, userdata, flags, rc):
    print("on_connect", userdata, flags, rc)
    client.subscribe('/commands_result')
    client.subscribe("/slam_status")

msggg = {
    "cmd": "start_slam_mapping"
}

def processe_to_json(origin):
    return json.dumps(origin)

def iii():
    while True:
        ss = input('here:')
        if ss == 'j':
            client.publish("/commands", processe_to_json({"cmd": "get_slam_feature_points"}), 1)
        elif ss == 'k':
            client.publish("/commands", processe_to_json({"cmd": "start_slam_mapping"}), 1)
        elif ss == 'l':
            client.publish("/commands", processe_to_json({"cmd": "stop_slam_mapping"}), 1)
        elif ss == "w":
            client.publish("/commands", processe_to_json({"cmd": "slam_mapping_move","vector":[0.1,0.0,0]}), 2)
        elif ss == "s":
            client.publish("/commands", processe_to_json({"cmd": "slam_mapping_move","vector":[-0.1,0.0,0]}), 2)
        elif ss == "a":
            client.publish("/commands", processe_to_json({"cmd": "slam_mapping_move","vector":[0.0,0.06,0]}), 2)
        elif ss == "d":
            client.publish("/commands", processe_to_json({"cmd": "slam_mapping_move","vector":[0.0,-0.06,0]}), 2)
        elif ss == "z":
            client.publish("/commands", processe_to_json({"cmd": "slam_mapping_move","vector":[0.0,0.0,50]}), 2)
        elif ss == "c":
            client.publish("/commands", processe_to_json({"cmd": "slam_mapping_move","vector":[0.0,0.0,-50]}), 2)
        elif ss == "f":
            client.publish("/commands", processe_to_json({"cmd": "slam_set_config","config":{
                "featurePoint":250,
                "proportion":0.6
            }
            }), 2)
        elif ss == "g":
            client.publish("/commands", processe_to_json({"cmd": "set_fp_pub_rate","rate":2}), 2)
        elif ss == "n":
            client.publish("/commands", processe_to_json({"cmd": "set_slam_mapping_status","status":1}), 2)
        elif ss == "nn":
            client.publish("/commands", processe_to_json({"cmd": "set_slam_mapping_status","status":0}), 2)
        elif ss == "m":
            client.publish("/commands", processe_to_json({"cmd": "get_slam_mapping_status"}), 2)
        elif ss == "b":
            client.publish("/commands", processe_to_json({"cmd": "resume_slam_mapping"}), 2)
#MQTT连接
client = mqtt.Client(client_id,clean_session = None)

#遗言消息定义
will_MSG= {
            "ID":"2",
            "stat":"2",
          }          
#遗言消息 一旦连接到MQTT服务器，遗言消息就会被服务器托管，本客户端凡是非正常断开连接 服务器就会将本遗言发送给订阅该遗言消息的客户端告知对方本客户端离线；
client.will_set("W/topic", payload=json.dumps(will_MSG), qos=0, retain=True)

client.username_pw_set(username,password) #如果MQTT broker不要求身份认证可以注释本语句 必须要在connect前调用
client.on_connect = on_connect
client.connect(HOST, PORT, keepalive)
client.on_message = on_message
client.on_publish = on_publish
client.loop_start()

if __name__ == '__main__':   
    if len(sys.argv)>= 2:
        Debug=True
    iii()
