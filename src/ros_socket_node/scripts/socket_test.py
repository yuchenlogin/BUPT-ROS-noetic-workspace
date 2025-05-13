#!/usr/bin/env python3
# coding=utf-8
from urllib import response
import os
import sys
import random
import socket
import json
import time

#### socket connect info ####
SERVER_IP = "127.0.0.1"
SERVER_PORT = 12000
SOCKET_CONNECT_TIMEOUT = 10

def dict_to_bytes(dict_msg:dict):
    return json.dumps(dict_msg).encode("utf-8")

class Socket_test_client(object):
    def __init__(self):
        self.socket_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_client.settimeout(SOCKET_CONNECT_TIMEOUT)
        try:
            self.socket_client.connect((SERVER_IP, SERVER_PORT))
            print("Socket_runner_client Connected successfully")
        except Exception as err:
            print("{} {}".format(err,__file__))
        self.socket_client.settimeout(None)

    def send(self, msg):
        try:
            self.socket_client.send(dict_to_bytes(msg))
            self.socket_client.settimeout(3)
            recdata=self.socket_client.recv(1024*1024)
            tmp_json = json.loads(recdata.decode("utf-8"))
            print("rec:\n",json.dumps(tmp_json, indent=4, ensure_ascii=False, sort_keys=True,separators=(',', ':')))
            return 0
        except Exception as e:
            return -1
  
if __name__ == "__main__":
    cmd = {"cmd":"stop_node"}
    if len(sys.argv) == 2:
        if "{"in sys.argv[1]:
            try:
                cmd = json.loads(sys.argv[1])
            except Exception as e:
                print(e)
            
        else:                
            cmd = {"cmd":sys.argv[1]}
    
    runner=Socket_test_client()
    runner.send(cmd)
