#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import json
import os

SERVER_IP = '127.0.0.1'
SERVER_PORT = 12000
DEMO_DIRECTORY = '/home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/'
CMD_STOP_NODE = {"cmd": "stop_node"}
ignore_file_in_demo_directory = ['lejulib.py', '__init__.py']
demo_dict = {}

def _dict_to_bytes(dict_msg):
    return json.dumps(dict_msg).encode('utf-8')

def update_demo_dict():
    global demo_dict
    demo_list = list_demo()
    for index, demo_name in enumerate(demo_list):
        demo_dict[index+1] = demo_name

def list_demo():
    demo_list = []
    for file in os.listdir(DEMO_DIRECTORY):
        if file in ignore_file_in_demo_directory:
            continue
        if file.endswith('.py'):
            demo_list.append(file)
    return demo_list

def display_programe_info():
    global demo_dict
    print('%-20s%s' % (' ', '案例展示'))
    for key in demo_dict:
        print("%-15s%-15s" % (key, demo_dict[key]))
    print('')
    print("%-15s%-15s" % ('666', '终止案例'))
    print("%-15s%-15s" % ('999', '退出程序'))
    print('')

def main():
    global demo_dict
    update_demo_dict()
    display_programe_info()
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.connect((SERVER_IP, SERVER_PORT))
    
    while(1):
        try:
            receive_cmd = int(input("请输入需要执行的案例编号: "))
            if receive_cmd == 999:
                break
            elif receive_cmd == 666:
                send_msg = CMD_STOP_NODE
            else:
                send_msg = {"cmd": "run_node", "path" : DEMO_DIRECTORY + demo_dict[receive_cmd]}
            tcp_socket.send(_dict_to_bytes(send_msg))
        except:
            print('请检查输入的序号是否正确')

    tcp_socket.close()

if __name__ == '__main__':
    main()
