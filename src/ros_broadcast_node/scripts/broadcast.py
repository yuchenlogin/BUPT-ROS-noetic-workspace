#!/usr/bin/env python
import rospy
import os
import time
import json
import getpass
import socket
import re
import shutil
import rospkg
import sys

sys.path.append(rospkg.RosPack().get_path('ros_socket_node') + "/scripts")
from process_roban_config import get_robot_name

ROS_SOCKET_PORT = 12000
BRD_SOCKET_PORT = 5005

def get_ubuntu_version():
    return os.popen('lsb_release -a | grep "Release"').read().split()[-1]
ubuntu_version =get_ubuntu_version()
print("ubuntu version: %s" %ubuntu_version)

def get_username():
    return getpass.getuser()


def find_ip(value):
    ip_pattern = r'((2[0-4]\d\.|1\d{2}\.|[1-9]?\d\.){3}(25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d))'
    ip = re.findall(ip_pattern, value)
    return (ip[0][0], ip[1][0])


def find_mac(value):
    mac_pattern = r'([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}'
    mac = re.search(mac_pattern, value)
    return mac.group()


def get_net_name():
    if ubuntu_version=="20.04":
        cmd = r"ifconfig | grep  'flags=' | awk '{print $1}'"
    else:
        cmd = r"ifconfig | grep  'Link encap' | awk '{print $1}'"
    names = os.popen(cmd).readlines()
    return [name.rstrip().replace(":","") for name in names if name.replace(":","") != 'lo\n']


def find_all():
    result = []
    for name in get_net_name():
        try:
            val = "".join(os.popen(
                "ifconfig | grep -A3 {}".format(name)).readlines())
            ip = find_ip(val)
            mac = find_mac(val)
            result.append((ip, mac))
        except:
            continue
    return result


def send_broadcast(msgs):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    for msg in msgs:
        s.sendto(json.dumps(msg[1]).encode('utf-8'), (msg[0], BRD_SOCKET_PORT))
    s.close()


def main():
    ros_node_name = 'ros_broadcast_node'
    rospy.init_node(ros_node_name, anonymous=True)

    while not rospy.is_shutdown():
        try:
            rospy.sleep(3)
            datas = find_all()
            msgs = []
            for data in datas:
                ip = data[0][0]
                username = get_username()
                robot_name = get_robot_name()
                mac = data[1]
                msgs.append([])
                msgs[-1].append(data[0][1])
                msgs[-1].append({
                    "ip": ip,
                    "username": username,
                    "ros_port": ROS_SOCKET_PORT,
                    "mac": mac,
                    "robot_name": robot_name
                })
            send_broadcast(msgs)
        except Exception as error:
            rospy.loginfo("ros_broadcast_node exiting {}".format(sys.exc_info()))
            break
            


if __name__ == '__main__':
    main()
