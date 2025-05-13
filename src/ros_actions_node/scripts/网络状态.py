#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import os
import re
import psutil
import sys
import subprocess
sys.path.append('/home/lemon/robot_ros_application/catkin_ws/src/keyboards/scripts')
from play_tts_file import Player
from lejulib import *
def get_ubuntu_version():
    return os.popen('lsb_release -a | grep "Release"').read().split()[-1]
ubuntu_version =get_ubuntu_version()
            
def filter_ssid(ssid):
    ssid = re.sub(r'[^A-Za-z0-9 ]+', ' ', ssid)
    if len(ssid) == 1 and ssid[0] == '':
        ssid = 'chinese'
    return ssid
class NetworkInfo(Player):
    def __init__(self):
        Player.__init__(self)
        self.voice_file_path = '/home/lemon/robot_ros_application/catkin_ws/src/keyboards/voice/'
        self.connect_status = False
        self.ssid = ''
        self.ipv4 = ''
        self.wlan = ''

    @property
    def get_connect_status(self):
        try:
            self.ssid,self.ipv4 = self.get_wifi_information()
            print(self.ssid)
        except Exception as e:
            print(e,__file__)
        else:
            if len(self.ssid)!=0:
                self.connect_status = True
            else:
                self.connect_status = False
        
    def get_wifi_information(self):
        ssid = subprocess.check_output(["iwgetid", "-r"]).strip().decode("utf-8").strip()
        ssid = filter_ssid(ssid)
        print(ssid)
        if ubuntu_version == "16.04":
            ipv4 = subprocess.check_output("/sbin/ifconfig wlp0s20f3 | grep \"inet addr\" | awk -F: \'{print $2}\' | awk \'{print $1}\'", shell=True)
        else:
            ipv4 = subprocess.check_output("ifconfig wlo1 | grep 'inet' -w | awk -F ' ' '{print $2}'",shell=True).decode()
        ipv4 = re.match(r'\d+.\d+.\d+.\d+', ipv4).group()
        return ssid, ipv4

    def play_network_info(self):
        self.play_complete_voice(self.voice_file_path, "get_wifi_info")
        self.get_connect_status
        if self.connect_status == False:
            self.play_complete_voice(self.voice_file_path, "unconnected_wifi")
        else:
            self.play_complete_voice(self.voice_file_path, "roban")
            self.play_complete_voice(self.voice_file_path, "ssid")
            if self.ssid == 'chinese':
                self.play_complete_voice(self.voice_file_path, "chinese_wifiname")
            else:
                self.play_single_voice(self.voice_file_path, self.ssid.lower())
            self.play_complete_voice(self.voice_file_path, "ip")
            self.play_single_voice(self.voice_file_path, self.ipv4.lower())

def rosShutdownHook():
    print("shutdown")
    os.system('ps -aux | grep play | awk \'{print $2}\' | xargs kill -9')
    finishsend()
    os._exit(0)

if __name__ == "__main__":
    rospy.init_node("check_servo")
    rospy.on_shutdown(rosShutdownHook)
    rospy.Subscriber('terminate_current_process', String, terminate)
    net_info = NetworkInfo()
    net_info.play_network_info()
