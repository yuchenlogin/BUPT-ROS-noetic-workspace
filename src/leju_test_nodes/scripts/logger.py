#!usr/bin/python
# -*- coding: utf-8 -*-

import io
import os
import signal
import subprocess
import sys, time
import threading
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import yaml

SCRIPTS_PATH=os.path.split(sys.argv[0])[0]

class Logger(threading.Thread):
    def __init__(self, log_path="./"):
        super(Logger, self).__init__()
        self.terminal = sys.stdout
        file_name = time.strftime("%Y-%m-%d-%H.%M.%S.log", time.localtime(time.time()))
        self.log_file = os.path.join(log_path,file_name)
        self.log_path = log_path
        self.running = True
        self.log_write_time = time.time() - 2
        
        self.loglist = []
        if len(log_path) and not os.path.exists(log_path):
            os.system("mkdir -p {}".format(log_path))   
        self.bridge = CvBridge()
        self.start()

    def run(self):
        self.log = open(self.log_file, 'w')  
        log_change = True
        try:
            while self.running or len(self.loglist):
                if len(self.loglist):
                    self.process(self.loglist.pop(0))
                    log_change = True
                else:
                    time.sleep(0.05)
                if log_change and time.time() - self.log_write_time > 1:
                    log_change = False
                    self.flush() # 文件写入至少1s间隔,减少io操作
            self.flush()
        except:
            print(sys.exc_info(), __file__)
        finally:
            self.log.close()

    def stop(self):
        self.running = False

    def write(self, message, with_time_stamp = True, print_color=37):
        timestr = "[{}]\t".format(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))) if with_time_stamp else ""
        log_info = {"msg": timestr + message, "msg_color": print_color}
        self.loglist.append(log_info)

    def save_current_image(self,title = ""):
        try:
            name = os.path.join(self.log_path,title + time.strftime("%Y-%m-%d-%H.%M.%S.jpg", time.localtime(time.time())))
            msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=2)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite(name,cv_image)
            msg = "save current image to {}\n".format(name)
        except Exception as e:
            msg = "save current image FAIL! {}\n".format(e)
            rospy.logwarn(msg)
        finally:
            self.write(msg)

    def process(self, log_info):
        self.terminal.write("\033[{}m{}\033[0m".format(log_info["msg_color"], log_info["msg"]))
        self.log.write(log_info["msg"])
        self.terminal.flush()

    def flush(self):
        self.log_write_time = time.time()
        self.log.flush()

class TopicsRecorder(object):
    def __init__(self,topic=None,duration=180,split=True,split_count = 2,size=None,output="Topics_record.bag"):
        """"录制指定topics
        :topic 录制的topic列表或者包含topic的配置文件
        :duration 录制的时间,单位s
        :split 录制时间结束是否重新继续录制,为True时会打开另外一个文件录制,为False录完duration或size即停止
        :split_count 保留录制文件的数目,之前录制的文件会被删除
        :size 录制文件大小限制
        :output 录制文件输出的路径和文件名,分包录制时会自动加上后缀0,1,2..
        """
        if type(topic) == list:
            self.set_topics(topic)
        elif type(topic) == str:
            self.load_config(topic)
        else:
            self.topics = ""
        self.duration= "--duration={}".format(int(duration)) if duration else ""
        self.split = "--split" if split and (duration or size) else ""
        self.output = " -O {}".format(output)
        self.split_count = "--max-splits={}".format(int(split_count)) if split_count else ""
        self.slam_process=None

    def load_config(self,file = os.path.join(SCRIPTS_PATH,"record_topics.yaml")):
        if os.path.exists(file):
            with open(file,"r")as f:
                self.topics = yaml.load(f)

    def set_topics(self,topics = []):
        self.topics = " ".join([str(topic) for topic in topics])

    def start(self):
        cmd = "rosbag record {} {} {} {} {}".format(self.topics,self.duration,self.split,self.split_count,self.output)
        self.slam_process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
    def stop(self):
        if self.slam_process and self.slam_process.poll() is None:
            print("kill rosbag")
            os.system("for line in $(rosnode list | grep /record);do rosnode kill $line ;done")


if __name__ == "__main__":
    t = TopicsRecorder(["/chin_camera/image","/camera/depth/image_rect_raw"],duration=2,output="~/fatigue_test/ffff.bag")
    t.start()
    time.sleep(5)
    t.stop()
