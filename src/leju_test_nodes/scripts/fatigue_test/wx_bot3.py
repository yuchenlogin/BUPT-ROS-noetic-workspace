#!/usr/bin/env python3
import base64
import yaml
import sys, os
import requests
import hashlib
import rospy

CONFIG_PATH = os.path.join(os.path.split(sys.argv[0])[0], "../config")
WORK_WEIXIN_ROBOT_KEY = "WX_BOT_KEY"
NOTICE_LIST = "NOTICE_LIST"

def base64_read(path):
    data = ""
    with open(path,"rb") as f:
        data = base64.b64encode(f.read())
        md = hashlib.md5()
    md.update(data)
    image_md5 = md.hexdigest()
    return image_md5,str(data,'utf-8')

class WechatRobot(object):
    """
    企业微信机器人发送消息类,不能超过20条/分钟。
    企业微信机器人API文档：https://developer.work.weixin.qq.com/document/path/92455
    """

    def __init__(self):
        self.config = self.load_config()
        self.key = self.config[WORK_WEIXIN_ROBOT_KEY]
        self.call_list = self.config[NOTICE_LIST]
        self.headers = {'Content-Type': 'application/json;charset=utf-8'}
        self.robot_curl = "https://qyapi.weixin.qq.com/cgi-bin/webhook/send?key={}".format(self.key)
        self.upload_url = 'https://qyapi.weixin.qq.com/cgi-bin/webhook/upload_media?key={}&type=file'.format(self.key)
        self.req_json = {}
        rospy.spin()

    def load_config(self):
        with open(os.path.join(CONFIG_PATH, "config.yaml"), "r") as f:
            return yaml.safe_load(f)

    def send_txt(self, txt: str,phones = []) -> bool:
        """
        发送文本
        :param txt: 待发送文本内容
        :return:
        """
        self.req_json["msgtype"] = "text"
        self.req_json["text"] = {}
        self.req_json["text"]["content"] = txt
        self.req_json["text"]["mentioned_mobile_list"] = phones
        try:
            resp = requests.post(url=self.robot_curl, headers=self.headers, json=self.req_json)
        except Exception as e:
            print(e)

    def send_pic(self, pic_path: str) -> bool:
        """
        发送图片
        :param pic_path: 图片路径
        :return:
        """
        f_md5,base64data = base64_read(pic_path)
        self.req_json["msgtype"] = "image"
        self.req_json["image"] = {}
        self.req_json["image"]["base64"] = base64data
        self.req_json["image"]["md5"] = f_md5
        try:
            resp = requests.post(url=self.robot_curl, headers=self.headers, json=self.req_json)
        except Exception as e:
            print(e)

    def send_file(self, file_path: str) -> bool:
        """
        发送文件,例如: report.html
        :param file_path: 文件路径
        :return:
        """
        try:
            self.req_json["msgtype"] = "file"
            self.req_json["file"] = {}
            self.req_json["file"]["media_id"] = self.upload_file(file_path)
            resp = requests.post(url=self.robot_curl, headers=self.headers, json=self.req_json)
        except Exception as e:
            print(e)

    def upload_file(self, file_path: str) -> str:
        """
        上传文件，发送文件前需要现上传
        :param file_path: 文件路径
        :return:
        """
        try:
            data = {'file': open(file_path, 'rb')}
            resp = requests.post(self.upload_url, files=data)
            json_res = resp.json()
            if json_res.get('media_id'):
                return json_res.get('media_id')
        except Exception:
            return ""

if __name__ == '__main__':
    rospy.init_node("ros_work_weixin_robot_node", anonymous=False)
    wx = WechatRobot()
