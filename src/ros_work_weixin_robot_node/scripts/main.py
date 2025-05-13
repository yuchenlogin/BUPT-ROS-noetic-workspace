#!/usr/bin/env python3
import base64
import yaml
import sys, os
import requests
import hashlib
import rospy
import ast
from ros_work_weixin_robot_node.srv import WorkWeixinTextSend, WorkWeixinFileSend

CONFIG_PATH = os.path.join(os.path.split(sys.argv[0])[0], "../config")
WORK_WEIXIN_ROBOT_KEY = "WX_BOT_KEY"
NOTICE_LIST = "NOTICE_LIST"
SUCCESS = 0

def base64_read(path):
    data = ""
    with open(path,"rb") as f:
        rf = f.read()
        data = base64.b64encode(rf)
        md = hashlib.md5()
    md.update(rf)
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
        self.robot_curl_format = "https://qyapi.weixin.qq.com/cgi-bin/webhook/send?key={}"
        self.upload_url_format = 'https://qyapi.weixin.qq.com/cgi-bin/webhook/upload_media?key={}&type=file'
        self.req_json = {}
        self.text_send = rospy.Service("/work_weixin/text_send", WorkWeixinTextSend, self.text_send_callback)
        self.pic_send = rospy.Service("/work_weixin/pic_send", WorkWeixinFileSend, self.pic_send_callback)
        self.file_send = rospy.Service("/work_weixin/file_send", WorkWeixinFileSend, self.file_send_callback)
        rospy.spin()

    def get_post_status(self, result_content):
        is_send_succeed = False
        if result_content == None:
            return is_send_succeed, None, None
        else:
            if result_content["errcode"] == SUCCESS:
                is_send_succeed = True
            return is_send_succeed, result_content["errcode"], result_content["errmsg"]

    def text_send_callback(self, req):
        key = self.key
        if req.robotKey != "none":
            key = req.robotKey
        result_content = self.send_txt(key, req.text, req.phones)
        return self.get_post_status(result_content)

    def pic_send_callback(self, req):
        key = self.key
        if req.robotKey != "none":
            key = req.robotKey
        result_content = self.send_pic(key, req.filePath)
        return self.get_post_status(result_content)

    def file_send_callback(self, req):
        key = self.key
        if req.robotKey != "none":
            key = req.robotKey
        result_content = self.send_file(key, req.filePath)
        return self.get_post_status(result_content)

    def load_config(self):
        with open(os.path.join(CONFIG_PATH, "config.yaml"), "r") as f:
            return yaml.safe_load(f)

    def send_txt(self, key: str, txt: str, phones=[]):
        """
        发送文本
        :param txt: 待发送文本内容
        :return:
        """
        self.req_json["msgtype"] = "text"
        self.req_json["text"] = {}
        self.req_json["text"]["content"] = txt
        self.req_json["text"]["mentioned_mobile_list"] = phones
        robot_curl = self.robot_curl_format.format(key)
        try:
            resp = requests.post(url=robot_curl, headers=self.headers, json=self.req_json)
            return ast.literal_eval(resp.content.decode("utf-8"))
        except Exception as e:
            print(e)
            return None

    def send_pic(self, key: str, pic_path: str):
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
        robot_curl = self.robot_curl_format.format(key)
        try:
            resp = requests.post(url=robot_curl, headers=self.headers, json=self.req_json)
            return ast.literal_eval(resp.content.decode("utf-8"))
        except Exception as e:
            print(e)
            return None

    def send_file(self, key: str, file_path: str):
        """
        发送文件,例如: report.html
        :param file_path: 文件路径
        :return:
        """
        try:
            self.req_json["msgtype"] = "file"
            self.req_json["file"] = {}
            self.req_json["file"]["media_id"] = self.upload_file(key, file_path)
            robot_curl = self.robot_curl_format.format(key)
            resp = requests.post(url=robot_curl, headers=self.headers, json=self.req_json)
            return ast.literal_eval(resp.content.decode("utf-8"))
        except Exception as e:
            print(e)
            return None

    def upload_file(self, key: str, file_path: str) -> str:
        """
        上传文件，发送文件前需要现上传
        :param file_path: 文件路径
        :return:
        """
        try:
            data = {'file': open(file_path, 'rb')}
            upload_url = self.upload_url_format.format(key)
            resp = requests.post(upload_url, files=data)
            json_res = resp.json()
            if json_res.get('media_id'):
                return json_res.get('media_id')
        except Exception:
            return ""

if __name__ == '__main__':
    rospy.init_node("ros_work_weixin_robot_node", anonymous=False)
    wx = WechatRobot()
