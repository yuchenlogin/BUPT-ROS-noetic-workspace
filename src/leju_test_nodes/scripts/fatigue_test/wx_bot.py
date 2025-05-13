#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
import json
import urllib2

class WX_Bot(object):
    def __init__(self,key=""):
        self.header = "Content-Type: application/json"
        self.robot_curl = "https://qyapi.weixin.qq.com/cgi-bin/webhook/send?key={}".format(key)
        
    def send_message(self,msg,call_list = []):
        """
        发送文本
        :param msg: 待发送文本内容
        :call_list: 要艾特的人
        :return:
        """
        req_json={}
        req_json["msgtype"] = "text"
        req_json["text"] = {}
        req_json["text"]["content"] = msg
        req_json["text"]["mentioned_mobile_list"] = call_list
        data = data = json.dumps(req_json)
        try:
            req  = urllib2.Request(self.robot_curl, data, {'Content-Type':'application/json'})
            f = urllib2.urlopen(req)
            response = f.read()
            print(response)
            f.close()
        except Exception as e:
            print(e)


if __name__ == '__main__':
    key = ""
    call_list = []
    B = WX_Bot(key)
    B.send_message("测试")
