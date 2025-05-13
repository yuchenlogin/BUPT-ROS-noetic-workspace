#!/usr/bin/env python3
# coding=utf-8
import json
import rospy
import rospkg
import os
import sys
import random
from std_msgs.msg import *
from threading import Timer

sys.path.append(rospkg.RosPack().get_path('ros_AIUI_node')+"/scripts")
sys.path.append(rospkg.RosPack().get_path('leju_test_nodes')+"/scripts")
from EventListener import EventListener as aiui_eventlistener
from AIui_node import AIUINode, load_local_skills
from MqttClient import MqttClient
from healthy_checker import HealthChecker
skills_config_path = os.path.join(os.path.split(os.path.realpath(__file__))[0],"VoiceControlConfig.json")
local_custom_skills_dict = load_local_skills(skills_config_path)
NodeName = "speech_test_node"
MQTT_result_topic = "/speech_test_result"
Test_Timeout_s = 10

CHECK_SUCC = 0
class SpeechTester(aiui_eventlistener):
    def __init__(self):
        self.debug = True if len(sys.argv) > 1 and sys.argv[1] == "debug" else False
        super().__init__(skills_dict = local_custom_skills_dict, debug = self.debug)
        self.kill_aiui_voice_toward_voice()
        self.aiuinode = AIUINode(self, self.debug, wakeup_mute=True)
        self.aiuinode.start(spin=False)
        self.healthy_checker = HealthChecker()
        self.mqttclient = MqttClient(NodeName)
        self.mqttclient.loop_start()
        rospy.Subscriber("/micarrays/wakeup", String, self.wakeup_callback)
        self.test_timeout_timer = Timer(Test_Timeout_s,self.timeoutpub)
        self.MQTT_msg = {}
        
    def timeoutpub(self):
        self.MQTT_msg["healthy_info"] = self.check_roban_health()
        self.mqttclient.publish(MQTT_result_topic,self.MQTT_msg)
        
    def check_roban_health(self):
        print("检查roban健康状态...")
        network_result = self.healthy_checker.check_network()
        mic_result = self.healthy_checker.check_mic()
        servo_id_result = self.healthy_checker.check_servo_id()
        servo_temperature_result = self.healthy_checker.get_servo_temperature()
        healthy_info = {
            "network": "网络连接正常"if network_result == CHECK_SUCC else "网络连接异常："+str(network_result),
            "microphone": "麦克风连接{}".format("正常" if mic_result == CHECK_SUCC else "异常!"),
            "servo_id": "舵机连接正常" if servo_id_result == CHECK_SUCC else "舵机连接异常,异常id为{}".format(servo_id_result),
            "servo_T": servo_temperature_result
        }
        print(healthy_info)
        return healthy_info
        
    def rosShutdownHook(self):
        self.mqttclient.rosShutdownHook()
        self.aiuinode.rosShutdownHook()
    
    def kill_aiui_voice_toward_voice(self):
        rospy.logwarn("该测试会关闭aiui和声源定位节点!\n测试完毕需要重启start.sh或重启机器人才可以使用语音控制功能")
        os.system("rosnode kill /ros_aiui_node;rosnode kill /head_toward_sound")
            
    def wakeup_callback(self,msg):
        print(msg,"wakeup_callback")
        data = json.loads(msg.data.replace("'", '"'))
        wakeup_msg = {
            "type":"wakeup",
            "angle":data["angle"],
            "score":data["score"]
        }
        self.MQTT_msg = {}
        self.MQTT_msg["wakeup"] = wakeup_msg
        self.test_timeout_timer = Timer(Test_Timeout_s,self.timeoutpub)
        self.test_timeout_timer.start()

    def EventNLP(self, event):
        """重写父类语义结果事件回调函数"""
        nlp_result_text = event.data["text"] # 获取语义结果
        NLP_msg = {
            "type":"NLP",
            "text":nlp_result_text
        }
        print(NLP_msg)
        if not self.test_timeout_timer.finished.is_set():
            self.MQTT_msg["NLP"] = NLP_msg
            self.MQTT_msg["healthy_info"] = self.check_roban_health()
            self.test_timeout_timer.cancel()
            self.mqttclient.publish(MQTT_result_topic,self.MQTT_msg)

    def EventTTS(self,event):
        if self.debug:# debug 模式才播放
            super().EventTTS(event)
        
if __name__ == "__main__":
    rospy.init_node(NodeName, anonymous=False) # 实例化aiui节点
    t = SpeechTester()
    rospy.on_shutdown(t.rosShutdownHook)# 设置节点结束时的回调函数
    rospy.spin()
