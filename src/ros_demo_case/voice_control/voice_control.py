#!/usr/bin/env python3
# coding=utf-8
import rospy
import rospkg
import os
import sys
import random
sys.path.append(rospkg.RosPack().get_path('ros_AIUI_node')+"/scripts")
from EventListener import EventListener as aiui_eventlistener
from AIui_node import AIUINode, load_local_skills
skills_config_path = os.path.join(os.path.split(os.path.realpath(__file__))[0],"VoiceControlConfig.json")
local_custom_skills_dict = load_local_skills(skills_config_path)

class EventListener(aiui_eventlistener):
    def EventNLP(self, event):
        """重写父类语义结果事件回调函数"""
        print(event.data)
        nlp_result_text = event.data["text"] # 获取语义结果
        if nlp_result_text in self.local_custom_skills_dict.keys():# 查找匹配的命令
            local_skill_content = self.local_custom_skills_dict[nlp_result_text]
            choose_reply_text = random.choice(local_skill_content["reply"]) # 随机选择一个回答
            self.agent.cmd_tts(choose_reply_text) # 调用tts将回答的文字转语音播放
            if "cmd_type" in local_skill_content: # 如果时cmd_type类型的命令
                if local_skill_content["cmd_type"] == "stop_node":
                    socket_msg = {"cmd":"stop_node"}
                elif local_skill_content["cmd_type"] == "run_node":
                    socket_msg = {"cmd":"run_node","path": local_skill_content["path"]}
                self.socket_client.send(socket_msg) # 发送socket消息执行案例

    def EventTTS(self, event):
        """重写父类TTS(文字转语音)事件回调函数"""
        buffer = event.data["buffer"] # 获取一段语音数据
        if self.player.is_playing: # 如果正在播放语音,则停止播放
            self.player.stop()
            while self.player.is_abort == False:
                rospy.sleep(0.1)
        with open(self.tts_pcm_file, "ab+") as tts: # 将语音数据写入文件中
            tts.write(buffer)
        if self.is_tts_download_complete() == True: # 判断是否接收完成语音合成的结果
            self.player.play(self.tts_pcm_file) # 接收完则开始播放

if __name__ == "__main__":
    rospy.init_node("ros_aiui_node", anonymous=False) # 实例化aiui节点
    debug = True if len(sys.argv) > 1 and sys.argv[1] == "debug" else False
    eventlistener = EventListener(skills_dict = local_custom_skills_dict, debug = debug) # 实例化EventListener
    aiuinode = AIUINode(eventlistener, debug)
    rospy.on_shutdown(aiuinode.rosShutdownHook)# 设置节点结束时的回调函数
    aiuinode.start()
