#!/usr/bin/env python3

import rospy
from ros_AIUI_node.srv import SrvWakeupMute, textToSpeakMultipleOptions
from std_msgs.msg import Empty, String
import json
from custom_constants import Constants
from ros_mic_arrays.srv import setBeamSrv
import random
import subprocess
from collections import Counter

TEXT_TO_SPEAK_SERVER = '/aiui/text_to_speak_multiple_options'
SERVER_TIMEOUT = 2
AIUI_PLAY_END = "/aiui/play_end"
SET_MIC_BEAM_SERVICE = '/ros_mic_arrays/set_real_beam'
BEAM_INDEX_FORWARD = 5
AIUI_SPEECH_REQUEST_SERVICE = '/aiui/wakeup_mute'
DO_NOT_REPLY_SKILL = False
NLP_TOPIC = '/aiui/nlp'
CONFIG_ROUND_START_KEY = "round_start_command"
CONFIG_ROUND_END_KEY = "round_end_command"
RPS_TOPIC = '/rps_node/gesture'
RULES = {
    "rock": "scissors",
    "paper": "rock",
    "scissors": "paper"
}

CONFIG_TTS_PARAMS_KEY = "tts_params"

class RPSGame():
    def __init__(self):
        rospy.init_node("rpsgame_node")
        self.__load_config()
        self.wakeup_microphone()
        self.tts_and_wait_for_play_end(self.tts_text["demo_start"])
        self.is_count_rps = False
        self.rps_list = []
        rospy.Subscriber(RPS_TOPIC, String, self.rps_cb)

    def rps_cb(self, msg):
        if self.is_count_rps:
            self.rps_list.append(msg.data)

    def __load_config(self):
        with open(Constants.config_file, 'r') as f:
            configs = json.loads(f.read())
            tts_params = configs[CONFIG_TTS_PARAMS_KEY]
            self.vcn = tts_params["vcn"]
            self.speed = tts_params["speed"]
            self.pitch = tts_params["pitch"]
            self.volume = tts_params["volume"]
            self.tts_text = configs["tts_text"]
            self.rps = configs["rps"]
            self.rps_sound = configs["rps_sounds"]

    def wakeup_microphone(self):
        try:
            rospy.wait_for_service(SET_MIC_BEAM_SERVICE, timeout=SERVER_TIMEOUT)
        except rospy.ROSException:
            log = 'set mic beam error: wait for {} timeout!'.format(SET_MIC_BEAM_SERVICE)
            exit(log)
        set_beam_client = rospy.ServiceProxy(SET_MIC_BEAM_SERVICE, setBeamSrv)
        set_beam_client(BEAM_INDEX_FORWARD)

    def wakeup_aiui_agent(self):
        try:
            rospy.wait_for_service(AIUI_SPEECH_REQUEST_SERVICE, timeout=SERVER_TIMEOUT)
        except rospy.ROSException:
            log = 'request aiui record error: wait for {} timeout!'.format(AIUI_SPEECH_REQUEST_SERVICE)
            exit(log)
        aiui_record_client = rospy.ServiceProxy(AIUI_SPEECH_REQUEST_SERVICE, SrvWakeupMute)
        aiui_record_client(DO_NOT_REPLY_SKILL)

    def tts_and_wait_for_play_end(self, text):
        try:
            rospy.wait_for_service(TEXT_TO_SPEAK_SERVER, SERVER_TIMEOUT)
        except rospy.ROSException:
            log = "wait for {} server timeout!".format(TEXT_TO_SPEAK_SERVER)
            exit(log)
        rospy.ServiceProxy(TEXT_TO_SPEAK_SERVER, textToSpeakMultipleOptions)(text, self.vcn, self.speed, self.pitch, self.volume)
        rospy.wait_for_message(AIUI_PLAY_END, Empty)

    def get_most_frequent_rps(self):
        if not self.rps_list:
            return None
        counter = Counter(self.rps_list)
        most_common_rps, count = counter.most_common(1)[0]
        self.rps_list = []
        if count >= 10:
            return most_common_rps
        else:
            return None

    def main(self):
        while not rospy.core.is_shutdown_requested():
            self.wakeup_aiui_agent()
            try:
                iat_result = rospy.wait_for_message(NLP_TOPIC, String, timeout=30).data
            except rospy.ROSException:
                self.tts_and_wait_for_play_end("快来和我玩吧，请对我说开始")
                continue
            if iat_result == self.tts_text[CONFIG_ROUND_START_KEY]:
                random_rps = random.choice(list(self.rps.keys()))
                self.p = subprocess.Popen('export PYTHONPATH=/home/lemon/robot_ros_application/catkin_ws/src/leju_lib_pkg/:$PYTHONPATH;python {}'.format(self.rps[random_rps]), stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
                self.is_count_rps = True
                while self.p.poll() is None:
                    rospy.sleep(0.1)
                self.is_count_rps = False
                rps_is = self.get_most_frequent_rps()
                if rps_is == None:
                    self.tts_and_wait_for_play_end(self.rps_sound['unrecognized'])
                elif rps_is == random_rps:
                    self.tts_and_wait_for_play_end(self.rps_sound["tie"])
                elif RULES[rps_is] == random_rps:
                    self.tts_and_wait_for_play_end(self.rps_sound["lose"])
                else:
                    self.tts_and_wait_for_play_end(self.rps_sound["win"])
            elif iat_result == self.tts_text[CONFIG_ROUND_END_KEY]:
                self.tts_and_wait_for_play_end(self.tts_text["demo_end"])
                break
            else:
                get_wrong_command = "我不知道{}是什么意思，请对我说开始".format(iat_result)
                self.tts_and_wait_for_play_end(get_wrong_command)

if __name__ == "__main__":
    rpsgame = RPSGame()
    rpsgame.main()
