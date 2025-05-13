#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool
from std_srvs.srv import Empty as srvEmpty
from ros_AIUI_node.srv import SrvWakeupMute, textToSpeakMultipleOptions
from custom_constant import Constants
import json
from ros_speech_emotion_recognition.srv import SrvEmotionRecognition
from std_msgs.msg import String, Empty
from ros_mic_arrays.srv import setBeamSrv
import random

PAUSE_HEAD_TOWARD_SERVER = "/aiui/pause_head_toward_sound"
PAUSE = True
RESUME = False
SET_MIC_BEAM_SERVICE = '/ros_mic_arrays/set_real_beam'
SERVICE_TIMEOUT = 2
AIUI_SPEECH_REQUEST_SERVICE = '/aiui/wakeup_mute'
AIUI_TEXT_TO_SPEAK_SERVICE = '/aiui/text_to_speak_multiple_options'
DO_NOT_REPLY_SKILL = False
CONFIG_PATH = Constants.OS_MODULE.path.join(Constants.EMOTION_RECOGNITION_FOLDER, "configs", "config.json")
CONFIG_TTS_PARAMS_KEY = "tts_params"
EMOTION_RECOGNITION_SERVER = "/emotion_recognition"
AIUI_IAT_TOPIC = '/aiui/iat'
TTS_PLAY_END = "/aiui/play_end"
BEAM_INDEX_FORWARD = 5
AIUI_STOP_SPEECH_REQUEST_SERVICE = '/aiui/stop_recording'

class EmotionGuardian():
    def __init__(self):
        rospy.init_node("emotion_guardian_node")
        rospy.on_shutdown(self.rosShutdownHook)
        self.pause_head_toward_sound_client = rospy.ServiceProxy(PAUSE_HEAD_TOWARD_SERVER, SetBool)
        self.set_aiui_server_status(PAUSE)
        self.wakeup_microphone()
        self.load_config()
        self.text_to_speak(self.tts_text["demo_start"])
        rospy.wait_for_message(TTS_PLAY_END, Empty)

    def load_config(self):
        with open(CONFIG_PATH, 'r') as f:
            configs = json.loads(f.read())
            tts_params = configs[CONFIG_TTS_PARAMS_KEY]
            self.vcn = tts_params["vcn"]
            self.speed = tts_params["speed"]
            self.pitch = tts_params["pitch"]
            self.volume = tts_params["volume"]
            self.tts_text = configs["tts_text"]
            self.emotion_reply = configs["emotion_reply"]

    def wakeup_microphone(self):
        try:
            rospy.wait_for_service(SET_MIC_BEAM_SERVICE, timeout=SERVICE_TIMEOUT)
        except rospy.ROSException:
            log = 'set mic beam error: wait for {} timeout!'.format(SET_MIC_BEAM_SERVICE)
            exit(log)
        set_beam_client = rospy.ServiceProxy(SET_MIC_BEAM_SERVICE, setBeamSrv)
        set_beam_client(BEAM_INDEX_FORWARD)

    def set_aiui_server_status(self, is_pause):
        self.pause_head_toward_sound_client(is_pause)

    def rosShutdownHook(self):
        self.set_aiui_server_status(RESUME)
        rospy.ServiceProxy(AIUI_STOP_SPEECH_REQUEST_SERVICE, srvEmpty)()

    def wakeup_aiui_agent(self):
        try:
            rospy.wait_for_service(AIUI_SPEECH_REQUEST_SERVICE, timeout=SERVICE_TIMEOUT)
        except rospy.ROSException:
            log = 'request aiui record error: wait for {} timeout!'.format(AIUI_SPEECH_REQUEST_SERVICE)
            exit(log)
        aiui_record_client = rospy.ServiceProxy(AIUI_SPEECH_REQUEST_SERVICE, SrvWakeupMute)
        aiui_record_client(DO_NOT_REPLY_SKILL)

    def text_to_speak(self, text):
        try:
            rospy.wait_for_service(AIUI_TEXT_TO_SPEAK_SERVICE, timeout=SERVICE_TIMEOUT)
        except rospy.ROSException:
            log = 'request aiui text to speak error: wait for {} timeout!'.format(AIUI_TEXT_TO_SPEAK_SERVICE)
            exit(log)

        tts_client = rospy.ServiceProxy(AIUI_TEXT_TO_SPEAK_SERVICE, textToSpeakMultipleOptions)
        tts_client(text, self.vcn, self.speed, self.pitch, self.volume)

    def emotion_recognition(self, text):
        try:
            rospy.wait_for_service(EMOTION_RECOGNITION_SERVER, timeout=SERVICE_TIMEOUT)
        except rospy.ROSException:
            log = 'request emotion recognition error: wait for {} timeout!'.format(EMOTION_RECOGNITION_SERVER)
            exit(log)
        emotion_recognition_client = rospy.ServiceProxy(EMOTION_RECOGNITION_SERVER, SrvEmotionRecognition)
        emotion_list = emotion_recognition_client([text]).emotion_list
        return emotion_list[0]

    def main(self):
        try:
            while not rospy.core.is_shutdown_requested():
                self.wakeup_aiui_agent()
                iat_result = rospy.wait_for_message(AIUI_IAT_TOPIC, String).data
                emotion = self.emotion_recognition(iat_result)
                Constants.OS_MODULE.system('export PYTHONPATH=/home/lemon/robot_ros_application/catkin_ws/src/leju_lib_pkg/:$PYTHONPATH;python {}'.format(random.choice(self.emotion_reply[emotion])))
        except KeyboardInterrupt as e:
            rospy.loginfo("程序中止")

if __name__ == "__main__":
    eg = EmotionGuardian()
    eg.main()
