#!/usr/bin/env python
# coding=utf-8

import rospy
from ros_mic_arrays.srv import setBeamSrv
from std_srvs.srv import Empty, SetBool
from std_msgs.msg import String
from ros_AIUI_node.srv import textToSpeakMultipleOptions, SrvWakeupMute

SET_MIC_BEAM_SERVICE = '/ros_mic_arrays/set_real_beam'
SERVICE_TIMEOUT = 2
BEAM_INDEX_FORWARD = 5
AIUI_SPEECH_REQUEST_SERVICE = '/aiui/wakeup_mute'
AIUI_IAT_TOPIC = '/aiui/iat'
AIUI_STOP_SPEECH_REQUEST_SERVICE = '/aiui/stop_recording'
PAUSE = True
RESUME = False
AIUI_TEXT_TO_SPEAK_SERVICE = '/aiui/text_to_speak_multiple_options'
VCN_DICT = {
	'male': {
		'CN': 'qige',
		'EN': 'qige'
	},
	'female': {
		'CN': 'xiaojuan',
		'EN': 'xiaojuan'
	}
}

pause_head_toward_sound_client = rospy.ServiceProxy("/aiui/pause_head_toward_sound", SetBool)

def set_head_toward_sound_status(is_pause):
	pause_head_toward_sound_client(is_pause)

class AIUIError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)

def punctuation_filter(text):
	punctuation_list = ["，", "。", "？", "！", "：", "“", "”", "《", "》", "；", "、"]
	after_process_text = text

	for p in punctuation_list:
		after_process_text = after_process_text.replace(p, "")

	return after_process_text


def speech_to_text(text):
	"""
	Transfer speech to text, match target words / sentence

	Return:
	  True / False
	"""

	try:
		set_head_toward_sound_status(PAUSE)
		try:
			rospy.wait_for_service(SET_MIC_BEAM_SERVICE, timeout=SERVICE_TIMEOUT)
		except rospy.ROSException:
			log = 'set mic beam error: wait for {} timeout!'.format(SET_MIC_BEAM_SERVICE)
			rospy.logerr(log)
			raise AIUIError(log)
		set_beam_client = rospy.ServiceProxy(SET_MIC_BEAM_SERVICE, setBeamSrv)
		set_beam_client(BEAM_INDEX_FORWARD)

		try:
			rospy.wait_for_service(AIUI_SPEECH_REQUEST_SERVICE, timeout=SERVICE_TIMEOUT)
		except rospy.ROSException:
			log = 'request aiui record error: wait for {} timeout!'.format(AIUI_SPEECH_REQUEST_SERVICE)
			rospy.logerr(log)
			raise AIUIError(log)
		aiui_record_client = rospy.ServiceProxy(AIUI_SPEECH_REQUEST_SERVICE, SrvWakeupMute)
		aiui_record_client(False)

		try:
			msg = rospy.wait_for_message(AIUI_IAT_TOPIC, String)
		except:
			rospy.ServiceProxy(AIUI_STOP_SPEECH_REQUEST_SERVICE, Empty)()
			raise AIUIError('')
		result = msg.data
		result = punctuation_filter(result)
		target_text = punctuation_filter(text)
		return result == target_text
	except (KeyboardInterrupt, AIUIError):
		return False
	finally:
		set_head_toward_sound_status(RESUME)


def text_to_speech(language, gender, text, speed=50, pitch=5, volume=20):
	"""
	Transfer text to speech

	Args:
		language: target language, CN / EN
		gender: target gender, male / female
		text: target text
		speed: speed of speech	[0, 100]
		pitch: tone		[0, 100]
		volume: tone	[0, 100]

	No return, play the audio file instead
	"""

	try:
		rospy.wait_for_service(AIUI_TEXT_TO_SPEAK_SERVICE, timeout=SERVICE_TIMEOUT)
	except rospy.ROSException:
		log = 'request aiui text to speak error: wait for {} timeout!'.format(AIUI_TEXT_TO_SPEAK_SERVICE)
		rospy.logerr(log)
		return

	vcn = VCN_DICT[gender][language]
	tts_client = rospy.ServiceProxy(AIUI_TEXT_TO_SPEAK_SERVICE, textToSpeakMultipleOptions)
	tts_client(text, vcn, speed, pitch, volume)

if __name__ == '__main__':
	rospy.init_node('speech_test', anonymous=True)

	try:
		result = speech_to_text("你，叫。什么名字？？？？？")
		print(result)
	except Exception as err:
		rospy.logerr(err)
