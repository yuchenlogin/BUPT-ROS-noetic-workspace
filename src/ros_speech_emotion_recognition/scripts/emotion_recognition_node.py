#!/usr/bin/env python3

from custom_constant import Constants
import rospy
from emotion_recognition import EmotionRecognition
from ros_speech_emotion_recognition.srv import SrvEmotionRecognition, SrvEmotionRecognitionResponse

class EmotionRecognitionNode(EmotionRecognition):
    def __init__(self):
        rospy.init_node("emotion_recognition_node")
        super().__init__(Constants.VOCAB_FILE_PATH, True)
        rospy.Service("/emotion_recognition", SrvEmotionRecognition, self.handle_emotion_recognition)
        rospy.spin()

    def handle_emotion_recognition(self, req):
        return SrvEmotionRecognitionResponse(self.run(req.text_list))

if __name__ == "__main__":
    ern = EmotionRecognitionNode()
