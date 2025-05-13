import os
import rospkg
from enum import IntEnum
import sys

class Emotion(IntEnum):
    ANGRY = 0
    HAPPY = 1
    NEUTRAL = 2
    SURPRISE = 3
    SAD = 4
    FEAR = 5

class Constants:
    EMOTION_RECOGNITION_MODEL_FILE_NAME = 'speech_emotion_recognition.onnx'
    EMOTION_RECOGNITION_FOLDER = rospkg.RosPack().get_path('ros_speech_emotion_recognition')
    EMOTION_RECOGNITION_MODEL_FOLDER = os.path.join(EMOTION_RECOGNITION_FOLDER, 'models')
    EMOTION_RECOGNITION_MODEL = os.path.join(EMOTION_RECOGNITION_MODEL_FOLDER, EMOTION_RECOGNITION_MODEL_FILE_NAME)
    EMOTION_RECOGNITION_MODEL_MD5 = "99116ce213564de407d292ffd28fcad0"
    MODEL_DOWNLOAD_URL = "https://roban.lejurobot.com/modols/speech_emotion_recognition.onnx"
    VOCAB_FILE_PATH = os.path.join(EMOTION_RECOGNITION_FOLDER, 'configs', 'vocab.txt')
    MAX_LEN = 256
    ID2LABEL = {Emotion.ANGRY: "angry", Emotion.HAPPY: "happy", Emotion.NEUTRAL: "neutral", Emotion.SURPRISE: "surprise", Emotion.SAD: "sad", Emotion.FEAR: "fear"}
    OS_MODULE = os
    ROSPKG_MODULE = rospkg
    SYS_MODULE = sys
