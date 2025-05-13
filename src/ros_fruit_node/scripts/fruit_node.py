#! /usr/bin/python3

import rospy, rospkg
import os, sys
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ros_fruit_node.srv import SetConfidence
from fruit import FruitClassification

IMAGE_TOPIC = '/camera/color/image_raw'
FRUIT_CONFIDENCE_THRESHOLD = 0.9
FRUIT_RECOGNIZER_PUBLISHER = '/fruit_recognizer_node/fruit_result'
FRUIT_RECOGNIZER_SET_CONFIDENCE_SERVER = '/fruit_recognizer_node/fruit_result'

class FruitRecognizerNode:
    def __init__(self):
        rospy.init_node('fruit_recognizer_node', anonymous=True)
        self.cv_bridge = CvBridge()
        self.fruit_classifier = FruitClassification()
        self.fruit_classifier.load_model()
        self.confidence = rospy.get_param('~confidence', FRUIT_CONFIDENCE_THRESHOLD)
        self.fruit_pub = rospy.Publisher(FRUIT_RECOGNIZER_PUBLISHER, String, queue_size=1)
        self.set_confidence_service = rospy.Service(FRUIT_RECOGNIZER_SET_CONFIDENCE_SERVER, SetConfidence, self.set_confidence_callback)

    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        result = self.fruit_classifier.run(cv_image, self.confidence)
        if result is not None:
            self.fruit_name = result

    def set_confidence_callback(self, req):
        self.confidence = req.confidence
        return {'success': True}

    def run(self):
        while not rospy.is_shutdown():
            try:
                msg = rospy.wait_for_message(IMAGE_TOPIC, Image, timeout=1.0)
                self.image_callback(msg)
            except rospy.exceptions.ROSException:
                continue
            if hasattr(self, 'fruit_name'):
                self.fruit_pub.publish(self.fruit_name)
                delattr(self, 'fruit_name')

if __name__ == '__main__':
    try:
        node = FruitRecognizerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
