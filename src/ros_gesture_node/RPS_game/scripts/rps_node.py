#!/usr/bin/env python3

import rospy
from rock_paper_scissors import RockPaperScissors
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image

GESTURE_CONFIDENCE_THRESHOLD = 0.7
RPS_PUBLISHER = '/rps_node/gesture'
IMAGE_TOPIC = '/camera/color/image_raw'

class RockPaperScissorsNode():
    def __init__(self):
        rospy.init_node("rock_paper_scissors_node")
        self.rps = RockPaperScissors()
        self.cv_bridge = CvBridge()
        self.confidence = rospy.get_param('~gesture_confidence', GESTURE_CONFIDENCE_THRESHOLD)
        self.rps_pub = rospy.Publisher(RPS_PUBLISHER, String, queue_size=1)

    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        result = self.rps.run(cv_image, self.confidence)
        if result != "None":
            self.rps_is = result

    def run(self):
        while not rospy.core.is_shutdown_requested():
            try:
                msg = rospy.wait_for_message(IMAGE_TOPIC, Image, timeout=1.0)
                self.image_callback(msg)
            except rospy.exceptions.ROSException:
                continue
            if hasattr(self, 'rps_is'):
                self.rps_pub.publish(self.rps_is)
                delattr(self, 'rps_is')

if __name__ == "__main__":
    try:
        rspn = RockPaperScissorsNode()
        rspn.run()
    except rospy.ROSInterruptException:
        pass
