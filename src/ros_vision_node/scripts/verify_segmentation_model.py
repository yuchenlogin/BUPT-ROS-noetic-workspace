#!/usr/bin python3
import rospy
from sensor_msgs.msg import Image
import sys, rospkg, os
sys.path.append(os.path.join(rospkg.RosPack().get_path('ros_vision_node'), 'src'))
import utils
from cv_bridge import CvBridge
import cv2 as cv
from detect_ball_in_area import DetectBallInArea
import numpy as np

CHIN_CAMERA_TOPIC = '/chin_camera/image'
HEAD_CAMERA_TOPIC = '/camera/color/image_raw'
LABEL_MAINAREA = 1
LABEL_BALL = 2
KERNEL_SIZE = (9, 9)
YELLOW_COLOR = (0, 255, 255)
BLUE_COLOR = (255, 0, 0)
GREEN_COLOR = (0, 255, 0)

class VerifySegmentationModel(DetectBallInArea):
    def __init__(self, camera_rgb_topic):
        super(VerifySegmentationModel, self).__init__()
        rospy.init_node('verify_segmentation_model_node', anonymous=False)
        self.bridge = CvBridge()
        self.topic = camera_rgb_topic
        self.load_model()

    def eval(self, img):
        rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        rgb_transpose = rgb.transpose(2, 0, 1)
        input_image = self._DetectBallInArea__normalize(rgb_transpose, 0.5, 0.5)
        input_feed = {self.model.get_inputs()[0].name: [input_image]}

        results = self.model.run(None, input_feed)

        result = np.squeeze(results)

        main_area_mask = (result==LABEL_MAINAREA).astype(np.uint8)
        ball_mask = (result==LABEL_BALL).astype(np.uint8)

        kernel = np.ones(KERNEL_SIZE, np.uint8)
        dilated_main_area_mask = cv.dilate(main_area_mask, kernel, iterations=1)

        ball_mask_colored = cv.cvtColor(ball_mask, cv.COLOR_GRAY2BGR)
        dilated_main_area_mask_colored = cv.cvtColor(dilated_main_area_mask, cv.COLOR_GRAY2BGR)

        ball_mask_colored[ball_mask > 0] = YELLOW_COLOR
        dilated_main_area_mask_colored[dilated_main_area_mask > 0] = BLUE_COLOR

        intersection = cv.bitwise_and(ball_mask_colored, dilated_main_area_mask_colored)
        intersection_colored = np.zeros_like(ball_mask_colored)
        intersection_colored[intersection[:,:,1] > 0] = GREEN_COLOR

        result = cv.addWeighted(ball_mask_colored, 0.5, dilated_main_area_mask_colored, 0.5, 0)
        result = cv.addWeighted(result, 1, intersection_colored, 1, 0)
        return result

    def main(self):
        while not rospy.is_shutdown():
            image_msg = rospy.wait_for_message(self.topic, Image, timeout=2)
            image_origin = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            image_result = self.eval(image_origin)
            cv.imshow('image_result', image_result)
            if cv.waitKey(1) == ord('q'):
                break

if __name__ == '__main__':
    vsm = VerifySegmentationModel(CHIN_CAMERA_TOPIC)
    vsm.main()
