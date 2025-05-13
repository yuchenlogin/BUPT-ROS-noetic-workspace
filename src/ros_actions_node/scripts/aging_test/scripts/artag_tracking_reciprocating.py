#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys, rospkg
sys.path.append(rospkg.RosPack(["/home/lemon/robot_ros_application/catkin_ws"]).get_path("ros_actions_node") + "/scripts/stair_convert")
from stair_convert_artag import StairConvert
from bodyhub.msg import JointControlPoint
import json
TURN_HEAD_TIMES = 5
ARTAG_CONFIG_FILE = "/home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/aging_test/config/config.json"
NUMBER_OF_ARTAG = 4

class AgingArtagTracking():
    def __init__(self):
        self.sc = StairConvert(node_name="artag_tracking_reciprocating", disable_signals=True)
        self.headjointpub = rospy.Publisher('MediumSize/BodyHub/HeadPosition', JointControlPoint, queue_size=100)
        self.artag_config = self.artag_config_load()

    def artag_config_load(self):
        with open(ARTAG_CONFIG_FILE, "r") as f:
            rf = f.read()
        return json.loads(rf)

    def set_goal_pos(self):
        for index in range(NUMBER_OF_ARTAG):
            while not rospy.is_shutdown():
                raw_input('将机器人放到目标位置, 按下回车键校准...')
                if self.sc.pos_valid:
                    self.artag_config[index] = [self.sc.real_pos[0], self.sc.real_pos[1], self.sc.real_pos[2]]
                    rospy.loginfo('第 {} 个 artag 码位置: {}'.format(index+1, self.artag_config[index]))
                    if index + 1 != NUMBER_OF_ARTAG:
                        rospy.loginfo('还有 {} 个 artag 码待标定，请顺时针移动机器人到下一个目标点'.format(NUMBER_OF_ARTAG - (index + 1)))
                    break
                else:
                    rospy.logwarn('没有识别到 ar_tag 码，请移动机器人或检查摄像头！')
        with open(ARTAG_CONFIG_FILE, "w") as wf:
            json.dump(self.artag_config, wf)

    def turn_head(self, headposition=[0.0, 0.0]):
        self.headjointpub.publish(positions=headposition, mainControlID=2)

    def aging_test_artag_tracking(self):
        exec_count = 0
        goal_pos_index = 0
        self.sc.bodyhub_walk()
        while not rospy.core.is_shutdown_requested():
            for _ in range(TURN_HEAD_TIMES):
                self.turn_head([-90.0, 0.0])
                rospy.sleep(0.5)
                self.turn_head([90.0, 0.0])
                rospy.sleep(0.5)
            self.turn_head([0.0, 0.0])
            rospy.sleep(0.5)
            self.sc.goto_rot(0.0)
            self.sc.goto_pose(self.artag_config[goal_pos_index])
            self.sc.bodyhub.walking_the_distance(0.0, 0.0, 90)
            self.sc.bodyhub.wait_walking_done()
            exec_count += 1
            goal_pos_index = exec_count % len(self.artag_config)

if __name__ == "__main__":
    agingartagtracking = AgingArtagTracking()
    if len(sys.argv) == 4:
        if sys.argv[1] == "set":
            agingartagtracking.set_goal_pos()
        elif sys.argv[1] == "run":
            agingartagtracking.aging_test_artag_tracking()
