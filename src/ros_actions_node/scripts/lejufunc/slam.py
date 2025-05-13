#!/usr/bin/python

import subprocess
from identify_digit import Identifier
from color_detect import Box_color_detecter
import rosnode

from public import PublicNode

SLAM_FOLDER = "/home/lemon/robot_ros_application/slam/"

run_slam_script = "run_slam.sh"
preview_mode = "false"
location_mode_or_create_map_mode = "true"

pubnode = PublicNode(init_node=False)

SLAM_NODE_NAME = "/RGBD"
NUMBER_OF_TIMES_TO_PING_SLAM_NODE = 3

def slaminit():
    subprocess.Popen("cd /home/lemon/robot_ros_application/catkin_ws/src/leju_lib_pkg/src/lejufunc/ && ./{} {} {} {}> /dev/null 2>&1".format(run_slam_script, preview_mode, location_mode_or_create_map_mode, SLAM_FOLDER), shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def SlamMoveTo(path_points_list=[]):
    pubnode.bodyhub_walk()
    pubnode.path_tracking(path_points_list)

def IdentifyDigit(digit):
    digit_pos = Identifier(digit).get_num_n_pos()
    return digit_pos

def ColorDetect(color):
    color_detecter = Box_color_detecter(color)
    target_box_direction = color_detecter.main()
    return target_box_direction

def check_is_kill_rosnode_success(kill_result):
    unmber_of_rosnode_kill_sucessfully = len(kill_result[0])
    return unmber_of_rosnode_kill_sucessfully

def killRGBD():
    node_is_running = rosnode.rosnode_ping(SLAM_NODE_NAME, NUMBER_OF_TIMES_TO_PING_SLAM_NODE)
    if node_is_running == True:
        kill_rosnode_result = rosnode.kill_nodes([SLAM_NODE_NAME])
        if check_is_kill_rosnode_success(kill_rosnode_result) == 0:
            return False
    return True

if __name__ == '__main__':
    slaminit()
    SlamMoveTo([[0.7, 0.0, 0.0], [1.0, 0.6, 0.0]])
    digit_pos = IdentifyDigit(4)
    target_box_direction = ColorDetect('blue')
