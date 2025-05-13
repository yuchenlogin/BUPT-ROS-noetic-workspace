#!/bin/bash
pile_node() 
{
    cd ~/robot_ros_application/pilewalk/
    . devel/setup.bash
    rosrun color_detect color_detect.py &
    rosrun ljhn_torcon ljhn_torcon_node roban auto
}

pile_node