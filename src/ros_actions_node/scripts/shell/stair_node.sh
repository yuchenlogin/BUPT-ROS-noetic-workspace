#!/bin/bash
stair_node() 
{
    cd ~/robot_ros_application/footstair/
    . devel/setup.bash
    rosrun jy901_module jy901_module_node &
    rosrun ljhn_torcon ljhn_torcon_node $1
}

stair_node 3