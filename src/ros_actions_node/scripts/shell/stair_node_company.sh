#!/bin/bash
stair_node() 
{
    # . ~/robot_ros_application/catkin_ws/devel/setup.bash
    # rosrun ros_actions_node navigation.py 1
    # sleep 3
    rosnode kill /jy901Module_node 
    . ~/robot_ros_application/footstair/devel/setup.bash
    rosrun jy901_module jy901_module_node &
    rosrun ljhn_torcon ljhn_torcon_node $1
}

stair_node 3