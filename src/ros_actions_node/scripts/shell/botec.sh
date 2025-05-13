#!/bin/bash

kill_rosnode() 
{
    cd ~/robot_ros_application/catkin_ws/
    . devel/setup.bash
    echo "softdev" | sudo -S killall roslaunch
    sleep 2
}

ar_tag_detact() 
{
    cd ~/robot_ros_application/catkin_ws/
    . devel/setup.bash
    roslaunch ar_track_alvar roban_chin_camera.launch &
}

slam()
{
    source ~/robot_ros_application/catkin_ws/devel/setup.bash
    rosrun SLAM RGBD false true $1 &
    sleep 3
}

stair_node() 
{
    cd ~/robot_ros_application/footstair/
    . devel/setup.bash
    rosrun jy901_module jy901_module_node &
    rosrun ljhn_torcon ljhn_torcon_node $1
}

pile_node() 
{
    cd ~/robot_ros_application/pilewalk/
    . devel/setup.bash
    rosrun color_detect color_detect.py &
    rosrun ljhn_torcon ljhn_torcon_node roban auto
}

botec_pose_board() 
{
    cd ~/robot_ros_application/catkin_ws/
    . devel/setup.bash
    rosrun ros_actions_node pose_board_company.py
}

botec_to_pile() 
{
    cd ~/robot_ros_application/catkin_ws/
    . devel/setup.bash
    rosrun ros_actions_node to_pile_company.py
}

slam_node_name="/RGBD"
stair_node_name="/action_node"
color_detect_node_name="/color_detect"
pile_node_name="/action_node"
ar_tag_node_name="/chin/ar_track_alvar"

slam "Slam_Map.bin"
ar_tag_detact
botec_pose_board
rosnode kill ${slam_node_name}

stair_node 3
rosnode kill ${stair_node_name}

botec_to_pile
pile_node

rosnode kill $(rosnode list | grep ${color_detect_node_name})
rosnode kill ${ar_tag_node_name}
rosnode kill ${pile_node_name}
