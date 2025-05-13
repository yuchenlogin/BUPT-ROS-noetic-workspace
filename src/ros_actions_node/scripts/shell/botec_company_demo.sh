#!/bin/bash
script_path="/home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/shell"
slam_node_name="/RGBD"
image_publisher_node="/image_publisher"
stair_node_name="/action_node"
color_detect_node_name="/color_detect"
pile_node_name="/action_node"
ar_tag_node_name="/chin/ar_track_alvar"
jy901Module_node="/jy901Module_node"
alias sss=".  ~/robot_ros_application/catkin_ws/devel/setup.sh"

cd $script_path
start_slam()
{
    source ~/robot_ros_application/catkin_ws/devel/setup.bash
    rosrun SLAM RGBD false true &
    echo "slam starting..."
}
start_ar_tag_detact()
{
    .  ~/robot_ros_application/catkin_ws/devel/setup.bash
    roslaunch ar_track_alvar roban_chin_camera.launch &
    echo "ar_tag_detact starting"
}
killall()
{
    rosnode kill ${slam_node_name} 
    rosnode kill ${image_publisher_node}
    rosnode kill ${pile_node_name} 
    rosnode kill ${ar_tag_node_name} 
    rosnode kill ${jy901Module_node}
    rosnode kill ${color_detect_node_name}
}
killall
sleep 1
start_slam 
start_ar_tag_detact
sleep 2

sss
rosrun ros_actions_node navigation.py reset
echo -e ">>>>>>>>\npose_board starting\n"
rosrun ros_actions_node pose_board_company.py
rosnode kill ${slam_node_name} 


echo -e ">>>>>>>>\nstairs starting\n"
cd $script_path
sss
rosservice call /MediumSize/BodyHub/StateJump 2 external_control #切换状态为external_control
. stair_node_company.sh
rosnode kill ${stair_node_name}

sss
rosrun ros_actions_node navigation.py to_pile_adjust
rosservice call /MediumSize/BodyHub/StateJump 2 external_control #切换状态为external_control

killall
sleep 1
echo -e ">>>>>>>>\npile starting\n "
cd $script_path
. pile_node_company.sh

sss
rosrun ros_actions_node navigation.py walk
rosrun ros_actions_node navigation.py ready
sleep 2
killall



