#!/bin/bash
export DISPLAY=:0.0
preview_mode=true
location_mode_or_create_map_mode=true

if [ $# -ge 2 ]
then
    preview_mode=$1
    location_mode_or_create_map_mode=$2
fi

source ~/robot_ros_application/catkin_ws/devel/setup.bash

echo "preview_mode ${preview_mode}"
echo "location_mode_or_create_map_mode ${location_mode_or_create_map_mode}"

rosrun SLAM RGBD ${preview_mode} ${location_mode_or_create_map_mode}
