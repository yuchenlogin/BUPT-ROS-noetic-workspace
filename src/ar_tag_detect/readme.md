## ar_tag_detect package
### 概述
使用[ar_track_alvar](http://wiki.ros.org/ar_track_alvar/)库识别ar_tag标签，图像消息来自usb_cam功能包。
### 如何运行
- 连接usb摄像头。
- 将`camera_info`文件夹复制到`/home/[user_name]/.ros/`目录下。
- 将`ar_tag_detect`功能包放置到ros工作空间，编译工作空间。
- 终端输入
  ```
  roscore
  roslaunch usb_cam usb_cam.launch
  roslaunch roban_chin_camera.launch
  ```
### 运行结果
若摄像头视野内有ar_tag码，则发布[visualization_marker_chin](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html)消息。