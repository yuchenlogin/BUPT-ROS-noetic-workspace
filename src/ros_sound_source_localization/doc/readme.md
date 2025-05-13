## 声源定位

### 功能包介绍

- 声源定位案例演示，程序启动后，在麦克风收音范围内唤醒 Roban ，Roban 转身到声源方向，抬头识别唤醒者并向其靠近

### 参数配置

- 文件路径

    `~/robot_ros_application/catkin_ws/src/ros_sound_source_localization/config/config.json`

- 配置内容

    ```json
    {
        "facedetectionarea": {  // 摄像头图像中检测人脸的有效区域
            "left_rage": 200,   // 图像左边界
            "right_rage": 440   // 图像右边界
        },
        "detect_face_timeouts": 20,     // 人脸识别的超时时间
        "head_pitch": -20,  // 头部俯仰角度，用于抬头识别人脸
        "target_distance": 1000,    // 人与机器人的最大距离
        "action_file": "/home/lemon/robot_ros_application/catkin_ws/src/ros_sound_source_localization/actions/wave.py",     // 机器人处于最大距离内的语音播报及动作展示
        "obstacle_detection_roi": { // 摄像头图像中检测障碍物的有效区域
            "top": 180,     // 上边界
            "bottom": 480,  // 下边界
            "left": 0,      // 左边界
            "right": 640    // 右边界
        },
        "audio": {      // 语音播报
            "声源定位已开启": "/home/lemon/robot_ros_application/catkin_ws/src/ros_sound_source_localization/voice/demo_start.mp3",
            "未检测到人脸": "/home/lemon/robot_ros_application/catkin_ws/src/ros_sound_source_localization/voice/face_not_found.mp3",
            "我在": "/home/lemon/robot_ros_application/catkin_ws/src/ros_sound_source_localization/voice/im_here.mp3"
        }
    }
    ```

- 音频合成

    - 可以通过 https://www.lejuhub.com/Talos/tts_flyos/-/tree/tts_aiui 这个项目获取

- 挥手及播报

    - 通过桌面软件调试动作

    - 通过上一步音频合成播报的语音，添加进动作中

        - 由于上一步获得的音频是 mp3 格式，桌面软件仅支持 wav 格式，可以使用以下脚本从 mp3 转换成 wav

            ```shell
            $ python3 /home/lemon/robot_ros_application/catkin_ws/src/leju_lib_pkg/scripts/audio_transform.py <mp3 文件路径>
            ```

### 案例启动

- 准备工作

    - 需要外接 jy901

- 案例运行

    ```shell
    $ source ~/robot_ros_application/catkin_ws/devel/setup.bash
    $ roslaunch ros_sound_source_localization sound_source_localization.launch
    ```
