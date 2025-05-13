## 情绪识别开发文档

### 案例实现

- 识别用户语音，转成文字

- 分析这段文字的情绪

- 根据不同的情绪反馈不同的语音和动作

### 可识别情绪

- 开心 happy

- 恐惧 fear

- 难过 sad

- 平静 neutral

- 生气 angry

- 惊讶 surprise

- 情绪识别相关的音乐文件在/home/lemon/robot_ros_application/Music路径下，如果路径下没有音乐文件则提取emotion.rar压缩包到当前路径下即可。
### 参数配置

- 配置文件

  - ```/home/lemon/robot_ros_application/catkin_ws/src/ros_speech_emotion_recognition/configs/config.json```

    ```json
    {
        "tts_params": {     // 文字转语音参数
            "vcn": "qige",  // 发音人 [qige | xiaojuan | dangdang]
            "speed": 50,    // 语速 [0-100]
            "pitch": 30,    // 语调 [0-100]
            "volume": 20    // 音量 [0-100]
        },
        "tts_text": {
            "demo_start": "情绪识别已开启，我会根据语音识别出情绪，请保证我可以听到你的声音"    // 案例启动的提示，通过 tts 将文字转成语音并播放
        },
        "emotion_reply": {      // 相应情绪的反馈
            "neutral": [
                "/home/lemon/robot_ros_application/catkin_ws/src/ros_speech_emotion_recognition/actions/neutral.py"
            ],
            "happy": [
                "/home/lemon/robot_ros_application/catkin_ws/src/ros_speech_emotion_recognition/actions/happy.py"
            ],
            "fear": [
                "/home/lemon/robot_ros_application/catkin_ws/src/ros_speech_emotion_recognition/actions/fear.py"
            ],
            "angry": [
                "/home/lemon/robot_ros_application/catkin_ws/src/ros_speech_emotion_recognition/actions/angry.py"
            ],
            "sad": [
                "/home/lemon/robot_ros_application/catkin_ws/src/ros_speech_emotion_recognition/actions/sad.py"
            ],
            "surprise": [
                "/home/lemon/robot_ros_application/catkin_ws/src/ros_speech_emotion_recognition/actions/surprise.py"
            ]
        }
    }
    ```

### 自定义情绪反馈

- 音频合成

    - 可以通过 https://www.lejuhub.com/Talos/tts_flyos/-/tree/tts_aiui 这个项目获取

    - 由于获得的音频是 mp3 格式，桌面软件仅支持 wav 格式，可以使用以下脚本从 mp3 转换成 wav

        ```shell
        $ python3 /home/lemon/robot_ros_application/catkin_ws/src/leju_lib_pkg/scripts/audio_transform.py <mp3 文件路径>
        ```

- 自定义动作及播报

    - 导入合成的回复语音频，设计相应的动作

    - 导出模块并下载到机器人中

    - 将下载的文件放至 ```/home/lemon/robot_ros_application/catkin_ws/src/ros_speech_emotion_recognition/actions```

    - 在 config.json 中对应位置添加该自定义的动作文件

### 案例运行

- 运行命令

    ```shell
    $ source ~/robot_ros_application/catkin_ws/devel/setup.bash

    $ roslaunch ros_speech_emotion_recognition emotion_guardian.launch
    ```
