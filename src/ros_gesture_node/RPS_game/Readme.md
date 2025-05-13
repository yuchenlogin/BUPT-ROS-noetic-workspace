## 石头剪刀布游戏

### 游戏介绍

- 案例启动后，Roban 会提示 “我已经准备好，你可以喊“开始”我们开始一次游戏” 并进入就绪状态，等待用户说 “开始”

- 用户说 “开始” 后，Roban 会播报石头剪刀布的语音并出拳且播报出的拳是什么

    - 如：Roban 播报 “石头剪刀布，布” 且伴随 Roban 挥动手臂并做出 “布” 的手势

- Roban 通过头部摄像头捕捉用户出拳，并判断是哪方获胜并播报结果

- 随后恢复站立进入就绪状态并等待用户下一次说 “开始”

- 用户也可以在就绪状态说 “结束” ，停止游戏

### 案例启动

- 命令行启动

    ```shell
    $ source ~/robot_ros_application/catkin_ws/devel/setup.bash
    $ roslaunch ros_gesture_node rps_game.launch
    ```

### 案例配置

- 配置文件

    ```~/robot_ros_application/catkin_ws/src/ros_gesture_node/RPS_game/configs/config.json```

- 参数配置

    ```json
    // config.json
    {
        "label": [  // 模型使用的标签，如果模型参数改变，也应相应更改
            "paper",
            "rock",
            "scissors"
        ],
        "model_parameters": {   // 模型预测时用到的参数，如果模型参数改变，也应相应更改
            "mean": [0.485, 0.456, 0.406],
            "std": [0.229, 0.224, 0.225],
            "input_resize": {
                "width": 224,
                "height": 224
            }
        },
        "tts_params": {     // tts 参数
            "vcn": "qige",  // 发音人 [qige | xiaojuan | dangdang]
            "speed": 50,    // 语速 [0, 100]
            "pitch": 30,    // 语调 [0, 100]
            "volume": 20    // 音量 [0, 100]
        },
        "tts_text": {       // 预置的 tts 内容
            "demo_start": "我已经准备好，你可以喊“开始”我们开始一次游戏",   // 游戏开始前的提示
            "demo_end": "欢迎下次再和我玩",     // 游戏结束时的提示
            "round_start_command": "开始",      // 识别到用户说的关键字，启动一轮游戏
            "round_end_command": "结束"         // 识别到用户说的关键正，结束游戏
        },
        "rps": {
            "rock": "/home/lemon/robot_ros_application/catkin_ws/src/ros_gesture_node/RPS_game/actions/rock.py",    // 出石头的动作及语音
            "paper": "/home/lemon/robot_ros_application/catkin_ws/src/ros_gesture_node/RPS_game/actions/paper.py",  // 出布的动作及语音
            "scissors": "/home/lemon/robot_ros_application/catkin_ws/src/ros_gesture_node/RPS_game/actions/scissors.py" // 出剪刀的动作及语音
        },
        "rps_sounds": {
            "win": "我赢了",    // Roban 赢了的播报
            "lose": "Roban 输了，你好厉害",     // Roban 输了的播报
            "tie": "我们平局",      // 平局的播报
            "unrecognized": "抱歉 Roban 没认出你的手势，重新来一次好吗" // 没有识别出用户出拳的播报
        }
    }
    ```

- 出拳动作修改

    - 使用 Roban 的桌面软件制作动作，并配置相应音频

    - 通过 Roban 软件下载到机器人
    
        - 一般是以 Roban 软件新建的项目命名的 python 文件保存在 /home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/ 目录下

        - 如过没有项目名称则保存为 temp.py

    - 将该文件复制到 /home/lemon/robot_ros_application/catkin_ws/src/ros_gesture_node/RPS_game/actions/ 目录下并在 `config.json` 文件对应位置进行修改