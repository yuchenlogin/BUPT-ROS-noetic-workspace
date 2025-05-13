## 水果识别

### 配置文件

- ros_fruit_node/configs/config.json

    - 配置水果识别模型相关参数

        ```json
        {
            "label": [      // 识别的水果种类
                "apple",    // 顺序需要按照模型提供的参数排序
                "banana",
                "orange",
                "pear"
            ],
            "model_parameters": {
                "mean": [0.485, 0.456, 0.406],  // 由模型提供
                "std": [0.229, 0.224, 0.225],   // 由模型提供
                "input_resize": {   // 由模型提供
                    "width": 224,
                    "height": 224
                }
            }
        }
        ```

- ros_fruit_node/configs/fruits_introducer_config.json

    - 配置识别到水果后的语音播报

        ```json
        {
            "apple": {      // 水果名
                "text": "这是一个苹果，它是营养水果，有水果之王的美誉，是减肥瘦身的必备品，还可以加工成果酱、果脯、果干或罐头等，也可用于酿酒",
                "vcn": "qige",  // 发音人 ["qige" | "xiaojuan" | "dangdang"]
                "speed": 50,    // 语速 [0, 100]
                "pitch": 5,     // 语调 [0, 100]
                "volume": 20    // 音量 [0, 100]
            },
            "banana": {
                "text": "这是一根香蕉，它世界四大水果之一，果肉营养丰富、香甜软糯，香蕉既可生食，也可炖熟及做成香蕉干或者果脯等食用",
                "vcn": "qige",
                "speed": 50,
                "pitch": 5,
                "volume": 20
            },
            "orange": {
                "text": "这是一个橙子，它是芸香科柑橘属植物橙树的果实，亦称为黄果、柑子、金环、柳丁，果实可以剥皮鲜食其果肉，果肉可以用作其他食物的调料或附加物",
                "vcn": "qige",
                "speed": 50,
                "pitch": 5,
                "volume": 20
            },
            "pear": {
                "text": "这是一个梨，梨含有丰富的维生素和矿物质，能维持人体细胞的健康状态，因其鲜嫩多汁，酸甜适口，所以又有天然矿泉水之称",
                "vcn": "qige",
                "speed": 50,
                "pitch": 5,
                "volume": 20
            }
        }
        ```

### 案例运行

- 终端启动

    ```shell
    $ source ~/robot_ros_application/catkin_ws/devel/setup.bash
    $ roslaunch ros_fruit_node fruits_introducer.launch
    ```
