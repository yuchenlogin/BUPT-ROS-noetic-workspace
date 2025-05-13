# keyboards Package

## 概述
用于接受按键板被按下，并执行对应按键程序。
## 如何运行
```
rosrun keyboards keyboards_node.py
```
## 订阅的话题
- /MediumSize/SensorHub/sensor_CF1 (sensor_msgs.msg::ChannelFloat32)  
sensorhubnode节点发布的按键和主板电压的原始数据。消息定义
  ```
  string name
  float32[] values
  ```
  name传感器名称，values传感器处理后的数据。