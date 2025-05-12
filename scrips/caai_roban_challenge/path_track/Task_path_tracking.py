#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
import math
import time
import numpy as np
import rospy
import rospkg
import yaml

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
sys.path.append(rospkg.RosPack().get_path('ros_higher_vocational_training_platform') + "/scripts")
# sys.path.append(rospkg.RosPack().get_path('higher_vocational_schools') + "/scripts")


from std_msgs.msg import *
from geometry_msgs.msg import *
import bodyhub_action as bodact

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
# from lejulib import *
from frames import RobanFrames
from public import PublicNode
SCRIPTS_PATH=os.path.split(sys.argv[0])[0]

# #!/usr/bin/env python
# # -*- coding: utf-8 -*-
# import sys
# import os
# import math
# import time
# import numpy as np
# import rospy
# import rospkg
# import yaml
# from std_msgs.msg import *
# from geometry_msgs.msg import *
# import bodyhub_action as bodact

# sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
# from lejulib import *
# from frames import RobanFrames
# from public import PublicNode
# SCRIPTS_PATH=os.path.split(sys.argv[0])[0]

NODE_NAME = 'path_tracking_node'
CONTROL_ID = 6
with open(os.path.join(SCRIPTS_PATH,"slam_path_tracking.yaml"),"r")as f:
    SLAM_POINT=yaml.load(f)
    print(SLAM_POINT)
class Task_path_tracking(PublicNode):
    def __init__(self,nodename=NODE_NAME,control_id=CONTROL_ID):
        super(Task_path_tracking, self).__init__(nodename, control_id)

    def start_path_tracking(self):
        self.set_arm_mode(1)
        self.bodyhub_walk()
        # self.path_tracking(SLAM_POINT["path_tracking_points"],pos_wait_mode=1,wait_time=0.1,pos_exit_high_acc=False)
        self.path_tracking(pos_wait_mode=1,wait_time=0.2,pos_exit_high_acc=False)

    # class Task_path_tracking(PublicNode):
    #     def __init__(self,nodename=NODE_NAME,control_id=CONTROL_ID):
    #         super(Task_path_tracking, self).__init__(nodename, control_id)
    #         rospy.loginfo("等待服务启动...")
    #         # 加入延时，单位是秒，可以根据需要调整时间长度
    #         rospy.sleep(5.0) # 在ROS节点环境里推荐用 rospy.sleep()
    #         # 或者 time.sleep(5.0) # 如果 rospy 可能还没完全就绪，可以用这个，但不太推荐
    #         rospy.loginfo("等待结束，继续执行任务。")

    #     def start_path_tracking(self):
    #         # 如果 __init__ 里的延时不够，这里也可以再加一个短暂延时
    #         # rospy.sleep(2.0)
    #         rospy.loginfo("正在设置手臂模式...")
    #         self.set_arm_mode(1) # 现在再尝试调用
    #         rospy.loginfo("手臂模式设置完毕，开始行走。")
    #         self.bodyhub_walk()
    #         rospy.loginfo("开始路径跟踪。")

if __name__ == '__main__':
    task=Task_path_tracking()
    task.start_path_tracking()
