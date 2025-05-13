import os
import rospkg

class Constants:
    config_file = os.path.join(rospkg.RosPack().get_path('ros_gesture_node'), 'RPS_game', 'configs', 'config.json')
