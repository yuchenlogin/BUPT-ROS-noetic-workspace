#!/usr/bin/env python
# coding=utf-8

# 模块导入
import rospy
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import SetBool
from ros_vision_node.srv import FaceDetectService
import json
import sys, rospkg, os
from cv_bridge import CvBridge
import numpy as np
import pyrealsense2
import math
from threading import Thread
import subprocess

# 添加 leju_lib_pkg 环境变量
sys.path.append(os.path.join(rospkg.RosPack().get_path('leju_lib_pkg'), 'src'))
from motion import bodyhub_client as bodycli
from lejufunc.bezier import send_custom_bezier

# 常量定义
WAKEUP_TOPIC = "/micarrays/wakeup"
JY901_TOPIC = "/jy901Module_node/jy901Data"
FACE_DETCT_SERVICE = '/ros_vision_node/face_detect'
DEPTH_IMAGE_TOPIC = '/camera/depth/image_rect_raw'
PAUSE = True
RESUME = False
CONTROLID = 2
CAL_DIFF_YAW_THRESHOLD = 5
JY901_REFRESH_TIMEOUT = 2
CONFIG_FILE_RELATIVE_PATH = "../config/config.json"
ERROR = 'Error'
DETECT_RATE = 1
FURTHEST_DETECTION = 3000
MILLIMETER_TO_METER = 0.001
HEAD_REVERSION_ANGLE = 0

CONFIG_FACEDETECTIONAREA_KEY = 'facedetectionarea'
CONFIG_LEFT_RAGE_KEY = 'left_rage'
CONFIG_RIGHT_RAGE_KEY = 'right_rage'
FACE_DETECT_LOCATION_KEY = 'location'
UPPER_BOUND = 0
LOWER_BOUND = 2
LEFT_BOUND = 3
RIGHT_BOUND = 1
DETECT_FACE_TIMEOUTS_KEY = 'detect_face_timeouts'
HEAD_PITCH_KEY = 'head_pitch'
TARGET_DISTANCE_KEY = 'target_distance'
OBSTACLE_ROI_KEY = 'obstacle_detection_roi'
AUDIO_KEY = 'audio'

DEMO_START = '声源定位已开启'
FACE_NOT_FOUND = '未检测到人脸'
IM_HERE = '我在'

def relative_path_to_absolute_path(relative_path):
    """
    将相对路径转换为绝对路径。

    Args:
        relative_path: str, 相对路径。

    Returns:
        abs_path: str, 绝对路径

    """
    abs_path = os.path.abspath(os.path.join(os.path.dirname(__file__), relative_path))
    return abs_path

def convert_depth_to_phys_coord_using_realsense(x, y, depth, cameraInfo):
    """
    将深度图像上的像素点坐标转换为物理坐标。

    Args:
        x: int，像素点的 x 坐标。
        y: int，像素点的 y 坐标。
        depth: float，深度图像上的像素点深度值。
        cameraInfo: rs2_intrinsics，相机参数信息。

    Returns:
        一个包含三个元素的元组，分别为物理坐标的 z、x、y 值。

    """
    # 创建相机内参对象
    _intrinsics = pyrealsense2.intrinsics()
    # 设置相机内参的宽和高
    _intrinsics.width = cameraInfo.width
    _intrinsics.height = cameraInfo.height
    # 设置相机内参的主点坐标
    _intrinsics.ppx = cameraInfo.K[2]
    _intrinsics.ppy = cameraInfo.K[5]
    # 设置相机内参的焦距
    _intrinsics.fx = cameraInfo.K[0]
    _intrinsics.fy = cameraInfo.K[4]
    # 设置相机内参的畸变模型和畸变参数
    _intrinsics.model  = pyrealsense2.distortion.none
    _intrinsics.coeffs = [i for i in cameraInfo.D]
    # 使用相机内参将像素点坐标和深度值转换为物理坐标
    result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)
    # 返回物理坐标的 z、x、y 值
    #result[0]: right, result[1]: down, result[2]: forward
    # 注意：由于相机内部坐标系和物理坐标系的差异，result[0] 对应的是物理坐标系下的 x 坐标，result[1] 对应的是物理坐标系下的 y 坐标，result[2] 对应的是物理坐标系下的 z 坐标。
    return result[2], result[0], result[1]

class SoundSourceLocalization(bodycli.BodyhubClient):
    """
    声源定位类。继承自 bodycli.BodyhubClient 类。

    """

    def __init__(self):
        """
        构造函数，初始化 ROS 节点和相关参数。

        """
        # 初始化 ROS 节点
        rospy.init_node("ros_sound_source_localization")
        # 调用父类的构造函数
        super(SoundSourceLocalization, self).__init__(CONTROLID)
        # 加载配置文件
        self.config = self.load_config(relative_path_to_absolute_path(CONFIG_FILE_RELATIVE_PATH))
        # 播放开始提示音
        self.play_tts_audio(DEMO_START)
        # 初始化 CvBridge，用于图像数据的转换
        self.cvBridge = CvBridge()
        # 初始化服务代理，用于暂停 AIUI 服务和头部转向声源服务
        self.pause_aiui_server_client = rospy.ServiceProxy("/aiui/pause_aiui_server", SetBool)
        self.pause_head_toward_sound_client = rospy.ServiceProxy("/aiui/pause_head_toward_sound", SetBool)
        # 将 AIUI 服务状态设置为暂停
        self.set_aiui_server_status(PAUSE)
        # 订阅唤醒话题，用于接收唤醒指令并开始声源定位
        rospy.Subscriber(WAKEUP_TOPIC, String, self.wakeup_cb, queue_size=1)
        # 记录头部舵机的偏航角度
        self.torso_yaw = None
        self.torso_yaw_is_refreshed = False
        rospy.Subscriber(JY901_TOPIC, Float64MultiArray, self.jy901_cb, queue_size=1)
        # 初始化人脸检测服务代理
        self.face_detection_client = rospy.ServiceProxy(FACE_DETCT_SERVICE, FaceDetectService)
        # 注册 ROS 关闭时的回调函数
        rospy.on_shutdown(self.rosShutdownHook)
        # 设置检测频率
        self.rate = rospy.Rate(DETECT_RATE)
        # 记录相机内参信息
        self.cameraInfo = None
        rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.al_cam_info_callback)
        # 记录障碍物检测次数和安全行走状态
        self.obstacle_detection_count = 0
        self.safe_to_walk = True
        # 订阅深度图像话题，用于障碍物检测
        rospy.Subscriber(DEPTH_IMAGE_TOPIC, Image, self.depth_image_cb, queue_size=1)
        # 设置音频文件路径
        self.audio_path = ''
        # 进入 ROS 事件循环
        rospy.spin()

    def depth_image_cb(self, msg):
        """
        深度图像话题的回调函数，用于障碍物检测。

        Args:
            msg: 深度图像话题的消息。

        """
        # 将 ROS 消息转换为 OpenCV 格式
        cv_image = self.cvBridge.imgmsg_to_cv2(msg, msg.encoding)
        cv_image = np.array(cv_image)
        # 获取障碍物 ROI 区域
        top = self.config[OBSTACLE_ROI_KEY]['top']
        bottom = self.config[OBSTACLE_ROI_KEY]['bottom']
        left = self.config[OBSTACLE_ROI_KEY]['left']
        right = self.config[OBSTACLE_ROI_KEY]['right']
        roi_image = cv_image[top : bottom, left : right]
        # 根据深度值生成掩码
        mask = (roi_image < 500) & (roi_image != 0)
        count = np.count_nonzero(mask)
        # 如果障碍物数量超过阈值，则认为存在障碍物
        if count > 1000:
            self.obstacle_detection_count += 1
            if self.obstacle_detection_count > 3:
                # 如果障碍物检测次数超过三次，则设置为不安全行走状态并停止行走
                self.safe_to_walk = False
                self.stop_walking()
        # 否则，认为不存在障碍物，设置为安全行走状态
        else:
            self.safe_to_walk = True
            self.obstacle_detection_count = 0

    def al_cam_info_callback(self, msg):
        """
        相机内参话题的回调函数，用于获取相机内参信息。

        Args:
            msg: 相机内参话题的消息。

        """
        if self.cameraInfo == None:
            self.cameraInfo = msg

    def load_config(self, file_path):
        """
        加载配置文件

        Args:
            file_path: 配置文件路径。

        Returns:
            配置文件中的参数字典。

        """
        return json.loads(open(file_path).read())

    def rosShutdownHook(self):
        """
        ROS 关闭时的回调函数，用于停止音频播放和重置机器人状态。

        """
        if self.audio_path != '':
            # 停止音频播放
            os.system("ps -auxww | grep {} | awk '{{print $2}}' | xargs kill".format(self.audio_path))
        # 恢复 AIUI 服务状态
        self.set_aiui_server_status(RESUME)
        # 重置机器人状态
        self.reset()

    def play_tts_audio(self, text):
        """
        播放 TTS 语音。

        Args:
            text: 待播放的文本。

        """
        # 获取音频文件路径
        self.audio_path = self.config[AUDIO_KEY][text.encode('utf-8').decode('utf-8')]
        # 在新线程中播放音频
        Thread(target=self.__play).start()

    def __play(self):
        """
        播放音频的私有函数。

        """
        if os.path.exists(self.audio_path):
            subprocess.Popen("play -q {}".format(self.audio_path), shell=True)

    def cal_micarrays_angle(self, angle):
        """
        计算麦克风阵列的角度。

        Args:
            angle: 麦克风阵列的原始角度。

        Returns:
            转换后的麦克风阵列角度。

        """
        # 将原始角度转换为 0~360 度范围内的角度
        calculated_angle = (300 - angle + 360) % 360
        # 如果角度大于180，则减去 360，得到负数角度；否则返回转换后的角度
        return calculated_angle - 360 if calculated_angle > 180 else calculated_angle

    def wakeup_cb(self, msg):
        """
        唤醒话题的回调函数，用于响应唤醒事件。

        Args:
            msg: 唤醒话题的消息。

        """
        # 将唤醒数据转换为字典格式
        wakeup_data = json.loads(msg.data.replace("'", '"'))
        # 获取声源角度并播放“我在”的 TTS 语音
        sound_source_angle = int(wakeup_data['angle'])
        self.play_tts_audio(IM_HERE)
        # 将原始角度转换为麦克风阵列的角度，并转向声源
        calculated_angle = self.cal_micarrays_angle(sound_source_angle)
        self.turn_to_sound_source(calculated_angle)
        # 进入人脸检测状态
        self.move_to_face_detection()

    def move_to_face_detection(self):
        """
        进入人脸检测状态，用于寻找人脸并直行到目标距离。

        """
        # 获取当前时间戳和检测标识符
        timestamp = rospy.Time.now()
        is_detect_flag = False
        # 将头部控制至固定角度
        self.head_control(self.config[HEAD_PITCH_KEY])
        # 循环检测人脸直至超时或检测到人脸
        while not rospy.core.is_shutdown_requested():
            # 如果超时则播放“未检测到人脸”的 TTS 语音并跳出循环
            if rospy.Time.now() - timestamp > rospy.Duration.from_sec(self.config[DETECT_FACE_TIMEOUTS_KEY]):
                self.play_tts_audio(FACE_NOT_FOUND)
                break
            # 获取深度图像和人脸检测结果
            depth_image = rospy.wait_for_message(DEPTH_IMAGE_TOPIC, Image)
            service_res = self.face_detection_client("")
            face_detect_result = json.loads(service_res.result)
            # 如果读取摄像头失败，则记录错误日志
            if face_detect_result is ERROR:
                rospy.logerr('读取摄像头失败')
            # 如果未识别到人脸，则记录错误日志
            elif face_detect_result == []:
                rospy.logerr('未识别到人脸')
            # 如果识别到人脸，则对检测结果进行筛选
            else:
                face_detection_filter_by_picture_area = self.filter_face_detection_by_picture_area(face_detect_result)
                # 如果筛选后的结果不为空，则将检测标识符置为 True 并跳出循环
                if len(face_detection_filter_by_picture_area):
                    is_detect_flag = True
                    break
            # 休眠一段时间
            self.rate.sleep()
        # 将头部控制回初始角度
        self.head_control(HEAD_REVERSION_ANGLE)
        # 如果检测到人脸，则计算距离并直行到目标距离
        if is_detect_flag:
            closest_detection_distance = self.filter_face_detection_closest_distance(face_detection_filter_by_picture_area, depth_image)
            walk_straight = (closest_detection_distance - self.config[TARGET_DISTANCE_KEY]) * MILLIMETER_TO_METER
            # 如果直行距离大于 0 并且允许行走，则直行到目标距离
            if walk_straight > 0 and self.enable_walk:
                # 开始行走，并直行到目标距离
                self.walk()
                self.walking_the_distance(walk_straight, 0, 0)
                self.wait_walking_done()
                # 行走结束后进入准备状态
                self.ready()
            # 执行动作文件
            os.system('export PYTHONPATH=/home/lemon/robot_ros_application/catkin_ws/src/leju_lib_pkg/:$PYTHONPATH;python {}'.format(self.config['action_file']))

    @property
    def enable_walk(self):
        """
        返回是否允许机器人行走的布尔值。

        """
        # 获取当前时间戳
        timestamp = rospy.Time.now()
        # 循环判断是否允许机器人行走
        while not rospy.core.is_shutdown_requested():
            # 如果时间未超过 1 秒，则返回 True
            if rospy.Time.now() - timestamp < rospy.Duration.from_sec(1):
                return True
            # 如果不允许行走，则返回 False
            if not self.safe_to_walk:
                return False
            # 休眠一段时间
            rospy.sleep(0.1)
        # 如果出现异常，则返回 False
        return False

    def filter_face_detection_by_picture_area(self, face_detection):
        """
        通过图片区域筛选人脸检测结果。

        Args:
            face_detection: 人脸检测结果。

        Returns:
            face_detection_filter_by_picture_area: 经过筛选后的人脸检测结果。

        """
        # 初始化人脸检测结果列表
        face_detection_filter_by_picture_area = []
        # 获取图片区域
        picture_area_left = self.config[CONFIG_FACEDETECTIONAREA_KEY][CONFIG_LEFT_RAGE_KEY]
        picture_area_right = self.config[CONFIG_FACEDETECTIONAREA_KEY][CONFIG_RIGHT_RAGE_KEY]
        # 遍历人脸检测结果，并将符合图片区域的结果加入列表
        for detection in face_detection:
            if detection[FACE_DETECT_LOCATION_KEY][FACE_DETECT_LOCATION_KEY][LEFT_BOUND] > picture_area_left and detection[FACE_DETECT_LOCATION_KEY][FACE_DETECT_LOCATION_KEY][RIGHT_BOUND] < picture_area_right:
                face_detection_filter_by_picture_area.append(detection)
        # 返回经过筛选后的人脸检测结果
        return face_detection_filter_by_picture_area

    def filter_face_detection_closest_distance(self, face_list, depth_image):
        """
        通过深度图像计算最近人脸距离。

        Args:
            face_list: 经过筛选后的人脸检测结果。
            depth_image: 深度图像。

        Returns:
            min_distance: 最近人脸距离。

        """
        # 初始化最小距离为正无穷
        min_distance = float('inf')
        # 将深度图像转化为 OpenCV 格式
        cv_image = np.array(self.cvBridge.imgmsg_to_cv2(depth_image, depth_image.encoding))
        # 遍历经过筛选后的人脸检测结果，并计算最近距离
        for detection in face_list:
            # 获取人脸位置信息
            face_location = detection[FACE_DETECT_LOCATION_KEY][FACE_DETECT_LOCATION_KEY]
            left_bound, right_bound, top_bound, lower_bound = face_location[LEFT_BOUND], face_location[RIGHT_BOUND], face_location[UPPER_BOUND], face_location[LOWER_BOUND]
            # 初始化坐标数组
            x_coords, y_coords, z_coords = [], [], []
            # 遍历人脸像素点，并计算其三维物理坐标，将其加入对应的数组中
            for y_pixel in range(top_bound, lower_bound+1):
                for x_pixel in range(left_bound, right_bound+1):
                    # 如果深度值为 0 或者超过最远检测范围，则忽略该像素点
                    if cv_image[y_pixel, x_pixel] == 0 or cv_image[y_pixel, x_pixel] > FURTHEST_DETECTION: continue
                    # 将像素坐标转换为物理坐标，并添加到对应的数组中
                    x, y, z = convert_depth_to_phys_coord_using_realsense(x_pixel, y_pixel, cv_image[y_pixel, x_pixel], self.cameraInfo)
                    x_coords.append(x)
                    y_coords.append(y)
                    z_coords.append(z)
            # 计算坐标数组的平均值，得到人脸的中心坐标
            mean_x, mean_y, mean_z = np.mean(x_coords), np.mean(y_coords), np.mean(z_coords)
            # 计算人脸中心坐标到摄像头的距离
            mean_distance = np.sqrt(mean_x**2 + mean_y**2 + mean_z**2)
            # 计算人脸中心坐标与摄像头之间的夹角的余弦值
            cos_degree = math.degrees(mean_z / mean_distance)
            # 将角度转换为弧度，并加上头部俯仰角
            cos_radians = math.radians(cos_degree + self.config[HEAD_PITCH_KEY])
            # 计算人脸中心到摄像头的距离乘以余弦值
            cal_result = mean_distance * math.cos(cos_radians)
            # 如果计算结果比已知最小距离小，则更新最小距离
            if cal_result < min_distance:
                min_distance = cal_result
        # 打印最小距离，并返回其值
        rospy.loginfo('min_distance: {}'.format(min_distance))
        return min_distance

    def jy901_cb(self, msg):
        """
        回调函数，用于处理 jy901 数据。

        Args:
            msg: jy901 数据。

        """
        # 获取 jy901 数据中的末尾值，即躯干偏航角
        self.torso_yaw = msg.data[-1]
        # 标记躯干偏航角已更新
        self.torso_yaw_is_refreshed = True

    def set_aiui_server_status(self, is_pause):
        """
        暂停或恢复 AIUI 服务器和头部转向声音服务器。

        Args:
            is_pause: 是否暂停。

        """
        # 调用暂停 AIUI 服务器和头部转向声音服务器的客户端
        self.pause_aiui_server_client(is_pause)
        self.pause_head_toward_sound_client(is_pause)

    def calculate_diff_angle(self, diff_angle):
        """
        计算角度差。

        Args:
            diff_angle: 角度差。

        Returns:
            diff_angle: 角度差。

        """
        return diff_angle + 360 if diff_angle < -180 else diff_angle - 360 if diff_angle > 180 else diff_angle

    @property
    def is_jy901_refreshed(self):
        """
        判断躯干偏航角是否已更新。

        Returns:
            bool: 是否已更新。

        """
        # 标记躯干偏航角未更新
        self.torso_yaw_is_refreshed = False
        # 获取当前时间戳
        timestamp = rospy.Time.now()
        # 循环判断躯干偏航角是否已更新
        while not rospy.core.is_shutdown_requested() and self.torso_yaw_is_refreshed == False:
            if rospy.Time.now() - timestamp > rospy.Duration.from_sec(JY901_REFRESH_TIMEOUT):
                rospy.logerr('JY901 data timeout!')
                return False
            rospy.sleep(0.1)
        # 如果已更新，则返回 True；否则返回 False
        return True

    def turn_to_sound_source(self, sound_source_angle):
        """
        将机器人转向声源。

        Args:
            sound_source_angle: 声源相对机器人的角度。

        """
        if self.is_jy901_refreshed:
            # 计算目标躯干偏航角
            target_torso_yaw = self.torso_yaw + sound_source_angle
            # 循环调整机器人的姿态，直到转向完成或者获取 jy901 数据超时
            while not rospy.core.is_shutdown_requested() and self.is_jy901_refreshed:
                # 计算当前躯干偏航角与目标躯干偏航角的差值
                diff_yaw = target_torso_yaw - self.torso_yaw
                # 计算转向声源的角度差值
                cal_diff_yaw = self.calculate_diff_angle(diff_yaw)
                # 如果差值的绝对值小于等于阈值，则跳出循环
                if abs(cal_diff_yaw) <= CAL_DIFF_YAW_THRESHOLD:
                    break
                # 进入 walking 状态
                self.walk()
                # 原地旋转，使其转向声源
                self.walking_the_distance(0, 0, cal_diff_yaw)
                # 等待走完
                self.wait_walking_done()
        # 如果躯干偏航角未更新，则直接转向声源
        else:
            self.walk()
            self.walking_the_distance(0, 0, sound_source_angle)
            self.wait_walking_done()

    def head_control(self, angle):
        """
        控制机器人的头部转动。

        Args:
            angle: 头部转动角度。

        """
        # 定义一个动作帧，控制头部的俯仰角度
        act_frames = {
            22: [
                ((1000, angle), (0, 0), (0, 0))
            ]
        }
        # 进入 ready 状态
        self.ready()
        # 发送自定义的贝塞尔动作帧
        send_custom_bezier(act_frames)

if __name__ == '__main__':
    # 创建一个 SoundSourceLocalization 实例
    soundsourcelocalization = SoundSourceLocalization()
