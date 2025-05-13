#!/usr/bin/python
# coding=utf-8
import sys
import os
import cv2
import signal
if sys.version>'3':
    import queue as Queue
else:
    import Queue 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import array
import time
import threading
import subprocess
from ros_AIUI_node.srv import textToSpeakMultipleOptions
from bodyhub.srv import *  # for SrvState.srv
from bodyhub.msg import JointControlPoint
from lejulib import *
import rospkg
# 添加leju_lib_pkg包的路径到python查找目录中
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
sys.path.append("/home/lemon/robot_ros_application/catkin_ws/src/leju_lib_pkg/src/motion")
file_path = "/home/lemon/robot_ros_application/catkin_ws/src/ros_AIUI_node/scripts/head_toward_sound.py"
faceadd = rospkg.RosPack().get_path("ros_actions_node") + "/scripts/tracking/haarcascade_frontalface_alt2.xml"

class FaceConfig:   # 储存人脸识别参数的类
    def __init__(self):
        self.running = True
        self.size = 0.5 # 识别的缩放比例，缩小分辨率可以加快识别速度
        self.face = 0, 0, 0, 0  # 人脸坐标
        self.face_roi = 0, 0, 0, 0  # 感兴趣区域的坐标，用于在上一次查找的人脸附近进行跟踪
        self.face_template = None   # 人脸模板，用于人脸短暂丢失时的查找
        self.found_face = False #表示是否找到了人脸，初始值为False
        self.template_matching_running = False #表示模板匹配是否正在运行，初始值为False
        self.template_matching_start_time = 0 #记录模板匹配的开始时间，初始值为0
        self.template_matching_current_time = 0 #记录当前时间，初始值为0
        self.center_x = 160 #表示图像中心点的横坐标和纵坐标，分别初始值为160和120
        self.center_y = 120
        self.pan = 0 #表示水平和垂直方向上的偏转角度，初始值都为0
        self.tlt = 0
        self.error_pan = 0 #表示水平和垂直方向上的误差，初始值都为0
        self.error_tlt = 0
        self.controlID=2 #控制ID，用于控制机器人的动作，初始值为2
        self.HeadJointPub = rospy.Publisher('MediumSize/BodyHub/HeadPosition', JointControlPoint, queue_size=100) #ROS话题，用于发布机器人头部位置的控制命令
        self.update_ctlid() #用于更新控制ID
        self.bridge = CvBridge() #将ROS图像消息转换为OpenCV图像格式
        self.SERVO = client_action.SERVO #控制舵机动作   
        self.QUEUE_IMG = Queue.Queue(maxsize=2) # 用于传递图像数据的队列
        self.face_detector = cv2.CascadeClassifier(faceadd)# 加载识别人脸的级联分类器
        self.reset_head_position = False  # 是否需要重新设置头部角度为[0, 0]
        self.tts_param = {              # 定义待转文字及合成的参数
            'text': '我看不到你，可以站在我面前吗',
            'vcn': 'qige',
            'speed': 50,
            'pitch': 5,
            'volume': 20
        }

    def text_to_speek(self):
            rospy.wait_for_service("/aiui/text_to_speak_multiple_options", timeout=2)   # 等待服务可用。超时时间这里设置为 2s，默认会一直等待，超时会抛出 rospy.ROSException 异常
            tts_client = rospy.ServiceProxy("/aiui/text_to_speak_multiple_options", textToSpeakMultipleOptions)     # 创建 ros 服务客户端
            tts_client(self.tts_param['text'], self.tts_param['vcn'], self.tts_param['speed'], self.tts_param['pitch'], self.tts_param['volume'])       # 客户端发起请求，参数与该服务的类型定义一一对应


    def update_ctlid(self):#get the newest Main_controlID
        try:
            rospy.wait_for_service('MediumSize/BodyHub/GetMasterID',2)
        except:
            print ('error: wait_for_service GetMasterID!',sys.exc_info())
            return 0
        client = rospy.ServiceProxy('MediumSize/BodyHub/GetMasterID', SrvTLSstring)
        response = client('get')
        self.controlID= response.data
        return 1
    
def doubleRectSize(input_rect, keep_inside):
    """将input_recta输入矩形围绕其中心放大2倍，同时确保不超出keep_inside的范围"""
    xi, yi, wi, hi = input_rect
    xk, yk, wk, hk = keep_inside
    wo = wi * 2
    ho = hi * 2
    xo = xi - wi // 2
    yo = yi - hi // 2
    if wo > wk:
        wo = wk
    if ho > hk:
        ho = hk
    if xo < xk:
        xo = xk
    if yo < yk:
        yo = yk
    if xo + wo > wk:
        xo = wk - wo
    if yo + ho > hk:
        yo = hk - ho
    return xo, yo, wo, ho


def face_size(face):
    """计算人脸面积"""

    x, y, w, h = face
    return w * h


def face_filter(face_list):
    """查找返回最大的人脸所在坐标"""
    face_size_list = list(map(face_size, face_list))
    target_index = face_size_list.index(max(face_size_list))
    return face_list[target_index]


def detectFaceAllSizes(frame):
    """Detect using cascades over whole image

    :param frame:
    :return:
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)    
    # gray = cv2.equalizeHist(gray)
    face_locations = Face.face_detector.detectMultiScale(
        gray, scaleFactor=1.1, minNeighbors=3, minSize=(int(frame.shape[1] / 12), int(frame.shape[0] / 12)),
        maxSize=(int(2 * frame.shape[1] / 3), int(2 * frame.shape[1] / 3)))
    if len(face_locations) <= 0:
        Face.face = 0, 0, 0, 0
        return
    Face.found_face = True
    Face.face = face_filter(face_locations)
    Face.face_template = frame[Face.face[1]:(Face.face[1] + Face.face[3]),
                         Face.face[0]:(Face.face[0] + Face.face[2])].copy()
    Face.face_roi = doubleRectSize(Face.face, (0, 0, frame.shape[1], frame.shape[0]))


def detectFaceAroundRoi(frame):
    """Detect using cascades only in ROI
    在感兴趣区域附近检测人脸
    :param frame:
    :return:
    """
    face_tem = frame[Face.face_roi[1]:Face.face_roi[1] + Face.face_roi[3],
               Face.face_roi[0]:Face.face_roi[0] + Face.face_roi[2]]
    gray = cv2.cvtColor(face_tem, cv2.COLOR_BGR2GRAY)
    face_locations = Face.face_detector.detectMultiScale(
        gray, scaleFactor=1.1, minNeighbors=3, minSize=(int(frame.shape[1] / 12), int(frame.shape[0] / 12)),
        maxSize=(int(2 * frame.shape[1] / 3), int(2 * frame.shape[1] / 3)))
    if len(face_locations) <= 0:# 如果没有检测到人脸则准备使用模板匹配进行查找
        Face.template_matching_running = True
        if Face.template_matching_start_time == 0:
            Face.template_matching_start_time = cv2.getTickCount()# 记录开始模板匹配的时间
        return
    Face.template_matching_running = False
    Face.template_matching_current_time = 0
    Face.template_matching_start_time = 0

    Face.face = face_filter(face_locations)# 在ROI中查找最大人脸区域
    # 人脸在ROI中的坐标加上ROI的全局坐标才是人脸的全局坐标
    Face.face[0] += Face.face_roi[0]
    Face.face[1] += Face.face_roi[1]
    # 储存最新的人脸模板    
    Face.face_template = frame[Face.face[1]:Face.face[1] + Face.face[3],
                         Face.face[0]:Face.face[0] + Face.face[2]].copy()
    # 将新的人脸区域放大两倍作为感兴趣区域(ROI)
    Face.face_roi = doubleRectSize(Face.face, (0, 0, frame.shape[1], frame.shape[0]))


def detectFacesTemplateMatching(frame):
    """Detect using template matching

    :param frame:
    :return:
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    Face.template_matching_current_time = cv2.getTickCount()
    duration = (Face.template_matching_current_time - Face.template_matching_start_time) / cv2.getTickFrequency()
    print("duration",duration)
    if duration > 1:# 如果1s内没有检测到人脸(使用模板匹配1s后)，则退出内循环进入外循环查找其他人脸
        Face.found_face = False
        Face.template_matching_running = False
        Face.template_matching_start_time = 0
        Face.template_matching_current_time = 0
    # 使用模板图片在感兴趣区域进行模板匹配
    target = gray[Face.face_roi[1]:Face.face_roi[1] + Face.face_roi[3],
             Face.face_roi[0]:Face.face_roi[0] + Face.face_roi[2]]
    Face.face_template = cv2.cvtColor(Face.face_template, cv2.COLOR_BGR2GRAY)
    res = cv2.matchTemplate(target, Face.face_template, cv2.TM_CCOEFF)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    max_x = max_loc[0] + Face.face_roi[0]
    max_y = max_loc[1] + Face.face_roi[1]
    # 使用模板匹配的结果作为人脸检测的结果
    Face.face = max_x, max_y, Face.face[2], Face.face[3]
    Face.face_template = frame[Face.face[1]:Face.face[1] + Face.face[3],
                         Face.face[0]:Face.face[0] + Face.face[2]].copy()
    # 将人脸区域围绕中心放大两倍作为感兴趣区域
    Face.face_roi = doubleRectSize(Face.face, (0, 0, frame.shape[1], frame.shape[0]))


def show_face(face):#显示人脸位置
    # 根据人脸区域的尺寸和位置信息，在客户端标签上标记出人脸的中心点，并根据需要设置标签的颜色。
    face_cx = (face[0] + face[2] / 2) / Face.size
    face_cy = (face[1] + face[3] / 2) / Face.size
    client_label.set_camera_label((255, 0, 0), (face_cx, face_cy), face[2]/Face.size, face[3]/Face.size)


def detectFace():
    rate = rospy.Rate(100) #设置循环的频率为 100Hz
    while not Face.found_face and Face.running:
        time.sleep(0.01)
        if not Face.QUEUE_IMG.empty():
            frame = Face.QUEUE_IMG.get() #如果不为空，则从队列中获取一帧图像，并赋值给 frame
        else:
            continue
        detectFaceAllSizes(frame) #进行人脸检测
        show_face(Face.face) #检测到人脸

        while Face.found_face and Face.running:
            rate.sleep()
            if not Face.face_template.any(): #检查是否有人脸模板
                continue
            if not Face.QUEUE_IMG.empty(): #检查输入图像队列是否为空
                frame = Face.QUEUE_IMG.get()
            else:
                continue
            detectFaceAroundRoi(frame)
            if Face.template_matching_running: #检查是否启用了人脸模板匹配功能
                detectFacesTemplateMatching(frame)
            show_face(Face.face)


def async_do_job(func): #守护线程，异步执行三个线程
    async_thread = threading.Thread(target=func)
    async_thread.setDaemon(True) #当主线程结束时，该线程也会被终止
    async_thread.start() #启动线程，开始执行start函数


def set_head_servo(angles):
    """set head servos angle

    :param angles:[pan, tilt]
    :return:
    """
    # angles = array.array("d", angles)
    if not rospy.core.is_shutdown_requested():
        print("pub head rot",angles)
        Face.HeadJointPub.publish(positions=angles, mainControlID=Face.controlID)
    time.sleep(0.01)


def terminate(data):
    """Terminate all threads
    终止所有线程
    """
    rospy.loginfo(data.data)
    Face.running = False


def thread_face_center():
    """获取人脸中心位置"""
    while Face.running:
        time.sleep(0.01)
        face_x = Face.face[0] + Face.face[2] / 2
        face_y = Face.face[1] + Face.face[3] / 2
        if face_x == 0 and face_y == 0:
            face_x = Face.center_x
            face_y = Face.center_y
        Face.error_pan = Face.center_x - face_x
        Face.error_tlt = Face.center_y - face_y
        rospy.logdebug("Face.error_pan,Face.error_tlt %f,%f", Face.error_pan, Face.error_tlt)

def reset_servos():
    """
    将舵机回零
    """
    set_head_servo([0.0, 0.0])  # 设置头部角度为[0, 0]
    rospy.logerr("没有识别到人脸")
    print("回零")
    # 等待一段时间，确保舵机运动到零位
    Face.text_to_speek()
    time.sleep(3)
    
def thread_set_servos():
    """操作舵机控制头部运动"""
    set_head_servo([Face.pan, Face.tlt])
    step = 0.01
    Face.update_ctlid()
    no_face_count = 0  # 没有识别到人脸的计数器
    MAX_NO_FACE_COUNT = 100  # 最大连续没有识别到人脸的次数
    last_speak_time = 0  # 上次播报语音的时间戳
    speak_interval = 5  # 播报语音的时间间隔
    while Face.running:
        if abs(Face.error_pan) > 15 or abs(Face.error_tlt) > 15:
            if abs(Face.error_pan) > 15:
                Face.pan += step * Face.error_pan
            if abs(Face.error_tlt) > 15:
                Face.tlt += step * Face.error_tlt
            if Face.pan > 90.0:
                Face.pan = 90.0
            if Face.pan < -90.0:
                Face.pan = -90.0
            if Face.tlt > 25.0:
                Face.tlt = 25.0
            if Face.tlt < -25.0:
                Face.tlt = -25.0
            set_head_servo([Face.pan, -Face.tlt])
            Face.reset_head_position = False  # 重置为False，保持人脸追踪连续性
        else:
            if Face.reset_head_position or not Face.found_face:  # 如果需要回零或没有识别到人脸
                if time.time() - last_speak_time >= speak_interval:  # 判断是否达到播报时间间隔
                    Face.text_to_speek()
                    last_speak_time = time.time()  # 更新上次播报语音的时间戳
                time.sleep(0.1)
                no_face_count += 1
                print(no_face_count)
                Face.reset_head_position = False
                if not Face.found_face:  # 没有识别到人脸
                    if no_face_count >= MAX_NO_FACE_COUNT:  # 连续十秒没有识别到人脸，回零
                        reset_servos()  # 将舵机回零
                        Face.pan = 0.0 # 将两个舵机角度设置为初始值
                        Face.tlt = 0.0
                        Face.reset_head_position = True  # 设置为 True，从零点开始追踪
                        no_face_count = 0  # 重置计数器
                else:  # 重新识别到人脸
                    no_face_count = 0



def image_callback(msg): #处理接收的图像信息
    try:
        cv2_img = Face.bridge.imgmsg_to_cv2(msg, "bgr8") #将ROS图像消息转换为OpenCV格式的图像
    except CvBridgeError as err:
        print(err)
    else:
        cv2_img = cv2.resize(cv2_img, (0, 0), fx=Face.size, fy=Face.size)
        if Face.QUEUE_IMG.full():
            Face.QUEUE_IMG.get() #如果队列已满则从队列中取出一个元素，释放空间
        Face.QUEUE_IMG.put(cv2_img, block=True)

def face_tracking_stop(): #停止人脸追踪
    Face.running = False
    client_controller.send_label_on(False) #关闭标签显示功能
    client_controller.send_video_status(False, "/camera/label/image_raw",width=640,height=480) #关闭视频流
    subprocess.Popen(["python", file_path])  # 在新的进程中打开Python文件
    
def main():
    node_initial(name = "face_tracking") #初始化节点
    rospy.on_shutdown(face_tracking_stop)
    os.system("rosnode kill /head_toward_sound") #启动脚本时杀死声源定位节点
    rospy.sleep(0.2)
    client_controller.send_label_on(True) #启动标签显示功能
    client_controller.send_video_status(True, "/camera/label/image_raw",width=640,height=480) #启动视频流
    image_topic = "/camera/color/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback) #当有新图像传入时，会调用image_callback进行处理
    rospy.Subscriber('terminate_current_process', String, terminate) #当接到消息时，会调用 terminate 函数进行处理，用于终止当前进程
    async_do_job(detectFace)
    async_do_job(thread_face_center)
    async_do_job(thread_set_servos)
    rospy.spin()

Face = FaceConfig()
if __name__ == '__main__':
    main()
    rospy.init_node("text_to_speak")