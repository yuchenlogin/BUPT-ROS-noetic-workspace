#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import sys
import json
import threading
import socket
import time
import array
import logging
import yaml
if sys.version>'3':
    import queue as Queue
else:
    import Queue 
import serial
from datetime import datetime
import subprocess
import re
sys.path.append('/home/lemon/robot_ros_application/catkin_ws/src/keyboards/scripts')
from play_tts_file import Player
from process_roban_config import get_robot_name, set_robot_name

from nodes import Node
from motion.motionControl import *
from ZeroServoControl import ZeroServoTransfer
from lejufunc.bezier import get_custom_points, get_bezier_frames
from lejufunc.ServoJointTrajectory import ServoJointTrajectoryC

from bodyhub.srv import *  
from sensor_msgs.msg import Joy
from ros_socket_node.srv import ModifyBtnsConfig
from ros_actions_node.msg import DeviceList
from bodyhub.msg import JointControlPoint
from std_msgs.msg import String, Float64MultiArray, UInt16
from ros_AIUI_node.srv import setDemoRunning
import hashlib

logging.basicConfig(level=logging.DEBUG, format=' %(asctime)s - %(levelname)s- %(message)s')


HOST = '0.0.0.0'
PORT = 12000
os.system('kill -9 $(fuser {}/tcp 2>/dev/null)'.format(PORT))

ROS_NODE_NAME = 'SocketNode'
rospy.loginfo("Starting {}".format(ROS_NODE_NAME))
rospy.init_node(ROS_NODE_NAME, anonymous=True, log_level=rospy.INFO)
rospy.sleep(2.0)  # The necessary delay for init

# TODO: immature code inside
SERVO = ServoJointTrajectoryC()
NODE = Node()
lock = threading.Lock()
run_lock = threading.Lock()
exec_lock = threading.Lock()

actQueue = Queue.Queue()
sck_recv_data_queue = Queue.Queue()
current_sock = None
isStopLast = False
EXE_RUN_TIME_OUT = 5
LEG_SERVO_LIMIT_VALUE_MIN = 260
LEG_SERVO_LIMIT_VALUE_MAX = 3940
ARM_SERVO_LIMIT_VALUE_MIN = 260
ARM_SERVO_LIMIT_VALUE_MAX = 3840
MEDIMOTOALPHA = 12.80
SMALMOTOALPHA = 18.61

servo_params = lambda leg, arm: [leg, leg, leg, leg, leg, leg, leg, leg, leg, leg, leg, leg, leg, arm, arm, leg, arm, arm, arm, arm, arm, arm]
SERVO_VALUES_MAX = servo_params(LEG_SERVO_LIMIT_VALUE_MAX, ARM_SERVO_LIMIT_VALUE_MAX)
SERVO_VALUES_MIN = servo_params(LEG_SERVO_LIMIT_VALUE_MIN, ARM_SERVO_LIMIT_VALUE_MIN)
ALPHA = servo_params(MEDIMOTOALPHA, SMALMOTOALPHA)


UNCONNECTED = False
CONNECTED = True
Executing=False

timestamp = 1

DOWNLOAD_EXEC_FILE_PATH = "/home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts"
DOWNLOAD_MUSIC_FILE_PATH = "/home/lemon/Music/actmusic"
DOWNLOAD_PYTHON_FILE_FLAG = ".py"
DOWNLOAD_MUSIC_FILE_FLAG = ".wav"

def executing_end(data=None):
    global Executing
    # rospy.logwarn('Executing end')
    Executing=False

def _list_to_array(act_list):
    return array.array("d", act_list)


def _dict_to_bytes(dict_msg):
    if sys.version>'3':
        return bytes(json.dumps(dict_msg).encode())
    else:
        return bytes(json.dumps(dict_msg))


def load_config(configfile):
    """reading config file

    :return: offset list or InitPose list
    """
    filePath = rospy.get_param(configfile)
    with open(filePath, "r") as file:
        file_body = yaml.load(file.read())
    if configfile == "poseOffsetPath":
        filecon = file_body["offset"]
    elif configfile == "poseInitPath":
        filecon = file_body["InitPose"]

    values = []
    for i in range(len(filecon)):
        keyvalue = "ID" + str(i + 1)
        values.append(filecon[keyvalue])
    return values


def async_do_job(func, args=None):
    """unblock do func task

    :param func:
    :param args:
    :return:
    """
    threading.Thread(target=func, args=args).start()


# TODO: NOT WORKING
def wait_move_done():
    """waiting for servo move done

    :return:
    """
    while not rospy.is_shutdown():
        try:
            rospy.wait_for_message('MediumSize/BodyHub/Status', UInt16, 0.1)
            break
        except Exception as err:
            print(err)
            break


def act_execution_state():
    """enter action execution mode

    :return:
    """
    lock.acquire()
    NODE.masterID_guarantee()
    current_state = NODE.get_current_status()
    if current_state == "error":
        rospy.logerr("bodyhub state error")
        raise
    if current_state == "directOperate":
        NODE.state_jump("reset")
        NODE.state_jump("setStatus")
    elif current_state == "preReady":
        NODE.state_jump("setStatus")
    elif current_state == "walking":
        NODE.state_jump("stop")
        NODE.wait_for_ready()
    lock.release()


def act_directoperation_state():
    """enter direct method mode

    :return:
    """
    lock.acquire()
    NODE.masterID_guarantee()
    current_state = NODE.get_current_status()
    if current_state == "error":
        rospy.logerr("bodyhub state error")
        raise
    if current_state != "directOperate":
        if current_state in ["running", "pause", "walking"]:
            NODE.state_jump("stop")
        elif current_state == "preReady":
            NODE.state_jump("setStatus")
        NODE.wait_for_ready()
        NODE.state_jump("derectOperate")
    lock.release()


def walking_state():
    """enter walking mode

    :return:
    """
    NODE.masterID_guarantee()
    current_state = NODE.get_current_status()
    if current_state == "error":
        rospy.logerr("bodyhub state error")
        raise
    if current_state != "walking":
        if current_state == "preReady":
            NODE.state_jump("setStatus")
        elif current_state in ["running", "pause"]:
            NODE.state_jump("stop")
        elif current_state == "directOperate":
            NODE.state_jump("reset")
            NODE.state_jump("setStatus")
        NODE.wait_for_ready()
        NODE.state_jump("walking")



def act_execution(path, logout =False):
    print(path)
    NODE.node_run(path, logout = logout)


def reset_walking_state():
    lock.acquire()
    NODE.walk_node_stop()
    while GetBodyhubStatus().data == "walking":
        time.sleep(0.01)
    NODE.walk_node_reset()
    NODE.walkflag = False
    lock.release()


def set_head_servo(angles, time, sck_conn):
    """set servos [21, 22] angles

    :param angles:[ang1,ang2]
    :return:
    """

    try:
        act_execution_state()
        SERVO.HeadJointTransfer(angles, time)
        SERVO.MotoWait()

        msg = {'cmd': 'set_head_servo',
               'state': 'ok'}
    except rospy.ServiceException as e:
        msg = {'cmd': 'set_head_servo',
               'state': 'error'}
    finally:
        sck_conn.send(_dict_to_bytes(msg))

is_stop = False
def set_all_servo(ids, angles, act_time, sck_conn, sendback=None):
    """set servos [1, 22] angles

    :param ids:
    :param angles:
    :return:
    """
    global is_stop
    msg = {
        'cmd': 'set_all_servo',
        'state': 'error'
    }
    is_first = True
    is_stop = False
    run_lock.acquire()
    try:
        rospy.wait_for_service('/MediumSize/BodyHub/GetJointAngle', 2)
        servo_client = rospy.ServiceProxy('/MediumSize/BodyHub/GetJointAngle', SrvServoAllRead)
        id_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
        act_execution_state()
        result = servo_client(id_list, len(id_list))
        p0 = result.getData 
        servo_angle_limit(angles)
        move_frames = get_bezier_frames(p0, angles, None, act_time)[1:]
        if len(move_frames) != 0:
            for frame in move_frames:
                if is_stop:
                    return
                SendJointCommand(2, id_list, frame)
                time.sleep(0 if is_first else 0.009)
                is_first = False
            WaitTrajectoryExecute(True)
        msg = {
            'cmd': 'set_all_servo',
            'state': 'ok'
        }
    finally:
        run_lock.release()
        if sendback == "TRUE":
            sck_conn.send(_dict_to_bytes(msg))


def set_multiple_all_servo(sck_conn, act_frames, start_index, stop_index):
    global is_stop 
    act_execution_state()
    last_point = None
    is_first = True
    is_stop = False
    msg = {
        'cmd': 'set_multiple_all_servo',
        'state': 'ok'
    }
    sck_conn.send(_dict_to_bytes(msg))
    act_frames = { int(key): act_frames[key] for key in act_frames }
    joint_ids = list(act_frames.keys())
    joint_ids.sort()
    run_lock.acquire()
    num = 0  
    for point in get_custom_points(act_frames):
        if is_stop:
            break       
        if num >= start_index and num <= stop_index:  
            SendJointCommand(2, joint_ids, point)
            time.sleep(0 if is_first else 0.009) 
            is_first = False 
        num = num + 1
    WaitTrajectoryExecute(True)
    run_lock.release()
    

def get_all_servo(id_list, sck_conn):
    """get servos [1, 22] angles

    :param id_list:
    :param sck_conn:
    :return:
    """

    try:
        rospy.wait_for_service('/MediumSize/BodyHub/GetJointAngle', 2)
        servo_client = rospy.ServiceProxy('/MediumSize/BodyHub/GetJointAngle', SrvServoAllRead)
        result = servo_client(id_list, len(id_list))
        resp_value = result.getData

        msg = {
            'cmd': 'get_position',
            'state': 'ok',
            'pos': resp_value
        }
    except rospy.ServiceException as e:
        msg = {
            'cmd': 'get_position',
            'state': 'error'
        }
    else:
        sck_conn.send(_dict_to_bytes(msg))


def view_action(act_frames, sck_conn):
    """ View action

    :param act_frames:
    :param sck_conn:
    :return:
    """
    try:
        lock.acquire()
        actQueue.put(act_frames)
        lock.release()
        msg = {'cmd': 'view_action',
               'state': 'ok'}
    except Exception as e:
        msg = {'cmd': 'view_action',
               'state': 'error'}
    finally:
        sck_conn.send(_dict_to_bytes(msg))


def stop_action(sck_conn, id):
    """stop action view

    :param sck_conn:
    :return:
    """
    
    global is_stop
    #if is_stop:
    is_stop = True
    msg = {
        'cmd': 'stop_action',
        'state': 'error',
        'id': id
    }
    try:
        rospy.wait_for_service('/MediumSize/BodyHub/GetJointAngle', 2)
        servo_client = rospy.ServiceProxy('/MediumSize/BodyHub/GetJointAngle', SrvServoAllRead)
        id_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
        result = servo_client(id_list, len(id_list))
        resp_value = result.getData

        msg = {
            'cmd': 'stop_action',
            'state': 'ok',
            'last_angle': resp_value,
            'id': id
        }
    except rospy.ServiceException as e:
        pass
    finally:
        sck_conn.send(_dict_to_bytes(msg))


def read_imuDate(sck_conn):
    '''use during robot gait,otherwise it will time out'''
    imuDateSub = rospy.wait_for_message('/imu/Torso', Float64MultiArray, 2)
    data = imuDateSub.data
    gyro = data[:3]
    acc = data[3:6]

    msg = {
        'cmd'  : 'read_imuDate',
        'gyro' : gyro, 
        'acc'  : acc
    }
    
    sck_conn.send(_dict_to_bytes(msg))


def set_imuState(sck_conn, imu_state):
    rospy.wait_for_service("imuState")
    try:
        imuState_client = rospy.ServiceProxy("imuState",SrvimuState)
        resp = imuState_client.call(imu_state)
        if 'on' in resp.freeback:
            state = "on"
        elif 'off' in resp.freeback:
            state = 'off'
        else:
            rospy.loginfo("%s" %resp.freeback)
    except rospy.ServiceException as err:
        rospy.logwarn("Service call failed: %s" %err)

    msg = {
        'cmd'  : 'set_imuState',
        'imu_state' : state
    }
    sck_conn.send(_dict_to_bytes(msg))


def actmoto():
    """ moto queue to act

    :return:
    """

    while not rospy.is_shutdown():
        if actQueue.empty():
            time.sleep(0.01)
        else:
            lock.acquire()
            frame = actQueue.get()
            lock.release()
            servo_value = frame[0]
            acttime = frame[1]
            interval = frame[2] / 1000.0
            set_all_servo(None, servo_value, acttime, None)
            time.sleep(interval)
def get_execution_status(sck_conn):
    """get execution status
    :param sck_conn:    
    """
    msg = {
        "cmd": "get_execution_status",
        "in_execution": NODE.is_executing
    }
    sck_conn.send(_dict_to_bytes(msg))

def get_act_nodes(sck_conn):
    """list ros node demo file under dir

    :param sck_conn:
    :return:
    """
    app_list = NODE.list_app()
    msg = {
        "cmd": "ls_node",
        "nodes": app_list
    }
    sck_conn.send(_dict_to_bytes(msg))


def get_login_account(sck_conn):
    """return ssh username and password

    :param sck_conn:
    :return:
    """
    msg = {
        "cmd": "get_login_account",
        "user": "lemon",
        "password": "softdev"
    }
    sck_conn.send(_dict_to_bytes(msg))

def exec_download_file(path):
    """executr download file

    :param path:
    :return:
    """

    if NODE.is_executing:
        rospy.logwarn("instance in executing...")
        return

    if (NODE.terminate_pub.get_num_connections() != 0):
        reset_body_state()
    exec_lock.acquire()
    # act_execution_state()
    act_execution(path, logout = False)
    if not wait_terminate_sub(EXE_RUN_TIME_OUT): #wait node inital
        rospy.logwarn("Exception in script startup!")
    exec_lock.release()


def wait_terminate_sub(time_out):
    for _ in range(time_out):
        if (NODE.terminate_pub.get_num_connections() != 0):
            return True
        time.sleep(0.5)
    return False


def reset_bodyhub(sck_conn):
    
    ResetBodyhub()
    msg = {
        "cmd": "reset_bodyhub",
        "state": "ok"
    }
    sck_conn.send(_dict_to_bytes(msg))


def reset_body_state():
    """reset bodyhub and actExec node state
       robot standby.
    :return:
    """
    while exec_lock.locked():
        time.sleep(0.1)
    
    exec_lock.acquire()
    if (NODE.terminate_pub.get_num_connections() != 0):
        terminal_str = "Terminate current python process"
        NODE.terminate_pub.publish(terminal_str)
        
        # waiting for some time while there are subscriptions
        WAITNG_STOP_TIMEOUT = 5
        s_time = time.time()
        while not rospy.is_shutdown() and NODE.is_executing and time.time() - s_time < WAITNG_STOP_TIMEOUT:
            print("waiting for demo to finish...")
            time.sleep(0.2)
    NODE.node_force_stop()
    # rospy.logwarn(NODE.runner.read_log())
    while GetBodyhubStatus().data != "preReady":
        rospy.loginfo("BodyhubStatus not int preReady, reset_state!")
        try:
            ResetBodyhub()
        except Exception as e:
            rospy.loginfo("ResetBodyhub failed {} ,retrying...".format(e))
        time.sleep(2)

    else:
        rospy.loginfo("reset_body_state done")
    exec_lock.release() 


def query_node_state(sck_conn):
    """check demo node is running

    :return:
    """
    state = NODE.get_current_status()
    msg = {
        "cmd": "query_node_state",
        "state": state
    }
    sck_conn.send(_dict_to_bytes(msg))

con_is_reset = False
def handle_controller_cmd(sck_conn, key_value):
    """handle controller command

    :param key_value: [x1,y1,x2,y2]
    :return:
    """
    global con_is_reset
    try:
        con_is_reset = False
        while lock.locked():
            if con_is_reset:
                time.sleep(0.01)
                break
        lock.acquire()
        if not NODE.walkflag:
            walking_state()
            NODE.walkflag = True
        lock.release()
        axes = [0, 0, 0, 0, 0, 0, 0, 0]  # 0-7
        buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # 0-10
        axes[0] = -1 * key_value[0]
        axes[1] = key_value[1]
        axes[3] = -1 * key_value[2]
        axes[4] = key_value[3]
        joyPub = rospy.Publisher('/joy', Joy, queue_size=2)
        joyPub.publish(axes=axes, buttons=buttons)

        msg = {
            "cmd": "receive_controller_cmd",
            "state": 'ok'
        }
    except rospy.ServiceException as e:
        msg = {
            'cmd': 'receive_controller_cmd',
            'state': 'error'
        }
    except Exception as e:
        print(e)
        msg = {
            'cmd': 'receive_controller_cmd',
            'state': 'error'
        }
    finally:
        sck_conn.send(_dict_to_bytes(msg))


def handle_controller_reset(sck_conn):
    """handle controller reset

    :param :
    :return:
    """
    global con_is_reset
    con_is_reset = True
    try:
        reset_walking_state()
        msg = {
            "cmd": "receive_controller_reset",
            "state": 'ok'
        }
    except rospy.ServiceException as e:
        msg = {
            'cmd': 'receive_controller_reset',
            'state': 'error'
        }
    except Exception as e:
        print(e)
        msg = {
            'cmd': 'receive_controller_cmd',
            'state': 'error'
        }
    finally:
        sck_conn.send(_dict_to_bytes(msg))


def get_zero_value(sck_conn):
    """get servos [1, 22] zero position offset

    :param sck_conn:
    :return:
    """
    zero_value = load_config("poseOffsetPath")
    msg = {
        "cmd": "get_zero_value",
        "zero_value": zero_value
    }
    sck_conn.send(_dict_to_bytes(msg))

def servo_value_limit(values=[]):
    for i in range(len(values)):
        if (values[i] > SERVO_VALUES_MAX[i]): values[i] = SERVO_VALUES_MAX[i]
        elif (values[i] < SERVO_VALUES_MIN[i]): values[i] = SERVO_VALUES_MIN[i]

def servo_angle_limit(angles=[]):
    for index, angle in enumerate(angles): 
        servo_limit_angle_max = (SERVO_VALUES_MAX[index] - 2048) / ALPHA[index]
        servo_limit_angle_min = (SERVO_VALUES_MIN[index] - 2048) / ALPHA[index]
        if (angle > servo_limit_angle_max): angles[index] = servo_limit_angle_max
        elif (angle < servo_limit_angle_min): angles[index] = servo_limit_angle_min

def set_zero_servo(sck_conn, sendback=None, slowspeed=None, ids=None, zerovalue=None):
    """ set servos [1, 22] zero position

    :param sck_conn:
    :param ids: None or [1-22]
    :param zerovalue: None or [value1-value22]
    :return:
    """
    try:
        act_directoperation_state()
        if ids == None or zerovalue == None:
            cmd = 'view_zero_position'
            ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
            angles = load_config("poseOffsetPath")
            slowspeed = "TRUE"
        else:
            cmd = 'set_zero_servo'
            ids = ids
            zero_servo_value = zerovalue
        zero_servo_value = [i + 2048.0 for i in zero_servo_value]
        servo_value_limit(zero_servo_value)
        if slowspeed == "TRUE":
            ZeroServoTransfer(zero_servo_value)
        else:
            rospy.wait_for_service("/MediumSize/BodyHub/DirectMethod/SetServoTarPositionValAll", 2)
            servomoto = rospy.ServiceProxy("/MediumSize/BodyHub/DirectMethod/SetServoTarPositionValAll",
                                           SrvServoAllWrite)
            servomoto(ids, len(zero_servo_value), zero_servo_value)

        msg = {'cmd': cmd,
               'state': 'ok'}
    except rospy.ServiceException as err:
        msg = {'cmd': cmd,
               'state': 'error'}
    finally:
        if sendback == "TRUE":
            sck_conn.send(_dict_to_bytes(msg))


def virtual_control_panel(sck_conn, key):
    """

    :param sck_conn:
    :param key: 'A' 'B' 'X' 'Y' 'LEFT' 'RIGHT' 'UP' 'DOWN' 'STOP'
    :return:
    """
    try:
        NODE.buttonPub.publish(data=key)
        msg = {'cmd': 'virtual_control_panel',
               'state': 'ok'}
    except rospy.ROSException as err:
        msg = {'cmd': 'virtual_control_panel',
               'state': 'error'}

    finally:
        sck_conn.send(_dict_to_bytes(msg))

def get_init_value(sck_conn):
    """get servos [1, 22] init position value

    :param sck_conn:
    :return:
    """
    init_value = load_config("poseInitPath")
    msg = {
        "cmd": "get_init_value",
        "init_value": init_value
    }
    sck_conn.send(_dict_to_bytes(msg))

def get_cameras_status(sck_conn):
    """get realsense D435i and usb cam status

    :param sck_conn:
    :return:
    """
    ISEXIST = 0
    result = {"realsense_D435i": UNCONNECTED, "usb_cam": UNCONNECTED}

    if os.system("lsusb | grep '8086:0b07'") == ISEXIST:
        result['realsense_D435i'] = CONNECTED
    if os.system("lsusb | grep '1e2f:9785'") == ISEXIST:
        result['usb_cam'] = CONNECTED

    msg = {
        "cmd": "get_cameras_status",
        "realsense_d435i_status": result['realsense_D435i'],
        "usb_cam_status": result['usb_cam']
    }
    sck_conn.send(_dict_to_bytes(msg))

def get_soundcard_status(sck_conn):
    """get soundcard status

    :param sck_conn:
    :return:
    """
    ISEXIST = 0
    XFM10X2_EXIST = '2\n'
    soundcard_status = UNCONNECTED

    if os.system("arecord -l | grep -qE \"(Bothlent)|(AC108) \"") == ISEXIST:
        soundcard_status = CONNECTED
    elif os.popen("lsusb | grep '0403:6001' -c").readlines()[0] == XFM10X2_EXIST:
        soundcard_status = CONNECTED

    msg = {
        "cmd": "get_soundcard_status",
        "soundcard_status": soundcard_status
    }
    sck_conn.send(_dict_to_bytes(msg))

def get_servo_status(sck_conn):
    """get servo status

    :param sck_conn:
    :return:
    """
    srv_name = '/MediumSize/BodyHub/ScanServo'
    correct_id_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
    scannable_servo_list = []
    miss_servo_list = []

    try:
        rospy.wait_for_service(srv_name, 2)
        servo_client = rospy.ServiceProxy(srv_name, SrvServoScan)
        result = servo_client('torso')
        servo_id_list = list(result.getData)
        for servo_id in correct_id_list:
            for scan_id in servo_id_list:
                if scan_id == servo_id:
                    scannable_servo_list.append(scan_id)
                    servo_id_list.remove(scan_id)
                    break
                else:
                    miss_servo_list.append(servo_id)
                    break

        msg = {
            "cmd": "get_servo_status",
            "scannable_servo": scannable_servo_list,
            "miss_servo": miss_servo_list
        }
    except rospy.ServiceException as e:
        msg = {
            'cmd': 'get_servo_status',
            'state': 'error'
        }
    finally:
        sck_conn.send(_dict_to_bytes(msg))

def get_robot_back_btns_feature(sck_conn):
    """get_robot_back_btns_feature

    :param sck_conn:
    :return:
    """
    BACK_BTNS_FEATURE_CONFIG_FILE_PATH = '/home/lemon/.lejuconfig/BackBtnsConfig.json'
    ISEXIST = True
    robotBackBtns = map(lambda x: {"id": x, "path": ""}, [1, 2, 3, 4, 5])

    if os.path.exists(BACK_BTNS_FEATURE_CONFIG_FILE_PATH) != ISEXIST:
        config_json = json.dumps(robotBackBtns)
        with open(BACK_BTNS_FEATURE_CONFIG_FILE_PATH, 'w+') as f:
            f.write(config_json)
    else:
        with open(BACK_BTNS_FEATURE_CONFIG_FILE_PATH, mode='r') as f:
            robotBackBtns = json.load(f)
    msg = {
        "cmd": "get_robot_back_btns_feature",
        "robotBackBtns": robotBackBtns
    }
    sck_conn.send(_dict_to_bytes(msg))

def set_robot_back_btns_feature(sck_conn, config_list):
    """set_robot_back_btns_feature

    :param sck_conn:
    :param config_list:
    :return:
    """
    WRITTING_CONFIG = 'writting'
    FINISHED_CONFIG = 'done'
    BACK_BTNS_FEATURE_CONFIG_FILE_PATH = '/home/lemon/.lejuconfig/BackBtnsConfig.json'
    write_state = False
    config_json = json.dumps(config_list)
    modify_btnsconfig_client = rospy.ServiceProxy('/ros_socket_node/modify_backbtnsconfig', ModifyBtnsConfig)
    modify_btnsconfig_client(WRITTING_CONFIG)
    with open(BACK_BTNS_FEATURE_CONFIG_FILE_PATH, mode='w+') as f:
        f.write(config_json)
    modify_btnsconfig_client(FINISHED_CONFIG)
    write_state = True
    msg = {
        "cmd": "set_robot_back_btns_feature",
        "state": write_state
    }
    sck_conn.send(_dict_to_bytes(msg))

def judge_ssid(target_ssid):
    current_ssid = subprocess.check_output(["iwgetid", "-r"]).strip()
    if current_ssid != target_ssid:
        return False
    return True

def get_wifi_ip():
    ipv4 = subprocess.check_output("/sbin/ifconfig wlp0s20f3 | grep \"inet addr\" | awk -F: \'{print $2}\' | awk \'{print $1}\'", shell=True)
    ipv4 = re.match(r'\d+.\d+.\d+.\d+', ipv4).group()
    return ipv4

def del_wifi(sck_conn,wifi_name):
    """del_wifi
    del_wifi that has connected
    """
    returncode=0
    try:
        returncode= subprocess.call("cd /etc/NetworkManager/system-connections;sudo find . -type f -name '{0}\ [0-9]' -delete;sudo find . -type f -name '{0}' -delete;".format(wifi_name), shell=True)
    except :
        returncode=1

    msg = {
        "cmd": "del_wifi",
        "result":returncode,
    }
    sck_conn.send(_dict_to_bytes(msg))

def get_connected_wifi_list(sck_conn):
    """get_wifi_list
    Get connected WiFi list and information
    
    socket msg: 
        "cmd": "get_wifi_list",
        "connected_wifi_info":connected wifi info list,
        "active_wifi_info": active wifi info list,
        "current_wifi":current wifi name
        
    wifi info:
        name(str): wifi name
        intensity(int): intensity of wifi,max to 100
        security(str): WPA1/WPA2/--, -- for no security
        is_saved(bool): have connected before?
        
    """
    connected_wifi_info={}
    active_wifi_info={}
    current_wifi="None"
    #get active wifi list

    scan_current_area_wifi_command = "nmcli -f SSID,BSSID,SSID-HEX,SIGNAL,SECURITY,ACTIVE device wifi"
    command_result = subprocess.check_output(scan_current_area_wifi_command, shell=True)
    partern = r'\n((?:\S.*\S)|(?: )) +(\w\w(?::\w\w){5})\s*[A-F0-9]+ +(\d+)\s*(\-{2}|\w+.?[\w|\.]+)\s*(\w+)'
    active_wifi_list = re.findall(partern, command_result)
        
    #get wifi that has connected
    connected_wifi_cmd = subprocess.check_output("nmcli -f NAME,TYPE con show", shell=True)
    connected_wifi_names = re.findall(r"\n((?:\S.*\S)|(?: )) +[\w-]+wireless", connected_wifi_cmd)
    active_wifi_list.sort(key=lambda x: int(x[2]),reverse=True)
    for wifi in active_wifi_list:
        if wifi[-1]=="yes":
            current_wifi=wifi[0]
        if wifi[0] not in active_wifi_info.keys():
            info={"intensity":int(wifi[2]),"security":wifi[3]if len(wifi[3]) else "--","is_saved":True if wifi[0] in connected_wifi_names else False}
            active_wifi_info[wifi[0]]=info
            
    for w_name in connected_wifi_names:
        if w_name not in active_wifi_info.keys():
            connected_wifi_info[w_name]={"intensity":-1,"security":"--","is_saved":True}
        else:
            connected_wifi_info[w_name]=active_wifi_info[w_name]
            
    msg = {
        "cmd": "get_wifi_list",
        "connected_wifi_info":connected_wifi_info,
        "active_wifi_info": active_wifi_info,
        "current_wifi":current_wifi
    }
    sck_conn.send(_dict_to_bytes(msg))
    
def set_wifi(sck_conn, wifi_info={}):
    """set_wifi

    :param sck_conn:
    :wifi_connect_info:
    :return:
    """
    NO_PASSWORD = ''
    VOICE_FOLDER_PATH = '/home/lemon/robot_ros_application/catkin_ws/src/keyboards/voice/'
    PLAY_SET_WIFI_RESULT = 0
    PRINT_SET_WIFI_RESULT = 1
    player = Player()
    returncode_result = {
        -1: ['socket_set_wifi_unknow_fail', "未知错误，请联系管理员"],
        0: ['socket_set_wifi_success', "wifi设置成功。"],
        2: ['socket_set_wifi_need_pwd', "连接的目标wifi需要密码，请重新设置。"],
        4: ['socket_set_wifi_check_pwd', "连接激活失败，请检查密码是否输入正确。"],
        10: ['socket_set_wifi_check_ssid', "连接的目标wifi不存在，请检查机器人所在区域是否被目标wifi覆盖或稍后再试。"]
    }
    if judge_ssid(wifi_info['ssid']) == True:
        returncode = 0
    else:
        try:
            if wifi_info['password'] != NO_PASSWORD:
                returncode = subprocess.call(['bash', '/home/lemon/robot_ros_application/scripts/developer_tools/set_wifi.sh', wifi_info['ssid'], wifi_info['password']], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            else:
                returncode = subprocess.call(['bash', '/home/lemon/robot_ros_application/scripts/developer_tools/set_wifi.sh', wifi_info['ssid']], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if returncode == 0:
                if judge_ssid(wifi_info['ssid']) == False:
                    returncode = 4
        except:
            returncode = -1
    msg = {
        "cmd": "set_wifi",
        "result": returncode
    }
    sck_conn.send(_dict_to_bytes(msg))
    player.play_complete_voice(VOICE_FOLDER_PATH, returncode_result[returncode][PLAY_SET_WIFI_RESULT])
    if returncode == 0:
        ipv4 = get_wifi_ip()
        player.play_complete_voice(VOICE_FOLDER_PATH, "ip")
        player.play_single_voice(VOICE_FOLDER_PATH, ipv4.lower())

def get_current_area_wifi_scanned(sck_conn):
    """get_current_area_wifi_scanned
    Get wifi scanned in the current area

    @param None
    @type None
    @return msg
    @rtype dict

    msg = {
        "cmd": "get_current_area_wifi_scanned",
        "current_wifi_list": "[
            ['lejurobot', 'E8:1D:A8:25:90:5C', '77', 'WPA2', 'yes']
            [SSID1, BSSID1, SIGNAL1, SECURITY1, ACTIVE1]
            ...
        ]"
    }
    """

    scan_current_area_wifi_command = "nmcli -f SSID,BSSID,SSID-HEX,SIGNAL,SECURITY,ACTIVE device wifi"
    command_result = subprocess.check_output(scan_current_area_wifi_command, shell=True)
    partern = r'\n((?:\S.*\S)|(?: )) +(\w\w(?::\w\w){5})\s*[A-F0-9]+ +(\d+)\s*(\-{2}|\w+.?[\w|\.]+)\s*(\w+)'
    current_wifi_list = re.findall(partern, command_result)

    msg = {
        "cmd": "get_current_area_wifi_scanned",
        "current_wifi_list": current_wifi_list
    }
    sck_conn.send(_dict_to_bytes(msg))

def get_file_md5(sck_conn, file_name):
    file_path = None
    md5 = None
    folder_path = ""
    if os.path.dirname(file_name) == "":
        if file_name.endswith(DOWNLOAD_PYTHON_FILE_FLAG):
            folder_path = DOWNLOAD_EXEC_FILE_PATH
        elif file_name.endswith(DOWNLOAD_MUSIC_FILE_FLAG):
            folder_path = DOWNLOAD_MUSIC_FILE_PATH
        file_path = os.path.join(folder_path, file_name)
    else:
        file_path = file_name
    if os.path.isfile(file_path) == False:
        file_path = None
    else:
        md5 = hashlib.md5(open(file_path).read()).hexdigest()
    msg = {
        "cmd": "get_file_md5",
        "file_path": file_path,
        "md5": md5
    }
    sck_conn.send(_dict_to_bytes(msg))


        
class Socket_CMD_Runner(threading.Thread):
    def __init__(self):
        super(Socket_CMD_Runner, self).__init__()
        self.cmd_Queue = Queue.Queue()
        
    def run(self):
        while not rospy.is_shutdown():
            if not self.cmd_Queue.empty():
                cmd_data,connetcion = self.cmd_Queue.get()
                self.run_cmd(cmd_data,connetcion)
            else:
                time.sleep(0.01)
            
    def put(self,cmd,conn):
        self.cmd_Queue.put((cmd,conn))
        
    def run_cmd(self,data,conn):
        """执行一个socket指令
        :param data->包含指令名称等信息的dict
        :param conn->socket connection 对象 
        """
        try:       
            rospy.loginfo(data) 
            cmd = data['cmd']
            print('cmd: ', cmd)
        #    print('data',data["name"])
            if cmd == 'set_all_servo':
                async_do_job(set_all_servo, args=(data['id'], data['angle'], data['time'], conn, data['sendback']))
            elif cmd == 'set_head_servo':
                async_do_job(set_head_servo, args=(data['angle'], data['time'], conn,))
            elif cmd == 'get_all_servo':
                async_do_job(get_all_servo, args=(data['id'], conn,))
            elif cmd == 'set_multiple_all_servo':
                async_do_job(set_multiple_all_servo, args=(conn, data['act_frames'],data['start'],data['stop']))
            elif cmd == 'set_body_state':
                async_do_job(NODE.set_act_state, args=(data['state'],))
            elif cmd == 'ls_node':
                async_do_job(get_act_nodes, args=(conn,))
            elif cmd == 'get_execution_status':
                async_do_job(get_execution_status, args=(conn,))
            elif cmd == 'run_node':
                async_do_job(exec_download_file, args=(data['path'],))
            elif cmd == 'stop_node':
                async_do_job(reset_body_state, args=())
            elif cmd == 'query_node_state':
                async_do_job(query_node_state, args=(conn,))
            elif cmd == 'get_login_account':
                async_do_job(get_login_account, args=(conn,))
            elif cmd == 'send_controller_cmd':
                async_do_job(handle_controller_cmd, args=(conn, data['key'],))
            elif cmd == 'send_controller_reset':
                async_do_job(handle_controller_reset, args=(conn,))
            elif cmd == 'get_zero_value':
                async_do_job(get_zero_value, args=(conn,))
            elif cmd == 'view_zero_position':
                async_do_job(set_zero_servo, args=(conn,))
            elif cmd == 'set_zero_servo':
                async_do_job(set_zero_servo,
                            args=(conn, data['sendback'], data['slowspeed'], data['id'], data['zerovalue']))
            elif cmd == 'virtual_control_panel':
                async_do_job(virtual_control_panel, args=(conn, data['key'],))
            elif cmd == 'view_action':
                async_do_job(view_action, args=(data['frame'], conn))
            elif cmd == 'stop_action':
                async_do_job(stop_action, args=(conn, data["id"]))
            elif cmd == 'get_init_value':
                async_do_job(get_init_value, args=(conn,))
            elif cmd == 'reset_bodyhub':
                async_do_job(reset_bodyhub, args=(conn,)) 
            elif cmd == 'set_imuState':
                async_do_job(set_imuState, args=(conn, data["imu_state"]))
            elif cmd == 'read_imuDate':
                async_do_job(read_imuDate, args=(conn,))
            elif cmd == 'get_cameras_status':
                async_do_job(get_cameras_status, args=(conn,))
            elif cmd == 'get_soundcard_status':
                async_do_job(get_soundcard_status, args=(conn,))
            elif cmd == 'get_servo_status':
                async_do_job(get_servo_status, args=(conn,))
            elif cmd == 'get_roban_name':
                async_do_job(get_roban_name,args=(conn,))
            elif cmd == 'set_roban_name':
                async_do_job(set_roban_name,args=(conn,data["name"]))
            elif cmd == 'set_robot_back_btns_feature':
                async_do_job(set_robot_back_btns_feature,args=(conn, data["robotBackBtns"]))
            elif cmd == 'get_robot_back_btns_feature':
                async_do_job(get_robot_back_btns_feature,args=(conn,))
            elif cmd == 'set_wifi':
                async_do_job(set_wifi,args=(conn, data["wifi_info"]))
            elif cmd == 'get_connected_wifi_list':
                async_do_job(get_connected_wifi_list,args=(conn,))
            elif cmd == 'del_wifi':
                async_do_job(del_wifi,args=(conn,data["name"]))
            elif cmd == 'kill_RGBD':
                async_do_job(kill_slam_node,args=(conn,))
            elif cmd == 'get_current_area_wifi_scanned':
                async_do_job(get_current_area_wifi_scanned,args=(conn,))
            elif cmd == 'get_file_md5':
                async_do_job(get_file_md5, args=(conn, data["file_name"]))
            else:
                print("[WARN] WIP...")

        except Exception as err:
            logging.error(err)

joinData = ""
cmd_runner = Socket_CMD_Runner()
cmd_runner.start()
def handle_sck_data(_datas, conn):
    """process socket json cmd
    :param data:
    :param conn:
    :return:
    """
    global joinData
    data = None
    _datas = _datas.decode("utf-8")
    cmd_list = []
    joinData += _datas
    split_index = 0
    for index, string in enumerate(joinData):
        if string == "}":
            try:
                data = json.loads(joinData[split_index:index + 1])
                cmd_list.append(data)
                split_index = index + 1
            except Exception as e:
                continue
    joinData = joinData[split_index:]
    if len(cmd_list) == 0:
        return
    
    for cmd_info in cmd_list:
        cmd_runner.put(cmd_info,conn)
# send device list in current demo
def send_device_list(data):
    rospy.loginfo("camera status: %d", data.camera_status)
    rospy.loginfo("controller status: %d", data.controller_status)
    if not current_sock is None:
        msg = {
            'cmd': 'demo_device_list',
            'camera_status': data.camera_status,
            'controller_status': data.controller_status
        }
        current_sock.send(_dict_to_bytes(msg))


# send device list in current demo
def send_device_list(data):
    rospy.loginfo("camera status: %d", data.camera_status)
    rospy.loginfo("controller status: %d", data.controller_status)
    if not current_sock is None:
        msg = {
            'cmd': 'demo_device_list',
            'camera_status': data.camera_status,
            'controller_status': data.controller_status
        }
        current_sock.send(_dict_to_bytes(msg))


def thread_handle_connect(conn):
    """wait for socket client bytes buffer data,
    try to reconnect when timeout
    :param conn:
    :return:
    """
    global current_sock
    global con_is_reset
    while not rospy.is_shutdown():
        try:  
              
            data = conn.recv(8192*3, 0x40)        
            rospy.loginfo("recviving.....")      
            if not data:
                print('[WARN] lost socket connection ...')
                conn.close()
                current_sock = None
                NODE.walkflag = False
                con_is_reset = True
                break   
            handle_sck_data(data, conn)       
        except BlockingIOError:
            continue
        except Exception as e:
            err = e.args[0]
            if err == 'timed out':
                continue
            else:
                print("thread_handle_connect exiting",sys.exc_info())
                conn.close()
                current_sock = None
                con_is_reset = True
                NODE.walkflag = False
                break
    print("thread_handle_connect exit")

def get_socket():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    server.setsockopt(socket.SOL_TCP, socket.TCP_KEEPIDLE, 5)
    server.setsockopt(socket.SOL_TCP, socket.TCP_KEEPINTVL, 3)
    server.setsockopt(socket.SOL_TCP, socket.TCP_KEEPCNT, 3)
    server.bind((HOST, PORT))
    server.listen(1)
    server.settimeout(None)
    return server

CONFIG_FOLDER_PATH = "/home/lemon"
CONFIG_FILENAME = ".robanConfig.json"
CONFIG_FILENAME_2 = ".lejuconfig/robanConfig.json"
CONFIG_FILE_PATH = os.path.join(CONFIG_FOLDER_PATH, CONFIG_FILENAME)
CONFIG_FILE_PATH_2 = os.path.join(CONFIG_FOLDER_PATH, CONFIG_FILENAME_2)
#get_robot_name 若配置文件存在，则返回文件中的机器人名称；否则返回roban_default
def get_roban_name(sck_conn):
    roban_name = get_robot_name()
    msg = {
        "cmd": "get_roban_name",
        "name": roban_name
    }
    sck_conn.send(_dict_to_bytes(msg))

#set_roban_name 
def set_roban_name(sck_conn, name):
    result = set_robot_name(name)
    if result[0] != result[1]:
        state = "fail"
    else:
        state = result[0]

    msg = {
        'cmd'  : 'set_roban_name',
        'state' : state
    }
    sck_conn.send(_dict_to_bytes(msg))

def modify_backbtnsconfig_server(req):
    global timestamp
    if req.modifyMode == 'writting':
        timestamp = -1
        return timestamp
    if req.modifyMode == 'done':
        timestamp = time.mktime(datetime.now().timetuple())
        return timestamp
    if req.modifyMode == 'check_time_stamp':
        return timestamp
             
def main():
    """main entry
    """
    def rosShutdownHook():
        rospy.loginfo("Shutting down socket server")
        server.close()
    rospy.on_shutdown(rosShutdownHook)
    rospy.Subscriber("/ActRunner/DeviceList", DeviceList, send_device_list)
    modify_backbtnsconfig_srv = rospy.Service('/ros_socket_node/modify_backbtnsconfig', ModifyBtnsConfig, modify_backbtnsconfig_server)
    # rospy.Subscriber('terminate_current_process', String, executing_end)
    # rospy.Subscriber('/Finish', String, executing_end)
    threading.Thread(target=actmoto).start()

    global current_sock
    server =  get_socket()
    while not rospy.is_shutdown():
        try:
            print('listening...')
            conn, addr = server.accept()
            current_sock = conn
            print('handling connection from %s' % (addr,))
            threading.Thread(target=thread_handle_connect, args=(conn,)).start()
        except Exception as e:
            print(e)
    
    

if __name__ == '__main__':
    main()
    


