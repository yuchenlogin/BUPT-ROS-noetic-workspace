#! /usr/bin/python
import sys
import os
import signal
import json
import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool
import array
from bodyhub.srv import *  # for SrvState.srv
from ros_AIUI_node.srv import demoIsRunning
import rospkg
sys.path.append(rospkg.RosPack().get_path('ros_actions_node')+ '/scripts')
from lejulib import *
from motion.motionControl import SetBodyhubTo_setStatus, GetBodyhubStatus
SERVO = client_action.SERVO

class HeadTowardSound():
    def __init__(self):
        self.ignore_head_toward_sound = False

    def set_head_servo(self, angles):
        """set head servos angle

        :param angles:[angle, 0.0]
        :return:
        """
        array_angles = array.array("d", angles)
        SERVO.HeadJointTransfer(array_angles)
        SERVO.MotoWait()

    def terminate(self):
        """Terminate all threads
        """
        pid = os.getpid()
        print('kill pid = {}'.format(pid))
        os.kill(pid, signal.SIGKILL)
        sys.exit()


    def get_angle_value(self, angle):
        """

        :param angle: the angle from micarrays
        :return: [-90.0 , 90.0]
        """
        angle = 300.0 - angle
        if angle > 180.0 and angle <= 300.0:
            angle = angle - 360.0
            return angle
        else:
            return angle


    def moto_destination(self, preAngle, angle):
        """

        :param angle: the angle from micarrays
        :return:
        """
        angle_value = self.get_angle_value(angle)
        moto_angle = preAngle + angle_value
        if moto_angle > 90.0:
            moto_angle = 90.0
        if moto_angle < -90.0:
            moto_angle = -90.0
        rospy.loginfo("angle:%s", str(moto_angle))
        self.set_head_servo([moto_angle, 0.0])
        return moto_angle

    def pause_head_toward_sound(self, req):
        if req.data:
            self.ignore_head_toward_sound = True 
            msg = 'not head toward sound mode'
        else:
            self.ignore_head_toward_sound = False 
            msg = 'head toward sound mode'
        return True, msg

    def main(self):
        try:
            rospy.init_node("head_toward_sound", anonymous=False)
            sound_topic = "/micarrays/wakeup"
            preAngle = client_action.get_servo_value(21)
            demo_running_status_client = rospy.ServiceProxy("aiui/demo_running_status", demoIsRunning)
            rospy.Service("/aiui/pause_head_toward_sound", SetBool, self.pause_head_toward_sound)

            while not rospy.is_shutdown():
                msg = rospy.wait_for_message(sound_topic, String)
                if self.ignore_head_toward_sound:
                    rospy.sleep(0.1)
                    rospy.logwarn("sound continue!")
                    continue
                data = demo_running_status_client()
                is_demo_running = data.demoIsRunning
                if is_demo_running == False:
                    data = json.loads(msg.data.replace("'", '"'))
                    angle = float(data['angle'])
                    rospy.loginfo("angle-----:%s", str(angle))
                    current_status = GetBodyhubStatus()
                    if current_status.data != "ready" and current_status.data != "pause":
                        SetBodyhubTo_setStatus(2)
                    preAngle = self.moto_destination(preAngle, angle)

        except Exception as err:
            serror(err)
        finally:
            finishsend()


if __name__ == '__main__':
    hts = HeadTowardSound()
    hts.main()
