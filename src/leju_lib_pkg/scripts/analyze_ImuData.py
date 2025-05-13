#! /usr/bin/ python3
import rospy
from sensor_msgs.msg import Imu

IMU_TOPIC_FREQUENCE = 200  # 200Hz
SENSOR_IMU_TOPIC = "/MediumSize/SensorHub/Imu"

class ImuData:
    def __init__(self, stable_imu_duration, limit=[]):
        """
        Analyze IMU data
        @param stable_imu_duration: imu data keep stable time (unit: second)
        @param limit: limit angular velocity range, use to determine whether it is stable
        """
        self.number_of_imu_data = int(stable_imu_duration * IMU_TOPIC_FREQUENCE)
        self.limit = limit
        self.number_of_data_in_limit_range = 0
        rospy.Subscriber(SENSOR_IMU_TOPIC, Imu, self.imu_callback, queue_size=1)

    def imu_callback(self, msg):
        if abs(msg.angular_velocity.x) <= self.limit[0] and abs(msg.angular_velocity.y) <= self.limit[1] and abs(msg.angular_velocity.z) <= self.limit[2]:
            self.number_of_data_in_limit_range += 1
        else:
            self.number_of_data_in_limit_range = 0

    @property
    def is_stablized(self):
        if self.number_of_data_in_limit_range >= self.number_of_imu_data:
            self.number_of_data_in_limit_range = 0
            return True
        return False
