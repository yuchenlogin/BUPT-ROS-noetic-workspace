#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include "imuData.h"

extern ImuData imuDataObj;
ros::Subscriber jy901_sub;

void jy901Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  imuDataObj.setGyro(msg->data[0], msg->data[1], msg->data[2]);
  imuDataObj.setAcc(msg->data[3], msg->data[4], msg->data[5]);
  imuDataObj.setEuler(msg->data[6], msg->data[7], msg->data[8]);
#if 0
  std::cout
      << "gx:" << msg->data[0]
      << std::setw(10) << "gy:" << msg->data[1]
      << std::setw(10) << "gz:" << msg->data[2]
      << std::setw(10) << "ax:" << msg->data[3]
      << std::setw(10) << "ay:" << msg->data[4]
      << std::setw(10) << "az:" << msg->data[5]
      << std::setw(10) << "R:" << msg->data[6]
      << std::setw(10) << "P:" << msg->data[7]
      << std::setw(10) << "Y:" << msg->data[8]
      << std::setw(5) << "\r";
#endif
}

void jy901ModuleInit()
{
  ros::NodeHandle nh;
  jy901_sub = nh.subscribe("/jy901Module_node/jy901Data", 1, jy901Callback);
}