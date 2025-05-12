#include <std_msgs/Float64MultiArray.h>
#include "imuData.h"
#include "ros/ros.h"

extern ImuData torsoImu;
ros::Subscriber jy901_sub;
double GRAVITY_ACC = 9.8;

void jy901Callback(const std_msgs::Float64MultiArray::ConstPtr &msg) {

  torsoImu.setGyro(msg->data[0],msg->data[1], msg->data[2]);
  torsoImu.setAcc( msg->data[3]*GRAVITY_ACC,msg->data[4]*GRAVITY_ACC, msg->data[5]*GRAVITY_ACC);
  // torsoImu.setAttitude(msg->data[7], msg->data[6],  -msg->data[8]);
  torsoImu.datafusion();
}

void jy901ModuleInit() {
  ros::NodeHandle nh;
  jy901_sub =
      nh.subscribe("/jy901Module_node/jy901Data", 1, jy901Callback);
}
