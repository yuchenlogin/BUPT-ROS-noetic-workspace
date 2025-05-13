#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include "motionInterface.h"

#define JOINT_NUM_MAX 22
#define JOINT_NUM_SENSOR 22
#define JOINT_NUM_CTL 22
#define FS_NUM 2
#define IMU_NUM 1

#define ROS_TOPIC_ON false

ros::Publisher jointDataPub;
ros::Publisher forceLDataPub;
ros::Publisher forceRDataPub;
ros::Publisher imuDataPub;

void JointDemo()
{
  uint8_t number = 22;

  std::vector<uint16_t> jointIds{
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
      13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
  JointParam jointData(22);

  std::vector<uint16_t> jointCtlIds{
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
      13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
  JointParam jointParam(22);

  bool result = false;
  result = motionPhy.getJointPosition(jointIds, jointData.position);
  if (result == false)
  {
    ROS_ERROR("error: motionPhy.getJointPosition failed!");
    return;
  }

#if !ROS_TOPIC_ON
  std::cout << std::endl
            << "current joint postion: " << std::endl;
  for (uint8_t i = 0; i < number; i++)
    std::cout << "  ID" << jointIds[i] << ": " << jointData.position[i] << "\n";
#else
  std_msgs::Float64MultiArray f64MultiMsg;
  f64MultiMsg.data.resize(number);
  for (uint8_t i = 0; i < number; i++)
    f64MultiMsg.data[i] = jointData.position[i];
  jointDataPub.publish(f64MultiMsg);
#endif

  jointParam.position.assign(jointData.position.begin(), jointData.position.end()); // jointParam.position[index] = value;
  motionPhy.setJointPosition(jointCtlIds, jointParam.position);
}

void GetForceDemo()
{
  std::vector<ForceParam_t> forceData(2); // left foot, right foot

  bool result = false;
  result = motionPhy.getForce(forceData);
  if (result == false)
  {
    ROS_ERROR("error: motionPhy.getForce failed!");
    return;
  }
#if !ROS_TOPIC_ON
  std::cout << std::endl
            << "left foot force: " << std::endl;
  std::cout << forceData[0].force.x << "  " << forceData[0].force.y << "  " << forceData[0].force.z << "  "
            << forceData[0].moment.x << "  " << forceData[0].moment.y << "  " << forceData[0].moment.z;
  std::cout << std::endl;
  std::cout << "right foot force: " << std::endl;
  std::cout << forceData[1].force.x << "  " << forceData[1].force.y << "  " << forceData[1].force.z << "  "
            << forceData[1].moment.x << "  " << forceData[1].moment.y << "  " << forceData[1].moment.z;
  std::cout << std::endl;
#else
  std_msgs::Float64MultiArray f64MultiMsg;
  f64MultiMsg.data.resize(6);
  f64MultiMsg.data[0] = forceData[0].force.x;
  f64MultiMsg.data[1] = forceData[0].force.y;
  f64MultiMsg.data[2] = forceData[0].force.z;
  f64MultiMsg.data[3] = forceData[0].moment.x;
  f64MultiMsg.data[4] = forceData[0].moment.y;
  f64MultiMsg.data[5] = forceData[0].moment.z;
  forceLDataPub.publish(f64MultiMsg);
  f64MultiMsg.data[0] = forceData[1].force.x;
  f64MultiMsg.data[1] = forceData[1].force.y;
  f64MultiMsg.data[2] = forceData[1].force.z;
  f64MultiMsg.data[3] = forceData[1].moment.x;
  f64MultiMsg.data[4] = forceData[1].moment.y;
  f64MultiMsg.data[5] = forceData[1].moment.z;
  forceRDataPub.publish(f64MultiMsg);
#endif
}

void GetImuDemo()
{
  std::vector<ImuParam_t> imuData(IMU_NUM);

  bool result = false;
  result = motionPhy.getImu(imuData);
  if (result == false)
  {
    ROS_ERROR("error: motionPhy.getImu failed!");
    return;
  }
#if !ROS_TOPIC_ON
  std::cout << std::endl
            << "imu data: " << std::endl;
  std::cout << imuData[0].angularVel.x << "  " << imuData[0].angularVel.y << "  " << imuData[0].angularVel.z << "  "
            << imuData[0].linearAcc.x << "  " << imuData[0].linearAcc.y << "  " << imuData[0].linearAcc.z;
  std::cout << std::endl;
#else
  std_msgs::Float64MultiArray f64MultiMsg;
  f64MultiMsg.data.resize(6);
  f64MultiMsg.data[0] = imuData[0].angularVel.x;
  f64MultiMsg.data[1] = imuData[0].angularVel.y;
  f64MultiMsg.data[2] = imuData[0].angularVel.z;
  f64MultiMsg.data[3] = imuData[0].linearAcc.x;
  f64MultiMsg.data[4] = imuData[0].linearAcc.y;
  f64MultiMsg.data[5] = imuData[0].linearAcc.z;
  imuDataPub.publish(f64MultiMsg);
#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_interface_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(5);
  spinner.start();

#if ROS_TOPIC_ON
  jointDataPub = nh.advertise<std_msgs::Float64MultiArray>("/motionInterface/joint/position", 1000);
  forceLDataPub = nh.advertise<std_msgs::Float64MultiArray>("/motionInterface/force/left", 1000);
  forceRDataPub = nh.advertise<std_msgs::Float64MultiArray>("/motionInterface/force/right", 1000);
  imuDataPub = nh.advertise<std_msgs::Float64MultiArray>("/motionInterface/imu", 1000);
#endif

  if (motionPhy.init() == false)
  {
    ROS_ERROR("error: robot init failed!");
    exit(1);
  }

#if !ROS_TOPIC_ON
  JointDemo();
  GetForceDemo();
  GetImuDemo();
#else
  ros::Rate loopRate(100);
  while (ros::ok())
  {
    JointDemo();
    GetForceDemo();
    GetImuDemo();
    loopRate.sleep();
  }
#endif

  motionPhy.exit();
  return 0;
}