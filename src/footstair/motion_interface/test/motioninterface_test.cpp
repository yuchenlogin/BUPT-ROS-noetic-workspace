#include "motionInterface.h"
#include <signal.h>
#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>

Motion::MotionInterface *motion;
static std::string robotPlatform;
std::vector<uint16_t> jointIds{
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
    13, 14, 15, 16,
    17, 18, 19, 20,
    21, 22};
std::vector<Motion::JointParam_t> jointCmd(jointIds.size());

ros::Publisher imuPub;
ros::Publisher jointPosPub;
ros::Publisher jointVelPub;
ros::Publisher jointTorquePub;
ros::Publisher lFootFTPub;
ros::Publisher rFootFTPub;
ros::Publisher jointCmdPosPub;
ros::Publisher jointCmdVelPub;
ros::Publisher jointCmdTorquePub;

void topicInit()
{
  ros::NodeHandle nh;
  imuPub = nh.advertise<std_msgs::Float64MultiArray>("/sensor/imu", 10);
  jointPosPub = nh.advertise<std_msgs::Float64MultiArray>("/sensor/joint/position", 10);
  jointVelPub = nh.advertise<std_msgs::Float64MultiArray>("/sensor/joint/velocity", 10);
  jointTorquePub = nh.advertise<std_msgs::Float64MultiArray>("/sensor/joint/torque", 10);
  lFootFTPub = nh.advertise<std_msgs::Float64MultiArray>("/sensor/lFootFT", 10);
  rFootFTPub = nh.advertise<std_msgs::Float64MultiArray>("/sensor/rFootFT", 10);
  jointCmdPosPub = nh.advertise<std_msgs::Float64MultiArray>("/command/joint/position", 10);
  jointCmdVelPub = nh.advertise<std_msgs::Float64MultiArray>("/command/joint/velocity", 10);
  jointCmdTorquePub = nh.advertise<std_msgs::Float64MultiArray>("/command/joint/torque", 10);
}

void topicPub(Motion::SensorParam sensor)
{
  std_msgs::Float64MultiArray f64ArrayMsg;
  uint16_t number;
  f64ArrayMsg.data.resize(9);
  if (sensor.imu.size() > 0)
  {
    f64ArrayMsg.data[0] = sensor.imu[0].angularVel.x * (180.0 / M_PI);
    f64ArrayMsg.data[1] = sensor.imu[0].angularVel.y * (180.0 / M_PI);
    f64ArrayMsg.data[2] = sensor.imu[0].angularVel.z * (180.0 / M_PI);
    f64ArrayMsg.data[3] = sensor.imu[0].linearAcc.x;
    f64ArrayMsg.data[4] = sensor.imu[0].linearAcc.y;
    f64ArrayMsg.data[5] = sensor.imu[0].linearAcc.z;
    f64ArrayMsg.data[6] = sensor.imu[0].magnetic.x;
    f64ArrayMsg.data[7] = sensor.imu[0].magnetic.y;
    f64ArrayMsg.data[8] = sensor.imu[0].magnetic.z;
  }
  imuPub.publish(f64ArrayMsg);

  number = sensor.joint.size();
  f64ArrayMsg.data.resize(number);
  for (uint16_t i = 0; i < number; i++)
    f64ArrayMsg.data[i] = sensor.joint[i].position;
  jointPosPub.publish(f64ArrayMsg);

  f64ArrayMsg.data.resize(number);
  for (uint16_t i = 0; i < number; i++)
    f64ArrayMsg.data[i] = sensor.joint[i].velocity;
  jointVelPub.publish(f64ArrayMsg);

  f64ArrayMsg.data.resize(number);
  for (uint16_t i = 0; i < number; i++)
    f64ArrayMsg.data[i] = sensor.joint[i].torque;
  jointTorquePub.publish(f64ArrayMsg);

  f64ArrayMsg.data.resize(6);
  if (sensor.force.size() > 0)
  {
    f64ArrayMsg.data[0] = sensor.force[0].force.x;
    f64ArrayMsg.data[1] = sensor.force[0].force.y;
    f64ArrayMsg.data[2] = sensor.force[0].force.z;
    f64ArrayMsg.data[3] = sensor.force[0].moment.x;
    f64ArrayMsg.data[4] = sensor.force[0].moment.y;
    f64ArrayMsg.data[5] = sensor.force[0].moment.z;
  }
  lFootFTPub.publish(f64ArrayMsg);

  f64ArrayMsg.data.resize(6);
  if (sensor.force.size() > 1)
  {
    f64ArrayMsg.data[0] = sensor.force[1].force.x;
    f64ArrayMsg.data[1] = sensor.force[1].force.y;
    f64ArrayMsg.data[2] = sensor.force[1].force.z;
    f64ArrayMsg.data[3] = sensor.force[1].moment.x;
    f64ArrayMsg.data[4] = sensor.force[1].moment.y;
    f64ArrayMsg.data[5] = sensor.force[1].moment.z;
  }
  rFootFTPub.publish(f64ArrayMsg);

  number = jointCmd.size();
  f64ArrayMsg.data.resize(number);
  for (uint16_t i = 0; i < number; i++)
    f64ArrayMsg.data[i] = jointCmd[i].position;
  jointCmdPosPub.publish(f64ArrayMsg);

  f64ArrayMsg.data.resize(number);
  for (uint16_t i = 0; i < number; i++)
    f64ArrayMsg.data[i] = jointCmd[i].velocity;
  jointCmdVelPub.publish(f64ArrayMsg);

  f64ArrayMsg.data.resize(number);
  for (uint16_t i = 0; i < number; i++)
    f64ArrayMsg.data[i] = jointCmd[i].torque;
  jointCmdTorquePub.publish(f64ArrayMsg);
}

void test()
{
  bool result = false;
  topicInit();
  std::vector<Motion::JointParam_t> jointData(jointIds.size());
  std::vector<Motion::ForceParam_t> forceData(2);
  std::vector<Motion::ImuParam_t> imuData(1);
  result = motion->getJointData(jointIds, jointData);
  if(result == false)
  {
    ROS_ERROR("getJointData failed!"); 
    motion->exit();
    return;
  }
  jointCmd = jointData;
  motion->setJointPosition(jointIds, jointCmd);
  result = motion->getForce(forceData);
  result = motion->getImu(imuData);

  Motion::SensorParam sensorData(jointIds.size(), 2, 1);
  result = motion->getSensor(sensorData);
  motion->setJointPosition(jointIds, sensorData.joint);

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    motion->getSensor(sensorData);
    motion->setJointPosition(jointIds, jointCmd);
    topicPub(sensorData);
    loop_rate.sleep();
  }
}

void sigintHandler(int sig)
{
  motion->exit();
  ros::shutdown();
  usleep(200 * 1000);
  exit(EXIT_SUCCESS);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motioninterface_test");
  ros::NodeHandle nh;
  signal(SIGINT, sigintHandler);
  if (argc >= 2 && (strcmp(argv[1], "sim") == 0))
  {
    nh.setParam("robot_platform", "sim");
  }
  else if (argc >= 2 && (strcmp(argv[1], "roban") == 0))
  {
    nh.setParam("robot_platform", "roban");
  }
  nh.param<std::string>("robot_platform", robotPlatform, "roban");
  ros::AsyncSpinner spinner(5);
  spinner.start();

  Motion::motionInterfaceSetup(robotPlatform, motion);
  if (motion->init() == false)
  {
    ROS_ERROR("error: motion init failed!");
    return 1;
  }

  test();

  return 0;
}