#include "ros/ros.h"
#include <iostream>
#include <thread>
#include <signal.h>
#include "walking.h"
#include "util/loopTime.h"
#include "util/keyboard.h"
#include "motion_interface/motionInterface.h"
#include "imuDatafusion/imuData.h"

#define FS_NUM 2
#define IMU_NUM 1

extern void rosDebugInit();
extern void debugPublish();

static std::string robotPlatform;
Walking::Walking mWalk;

JointParam jointData(22);
JointParam jointParam(22);
extern motionPhy_t motionPhy;

uint8_t imuType = 0;
ImuData imuDataObj;

namespace Eigen
{
  typedef Matrix<double, 6, 1> Vector6d;
}

double_t fifthSpline(double_t duration, double_t start, double_t stop, double_t t)
{
  if (t < 0)
    t = 0;
  if (t > duration)
    t = duration;
  Eigen::Vector6d bound;
  Eigen::Matrix<double, 6, 6> m6X6;
  Eigen::Vector6d coef; //coef--(a0,a1,a2,a3,a4,a5)
  Eigen::Vector6d tVect;
  bound << start, stop, 0, 0, 0, 0;
  m6X6 << 1, 0, 0, 0, 0, 0,
      1, duration, pow(duration, 2), pow(duration, 3), pow(duration, 4), pow(duration, 5),
      0, 1, 0, 0, 0, 0,
      0, 1, 2 * duration, 3 * pow(duration, 2), 4 * pow(duration, 3), 5 * pow(duration, 4),
      0, 0, 2, 0, 0, 0,
      0, 0, 2, 6 * duration, 12 * pow(duration, 2), 20 * pow(duration, 3);
  coef = m6X6.inverse() * bound;
  tVect << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);
  return tVect.dot(coef);
}

double_t linearPlan(double_t duration, double_t start, double_t stop, double_t t)
{
  if (t < 0)
    t = 0;
  if (t > duration)
    t = duration;
  return (stop - start) / duration * t + start;
}

void jointSetPositionWithSpeed(double_t duration, double_t timeStep, std::vector<uint16_t> &ids, std::vector<double_t> &currentPosition, std::vector<double_t> &goalPosition)
{
  uint16_t number = ids.size();
  uint16_t numberOfFrame = duration / timeStep;
  std::vector<double_t> jointV(number, 0);
  ros::Rate loopRate(1.0 / timeStep);
  for (uint16_t count = 1; count <= numberOfFrame; count++)
  {
    for (uint16_t i = 0; i < number; i++)
    {
      jointV[i] = fifthSpline(numberOfFrame, currentPosition[i], goalPosition[i], count);
    }
    motionPhy.setJointPosition(ids, jointV);
    loopRate.sleep();
    if (!ros::ok())
      break;
  }
}

void stand()
{
  std::cout << "standing" << std::endl;
  std::vector<uint16_t> jointIds{
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
      13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
  std::vector<double_t> standPos{
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, -75, -10, 0,
      0, 75, 10, 0,
      0, 0};

  ros::Rate loopRate(1.0 / mWalk.stepParam.dt);
  while (!motionPhy.getJointPosition(jointIds, jointData.position) && ros::ok())
  {
    loopRate.sleep();
  }
  if(!ros::ok())
    return;

  jointParam.position.assign(standPos.begin(), standPos.end());
#if 1
  mWalk.getInitJointValue(jointParam.position);
#endif

  jointSetPositionWithSpeed(1.0, mWalk.stepParam.dt, jointIds, jointData.position, jointParam.position);
  std::cout << "stand sucess" << std::endl;
}

bool imuRead(double_t *gyro, double_t *acc)
{
  bool result = false;
  std::vector<ImuParam_t> imuData(IMU_NUM);
  result = motionPhy.getImu(imuData);
  if (result == false)
  {
    ROS_ERROR("read imu fail!");
    return false;
  }
  gyro[0] = imuData[0].angularVel.x;
  gyro[1] = imuData[0].angularVel.y;
  gyro[2] = imuData[0].angularVel.z;
  acc[0] = imuData[0].linearAcc.x;
  acc[1] = imuData[0].linearAcc.y;
  acc[2] = imuData[0].linearAcc.z;
  // std::cout << "gyro:  " << gyro[0] << gyro[1] << gyro[2] << std::endl;
  return true;
}

void imuInit(uint8_t type)
{
  imuType = type;
  if (imuType == 0)
  {
    uint16_t number = 20;
    uint16_t count = number;
    double gyro[3], acc[3];
    double gyroSum[3], accSum[3];

    ROS_INFO("calibrate imu...");
    ros::Rate loopRate(100);
    while (ros::ok() && count)
    {
      if (imuRead(gyro, acc) == true)
      {
        count--;
        for (uint16_t i = 0; i < 3; i++)
        {
          gyroSum[i] += gyro[i];
          accSum[i] += acc[i];
        }
      }
      loopRate.sleep();
    }
    for (uint16_t i = 0; i < 3; i++)
    {
      gyro[i] = gyroSum[i] / number;
      acc[i] = accSum[i] / number;
    }
    imuDataObj.calibrateGyro(gyro[0], gyro[1], gyro[2]);
#if 1
    std::cout
        << "imuOffset gx:" << gyro[0]
        << std::setw(10) << "gy:" << gyro[1]
        << std::setw(10) << "gz:" << gyro[2]
        << std::setw(10) << "\n";
#endif
    ROS_INFO("calibrate imu over.");

    imuDataObj.setGyroLpf(100, 20);
    imuDataObj.setAccLpf(100, 20);

    count = 100;
    while (ros::ok() && count)
    {
      if (imuRead(gyro, acc) == true)
      {
        count--;
        imuDataObj.setGyro(gyro[0], gyro[1], gyro[2]);
        imuDataObj.setAcc(acc[0], acc[1], acc[2]);
        imuDataObj.datafusion();
      }
      loopRate.sleep();
    }
  }
  else
  {
    extern void jy901ModuleInit(void);
    jy901ModuleInit();
  }
}

void sensorUpdate()
{
  bool result = false;

#if 1
  std::vector<uint16_t> jointIds{
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  motionPhy.getJointPosition(jointIds, jointData.position);
  for (uint16_t i = 0; i < 12; i++)
    mWalk.float_q[FLOATING_CONFIG_NUM + i] = jointData.position[i] * Util::TO_RADIAN;
#else
  mWalk.float_q.segment<JOINT_NUM>(FLOATING_CONFIG_NUM) = mWalk.float_q_ref.segment<JOINT_NUM>(FLOATING_CONFIG_NUM);
#endif

  std::vector<ForceParam_t> forceData(FS_NUM);
  motionPhy.getForce(forceData);
  mWalk.lFootFT << forceData[0].force.x, forceData[0].force.y, forceData[0].force.z, forceData[0].moment.x, forceData[0].moment.y, forceData[0].moment.z;
  mWalk.rFootFT << forceData[1].force.x, forceData[1].force.y, forceData[1].force.z, forceData[1].moment.x, forceData[1].moment.y, forceData[1].moment.z;

  std::vector<ImuParam_t> imuData(IMU_NUM);
  if (imuType == 0)
  {
    result = motionPhy.getImu(imuData);
    if (result == true)
    {
      imuDataObj.setGyro(imuData[0].angularVel.x, imuData[0].angularVel.y, imuData[0].angularVel.z);
      imuDataObj.setAcc(imuData[0].linearAcc.x, imuData[0].linearAcc.y, imuData[0].linearAcc.z);
      imuDataObj.datafusion();
    }
  }
  imuData[0] = imuDataObj.getData();
  mWalk.imuGyro << imuData[0].angularVel.x, imuData[0].angularVel.y, imuData[0].angularVel.z;
  mWalk.imuAcc << imuData[0].linearAcc.x, imuData[0].linearAcc.y, imuData[0].linearAcc.z;
  mWalk.imuEuler << imuData[0].eulerAngle.roll, imuData[0].eulerAngle.pitch, 0.0;
}

void sendJointValue()
{
  std::vector<uint16_t> jointIds{
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  for (uint16_t i = 0; i < 12; i++)
  {
    jointParam.position[i] = mWalk.float_q_ref[FLOATING_CONFIG_NUM + i] * Util::TO_DEGREE;
  }
  motionPhy.setJointPosition(jointIds, jointParam.position);
}

void *control_thread(void *ptr)
{
  LoopTime loopTime(mWalk.stepParam.dt);
  TimeoutCheck timeout(mWalk.stepParam.dt);
  while (ros::ok())
  {
    sensorUpdate();
    mWalk.run();
    sendJointValue();
    debugPublish();
    loopTime.sleep();
    if (strcmp(robotPlatform.c_str(), "sim") != 0)
    {
      timeout.check("control_thread");
    }
  }
  std::cout << "control_thread exited normally.\n";
  return NULL;
}

void walkingParamInit()
{
  if(strcmp(robotPlatform.c_str(), "sim") == 0)
  {
    mWalk.setTimeStep(0.02);
    mWalk.setHipOffset(0.0, 0.0);
  }
  else
  {
    mWalk.setTimeStep(0.01);
    mWalk.setHipOffset(2.0, 2.0);
  }
  mWalk.WalkingInit();
}

void walkingExit()
{
  motionPhy.exit();
  ros::shutdown();
}

void sigintHandler(int sig)
{
  motionPhy.exit();
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "anti_disturbance_node");
  int res;
  res = system(". /home/lemon/robot_ros_application/catkin_ws/devel/setup.sh;rosservice call /MediumSize/BodyHub/StateJump 2 'external_control'");
  ros::NodeHandle nh;
  signal(SIGINT, sigintHandler);
  nh.param<std::string>("robot_platform", robotPlatform, "roban");

  if (argc >= 2 && (strcmp(argv[1], "sim") == 0))
  {
    nh.setParam("robot_platform", "sim");
    nh.param<std::string>("robot_platform", robotPlatform, "sim");
  }

  ros::AsyncSpinner spinner(5);
  spinner.start();

  motionInterfaceSetup(robotPlatform);
  if (motionPhy.init() == false)
  {
    ROS_ERROR("motionPhy.init failed!");
    exit(EXIT_FAILURE);
  }

  imuInit(1);
  mWalk.setExit(walkingExit);
  walkingParamInit();
  stand();
  rosDebugInit();

  pthread_t thread_control;
  int ret = pthread_create(&thread_control, NULL, control_thread, NULL);
  if (ret != 0)
  {
    ROS_ERROR("create thread failed!");
    exit(EXIT_FAILURE);
  }

  int tty_set_flag;
  tty_set_flag = tty_set();
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    if (kbhit())
    {
      const int cmd = getchar();
      printf("%c\n", cmd);
      switch (cmd)
      {
      case 'q':
        ros::shutdown();
        break;
      case 's':
        std::cout << "step\n";
        mWalk.running = true;
        break;
      }
    }
    loop_rate.sleep();
  }
  if (tty_set_flag == 0)
    tty_reset();
  pthread_join(thread_control, NULL);
  std::cout << "walking_node exited normally.\n";
  res = system(". /home/lemon/robot_ros_application/catkin_ws/devel/setup.sh;rosservice call /MediumSize/BodyHub/StateJump 2 'reset'");
  return 0;
}