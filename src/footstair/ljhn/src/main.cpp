#include <iostream>
#include <algorithm>
#include <signal.h>
#include <time.h>
#include <fstream>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Float64MultiArray.h>
#include "Util.h"
#include "motion_interface/motionInterface.h"
#include "imuData.h"
#include "include_input.h"
#include "PositionBasedController.h"

#define JOINT_NUM_MAX 22
#define JOINT_NUM_SENSOR 22
#define JOINT_NUM_CTL 22
#define FS_NUM 2
#define IMU_NUM 1

using namespace std;
using namespace Human;

void *replan_thread(void *ptr);
void *control_thread(void *ptr);
void advertise();
void debug_publish();

ros::Publisher cpref_pub;
ros::Publisher cpC_pub;
ros::Publisher cpD_pub;

ros::Publisher copm_pub;
ros::Publisher copD_pub;
ros::Publisher copS_pub;
ros::Publisher copref_pub;

ros::Publisher comaC_pub;
ros::Publisher comaD_pub;

ros::Publisher comC1_pub;
ros::Publisher comC2_pub;
ros::Publisher comvC1_pub;
ros::Publisher comvC2_pub;

ros::Publisher legS_pub;
ros::Publisher legSref_pub;

ros::Publisher lFT_pub;
ros::Publisher rFT_pub;

ros::Publisher joint_pub;
ros::Publisher jointref_pub;

ros::Publisher comvStatePub;

std::vector<uint16_t> jointIds{
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
    13, 14, 15, 16,
    17, 18, 19, 20,
    21, 22};
std::vector<JointParam_t> jointData(jointIds.size());
std::vector<JointParam_t> jointParam(jointIds.size());
pthread_t thread_control;

uint8_t imuType = 0;
double_t rawGyro[3];
double_t rawAcc[3];
ImuData imuDataObj;

Motion::MotionInterface *motion;
static std::string robotPlatform;
bool should_exiting = false;
PositionBasedController *positionController;

bool imuRead(double_t *gyro, double_t *acc)
{
  bool result = false;
  std::vector<ImuParam_t> imuData(IMU_NUM);
  result = motion->getImu(imuData);
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

void controllerParamInit()
{
  positionController->torsoStabilizerOn = true;
  positionController->coeffStabilizer << 0.04, 0.035, 0.0;
}

void sensorUpdate()
{
  bool result = false;
  std::vector<ImuParam_t> imuData(IMU_NUM);
  if (imuType == 0)
  {
    result = motion->getImu(imuData);
    if (result == true)
    {
      imuDataObj.setGyro(imuData[0].angularVel.x, imuData[0].angularVel.y, imuData[0].angularVel.z);
      imuDataObj.setAcc(imuData[0].linearAcc.x, imuData[0].linearAcc.y, imuData[0].linearAcc.z);
      imuDataObj.datafusion();
    }
  }
  imuData[0] = imuDataObj.getData();
  positionController->imuGyro << imuData[0].angularVel.x, imuData[0].angularVel.y, imuData[0].angularVel.z;
  positionController->imuAcc << imuData[0].linearAcc.x, imuData[0].linearAcc.y, imuData[0].linearAcc.z;
  positionController->imuEuler << imuData[0].eulerAngle.roll, imuData[0].eulerAngle.pitch, imuData[0].eulerAngle.yaw;
  positionController->imuGyro = positionController->imuGyro.eval() * Util::TO_RADIAN;
  positionController->imuAcc = positionController->imuGyro.eval() * Util::TO_RADIAN;
  positionController->imuEuler = positionController->imuGyro.eval() * Util::TO_RADIAN;
}

void *control_thread(void *ptr)
{
  static struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC, &next_time);

  // std::ofstream FTstream("FTstream.txt");
  while (ros::ok() && !should_exiting)
  {
    double timestep = positionController->timeStep;
    next_time.tv_sec += (next_time.tv_nsec + (int)(timestep * 1000000000)) / 1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + (int)(timestep * 1000000000)) % 1000000000;

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

    static struct timeval start;
    static struct timeval end;

    static struct timeval start1;
    static struct timeval end1;

    gettimeofday(&start1, NULL);
    sensorUpdate();
    positionController->Controller();
    for (int i = 0; i < 12; i++)
    {
      jointParam[i].position = positionController->floatbaseJointCommand(i + 6) * Util::TO_DEGREE;
    }
    auto it = std::find(jointIds.begin(), jointIds.end(), 13);
    if (it != jointIds.end())
      jointParam[std::distance(jointIds.begin(), it)].position = positionController->leftArm * Util::TO_DEGREE;
    it = std::find(jointIds.begin(), jointIds.end(), 17);
    if (it != jointIds.end())
      jointParam[std::distance(jointIds.begin(), it)].position = positionController->rightArm * Util::TO_DEGREE;
    // std::cout << "jointPostion:\n" << positionController->floatbaseJointCommand.segment(6, 12) * Util::TO_DEGREE << "\n";
    motion->setJointPosition(jointIds, jointParam);
    gettimeofday(&end1, NULL);
    // cout<<"used time ms:"<<(end1.tv_usec-start1.tv_usec)/1000.<<endl;
    gettimeofday(&end, NULL);
    // cout<<"All time ms:"<<(end.tv_usec-start.tv_usec)/1000.<<endl<<endl;
    gettimeofday(&start, NULL);
    debug_publish();
  }
}

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
  std::vector<JointParam_t> jointV(number);
  ros::Rate loopRate(1.0 / timeStep);
  for (uint16_t count = 1; count <= numberOfFrame; count++)
  {
    for (uint16_t i = 0; i < number; i++)
    {
      jointV[i].position = fifthSpline(numberOfFrame, currentPosition[ids[i] - 1], goalPosition[ids[i] - 1], count);
    }
    motion->setJointPosition(ids, jointV);
    loopRate.sleep();
    if (!ros::ok())
      break;
  }
}

void debug_publish()
{
  std_msgs::Float64 msg_val;
  std_msgs::Float64MultiArray f64MultiMsg;
  int XorY = 1;

  msg_val.data = positionController->LegS_old * 0.15;
  joint_pub.publish(msg_val);
  msg_val.data = positionController->LegS_oldref * 0.15; //imu pitch*57.29;//float_q(16)*57.29;
  jointref_pub.publish(msg_val);

  msg_val.data = 0.1 * positionController->LegS;
  legS_pub.publish(msg_val);
  msg_val.data = 0.1 * positionController->LegSref;
  legSref_pub.publish(msg_val);

  msg_val.data = positionController->leftFT(5) / 2000;
  lFT_pub.publish(msg_val);
  msg_val.data = positionController->rightFT(5) / 2000;
  rFT_pub.publish(msg_val);

  msg_val.data = positionController->copDesir(XorY);
  copD_pub.publish(msg_val);
  msg_val.data = positionController->copMeasure(XorY);
  copm_pub.publish(msg_val);
  msg_val.data = positionController->DcmPlanner.coprefvec[positionController->timeCount](XorY);
  copref_pub.publish(msg_val);

  msg_val.data = positionController->cpComputed(XorY);
  cpC_pub.publish(msg_val);
  msg_val.data = positionController->comV; //cpDesir(XorY);
  cpD_pub.publish(msg_val);
  msg_val.data = positionController->comH; //DcmPlanner.cprefvec[positionController->timeCount](XorY);
  cpref_pub.publish(msg_val);

  msg_val.data = positionController->CoMcomputed(XorY);
  comC1_pub.publish(msg_val);
  msg_val.data = positionController->comDesir(XorY);
  comC2_pub.publish(msg_val);

  msg_val.data = positionController->lFootH; //CoMacomputed(XorY)*0.2;
  comaC_pub.publish(msg_val);
  msg_val.data = positionController->rFootH; //comaDesir(XorY)*0.2;
  comaD_pub.publish(msg_val);

  msg_val.data = positionController->CoMVcomputed(XorY);
  comvC1_pub.publish(msg_val);
  msg_val.data = positionController->comvDesir(XorY);
  comvC2_pub.publish(msg_val);

  // f64MultiMsg.data.resize(3);
  // f64MultiMsg.data[0] = positionController->motionState.CoM.vel.linear()[0];
  // f64MultiMsg.data[1] = positionController->motionState.CoM.vel.linear()[1];
  // f64MultiMsg.data[2] = positionController->motionState.CoM.vel.linear()[2];
  // comvStatePub.publish(f64MultiMsg);
}

void advertise()
{
  ros::NodeHandle n;
  cpref_pub = n.advertise<std_msgs::Float64>("cpref", 1000);
  cpC_pub = n.advertise<std_msgs::Float64>("cpC", 1000);
  cpD_pub = n.advertise<std_msgs::Float64>("cpD", 1000);
  legS_pub = n.advertise<std_msgs::Float64>("legS", 1000);
  legSref_pub = n.advertise<std_msgs::Float64>("legSref", 1000);

  copm_pub = n.advertise<std_msgs::Float64>("copm", 1000);
  copD_pub = n.advertise<std_msgs::Float64>("copD", 1000);
  copS_pub = n.advertise<std_msgs::Float64>("copS", 1000);
  copref_pub = n.advertise<std_msgs::Float64>("copref", 1000);

  comaC_pub = n.advertise<std_msgs::Float64>("comaC", 1000);
  comaD_pub = n.advertise<std_msgs::Float64>("comaD", 1000);

  comC1_pub = n.advertise<std_msgs::Float64>("comC1", 1000);
  comC2_pub = n.advertise<std_msgs::Float64>("comC2", 1000);
  comvC1_pub = n.advertise<std_msgs::Float64>("comvC1", 1000);
  comvC2_pub = n.advertise<std_msgs::Float64>("comvC2", 1000);

  lFT_pub = n.advertise<std_msgs::Float64>("lFT", 1000);
  rFT_pub = n.advertise<std_msgs::Float64>("rFT", 1000);

  joint_pub = n.advertise<std_msgs::Float64>("joint", 1000);
  jointref_pub = n.advertise<std_msgs::Float64>("jointref", 1000);

  comvStatePub = n.advertise<std_msgs::Float64MultiArray>("/state/comv", 1000);
}

void stand()
{
  std::cout << "standing" << std::endl;
  std::vector<double_t> jointPos(22);
  std::vector<double_t> standPos{
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, -75, -15, 0,
      0, 75, 15, 0,
      0, 0};

  ros::Rate loopRate(1.0 / positionController->timeStep);
  while (!motion->getJointData(jointIds, jointData) && ros::ok())
  {
    loopRate.sleep();
  }

  uint8_t i = 0;
  for (auto it = jointData.begin(); it != jointData.end(); it++)
  {
    jointPos[jointIds[i++] - 1] = it->position;
  }

#if 1
  positionController->getFirstJointPosition(standPos);
  jointSetPositionWithSpeed(1.5, positionController->timeStep, jointIds, jointPos, standPos);
  std::cout << "stand sucess" << std::endl;
#else
  jointSetPositionWithSpeed(1.5, positionController->timeStep, jointIds, jointPos, standPos);
  std::cout << "stand sucess" << std::endl;
  exit(0);
#endif
  i = 0;
  for (auto it = jointParam.begin(); it != jointParam.end(); it++)
  {
    it->position = standPos[jointIds[i++] - 1];
  }
}

void sigintHandler(int sig)
{
  std::cout << "walk_node exit!" << std::endl;
  motion->exit();
  should_exiting=true;
  pthread_join(thread_control, NULL);
  int res = system("source /home/lemon/robot_ros_application/catkin_ws/devel/setup.sh;rosservice call /MediumSize/BodyHub/StateJump 2 'reset'");
  ros::shutdown();
  
  // kill(getpid(), SIGQUIT);
}

int main(int argc, char **argv)
{
  int res = system("source /home/lemon/robot_ros_application/catkin_ws/devel/setup.sh;rosservice call /MediumSize/BodyHub/StateJump 2 'external_control'");
  ros::init(argc, argv, "action_node");
  ros::NodeHandle nh("~");
  signal(SIGINT, sigintHandler);
  if (argc >= 3 && (strcmp(argv[2], "sim") == 0))
  {
    nh.setParam("robot_platform", "sim");
  }
  else if (argc >= 3 && (strcmp(argv[2], "roban") == 0))
  {
    nh.setParam("robot_platform", "roban");
  }
  nh.param<std::string>("robot_platform", robotPlatform, "roban");

  int targetStep = 5;
  if (argc >= 2)
  {
    targetStep = atoi(argv[1]) + 1;
    if (targetStep < 3)
      targetStep = 3;
    std::cout << "targetStep: " << targetStep << "\n";
  }

  ros::AsyncSpinner spinner(5);
  spinner.start();
  advertise();

  Motion::motionInterfaceSetup(robotPlatform, motion);
  if (motion->init() == false)
  {
    ROS_ERROR("error: motion init failed!");
    return 1;
  }

  positionController = PositionBasedController::GetInstance();
  positionController->ControllerInit(targetStep);
  controllerParamInit();

  imuInit(1);
  stand();
  // exit(0);
  int ret = pthread_create(&thread_control, NULL, control_thread, NULL);
  if (ret != 0)
  {
    ROS_ERROR("create thread failed!");
    exit(1);
  }

  initInput();

   usleep(1000 * 200);
   positionController->StartFlag = true;
   printf("start\n");

  while (ros::ok())
  {
    //block at readInput n msec
    char cmd = readInput(5);
    if (cmd == 'q')
    {
      motion->exit();
      break;
    }
    if (cmd == 'w')
    {
      positionController->StartFlag = true;
      printf("start\n");
    }
    if (cmd == 't')
    {
      positionController->StepFlag = true;
      printf("step\n");
    }
  }

  destoryInput();

  return 0;
}
