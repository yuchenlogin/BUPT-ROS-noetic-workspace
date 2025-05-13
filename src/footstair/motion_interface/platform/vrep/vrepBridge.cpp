#include "vrepBridge.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>

namespace vrepBridge
{
#define SIM_JOINT_NUM 22
#define TO_DEGREE (180.0 / M_PI)
#define TO_RADIAN (M_PI / 180.0)

  ros::Subscriber simStepDone_sub;
  ros::Subscriber simState_sub;
  ros::Publisher simStart_pub;
  ros::Publisher simStop_pub;
  ros::Publisher simPause_pub;
  ros::Publisher simEnSync_pub;
  ros::Publisher simTrigNext_pub;

  ros::Subscriber jointState_sub;
  ros::Subscriber lFootFT_sub;
  ros::Subscriber rFootFT_sub;
  ros::Subscriber imu_sub;

  ros::Subscriber basePose_sub;
  ros::Subscriber com_sub;
  ros::Subscriber comv_sub;
  ros::Subscriber cop_sub;

  ros::Publisher jointPos_pub;
  ros::Publisher jointVel_pub;
  ros::Publisher jointTorque_pub;
  ros::Publisher cop_pub;

  bool stepDone; //一步仿真完成标志

  std::vector<Motion::JointParam_t> jointData(SIM_JOINT_NUM);
  Eigen::Matrix<double, 6, 1> lFootFT; // torque[x y z] force[x y z]
  Eigen::Matrix<double, 6, 1> rFootFT;
  Eigen::Vector3d imuGyro;
  Eigen::Vector3d imuAcc;
  Eigen::Vector3d imuEuler;

  bool mtxJointDataI;

  Eigen::Vector3d basePos; // translation[x y z]
  Eigen::Vector3d baseEuler;
  Eigen::Vector3d com;
  Eigen::Vector3d comv;
  Eigen::Vector3d cop;

  Eigen::Matrix<double_t, SIM_JOINT_NUM, 1> jointPosCmd;
  Eigen::Matrix<double_t, SIM_JOINT_NUM, 1> jointVelCmd;
  Eigen::Matrix<double_t, SIM_JOINT_NUM, 1> jointTorqueCmd;

  Eigen::Vector3d xyzEulerFromQuat(const Eigen::Quaterniond &q)
  {
    Eigen::Vector3d rpy;
    double w = q.w();
    double x = q.x();
    double y = q.y();
    double z = q.z();
    rpy(0) = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    rpy(1) = asin(2 * (w * y - z * x));
    rpy(2) = atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y));
    return rpy;
  }

  void simStateCallback(const std_msgs::Int32::ConstPtr &state)
  {
    // std_msgs::Bool start;
    // std::cout<<"here:" << std::endl;
    // if(state->data == 0)
    // {
    // 	start.data=1;
    // 	start_pub.publish(start);
    // }
  }

  void simStepDoneCallback(const std_msgs::Bool::ConstPtr &done)
  {
    stepDone = true;
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
  {
    uint16_t jointNum = msg->name.size();
    std::vector<uint16_t> ids(jointNum);
    for (uint16_t i = 0; i < jointNum; i++)
    {
      ids[i] = std::stoi(msg->name[i]);
      jointData[ids[i] - 1].position = msg->position[i] * TO_DEGREE;
      jointData[ids[i] - 1].velocity = msg->velocity[i] * TO_DEGREE;
      jointData[ids[i] - 1].torque = msg->effort[i];
    }
    mtxJointDataI = true;
  }

  void lFootForceCallback(const std_msgs::Float64MultiArray::ConstPtr &ft)
  {
    for (int i = 0; i < 6; ++i)
      lFootFT[i] = ft->data[i];
  }

  void rFootForceCallback(const std_msgs::Float64MultiArray::ConstPtr &ft)
  {
    for (int i = 0; i < 6; ++i)
      rFootFT[i] = ft->data[i];
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
  {
    imuGyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    imuAcc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    Eigen::Quaterniond quaternion(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    imuEuler = xyzEulerFromQuat(quaternion);
  }

  void basePoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
  {
    basePos << msg->position.x, msg->position.y, msg->position.z;
    Eigen::Quaterniond quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    baseEuler = xyzEulerFromQuat(quat);
  }

  void comCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    for (int i = 0; i < 3; ++i)
      com[i] = msg->data[i];
  }

  void comvCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    for (int i = 0; i < 3; ++i)
      comv[i] = msg->data[i];
  }

  void copCallback(const std_msgs::Float64MultiArray::ConstPtr &fcop)
  {
    for (int i = 0; i < 3; ++i)
      cop[i] = fcop->data[i];
  }

  void simTrigNext()
  {
    std_msgs::Bool msg;
    msg.data = 1;
    stepDone = false;
    simTrigNext_pub.publish(msg); //vrep仿真一步
    while (stepDone == false && ros::ok())
    {
      // ros::spinOnce();
      usleep(1);
    }
  }

  void simStart()
  {
    std_msgs::Bool msg;
    msg.data = 1;
    simEnSync_pub.publish(msg); //开启vrep同步模式
    simStart_pub.publish(msg);  //开始vrep仿真
    usleep(1000 * 200);

    for (uint16_t i = 0; i < 5; i++)
    {
      simTrigNext();
    }
  }

  bool simStop()
  {
    std_msgs::Bool msg;
    msg.data = 0;
    simEnSync_pub.publish(msg);
    msg.data = 1;
    simStop_pub.publish(msg); //停止vrep仿真
    usleep(1000 * 200);
    ROS_INFO("simulation in vrep stopped!");
    return true;
  }

  bool simInit()
  {
    ros::NodeHandle nh;
    simStepDone_sub = nh.subscribe<std_msgs::Bool>("simulationStepDone", 1, &simStepDoneCallback);
    simState_sub = nh.subscribe<std_msgs::Int32>("simulationState", 1, &simStateCallback);
    simStart_pub = nh.advertise<std_msgs::Bool>("startSimulation", 1);
    simStop_pub = nh.advertise<std_msgs::Bool>("stopSimulation", 1);
    simPause_pub = nh.advertise<std_msgs::Bool>("pauseSimulation", 1);
    simEnSync_pub = nh.advertise<std_msgs::Bool>("enableSyncMode", 1);
    simTrigNext_pub = nh.advertise<std_msgs::Bool>("triggerNextStep", 1);

    jointState_sub = nh.subscribe<sensor_msgs::JointState>("/sim/sensor/joint/state", 10, &jointStateCallback);
    lFootFT_sub = nh.subscribe<std_msgs::Float64MultiArray>("/sim/sensor/force/lFoot", 10, &lFootForceCallback);
    rFootFT_sub = nh.subscribe<std_msgs::Float64MultiArray>("/sim/sensor/force/rFoot", 10, &rFootForceCallback);
    imu_sub = nh.subscribe<sensor_msgs::Imu>("/sim/sensor/imu", 10, &imuCallback);

    basePose_sub = nh.subscribe<geometry_msgs::Pose>("/sim/state/base", 10, &basePoseCallback);
    com_sub = nh.subscribe<std_msgs::Float64MultiArray>("/sim/state/com", 10, &comCallback);
    comv_sub = nh.subscribe<std_msgs::Float64MultiArray>("/sim/state/comv", 10, &comvCallback);
    cop_sub = nh.subscribe<std_msgs::Float64MultiArray>("/sim/state/cop", 10, &copCallback);

    jointPos_pub = nh.advertise<std_msgs::Float64MultiArray>("/sim/param/joint/position", 1);
    jointVel_pub = nh.advertise<std_msgs::Float64MultiArray>("/sim/param/joint/velocity", 1);
    jointTorque_pub = nh.advertise<std_msgs::Float64MultiArray>("/sim/param/joint/torque", 1);
    cop_pub = nh.advertise<std_msgs::Float64MultiArray>("/sim/param/cop", 1);

    ROS_INFO("waiting for connect to vrep...");
    while (ros::ok() &&
           (simStart_pub.getNumSubscribers() <= 0 ||
            simEnSync_pub.getNumSubscribers() <= 0 ||
            simTrigNext_pub.getNumSubscribers() <= 0))
      ; //等待发布者与接收者建立连接
    ROS_INFO("vrep connected.");

    simStart();
    return true;
  }

  void sendJointPos(Eigen::VectorXd position)
  {
    std_msgs::Float64MultiArray msg;
    msg.data.resize(SIM_JOINT_NUM);
    for (int i = 0; i < SIM_JOINT_NUM; ++i)
    {
      msg.data[i] = position[i] * TO_RADIAN;
    }
    jointPos_pub.publish(msg);

    // while (jointPos_sub.getNumPublishers() <= 0)
    //   ;
    // while (lFootFT_sub.getNumPublishers() <= 0)
    //   ;
    // while (rFootFT_sub.getNumPublishers() <= 0)
    //   ;
    // while (imu_sub.getNumPublishers() <= 0)
    //   ;
    // while (jointPos_pub.getNumSubscribers() <= 0)
    //   ;
  }

  void sendJointVel(Eigen::VectorXd velocity)
  {
    std_msgs::Float64MultiArray msg;
    msg.data.resize(SIM_JOINT_NUM);
    for (int i = 0; i < SIM_JOINT_NUM; ++i)
    {
      msg.data[i] = velocity[i] * TO_RADIAN;
    }
    jointVel_pub.publish(msg);
  }

  void sendJointTorque(Eigen::VectorXd torque)
  {
    std_msgs::Float64MultiArray msg;
    msg.data.resize(SIM_JOINT_NUM);
    for (int i = 0; i < SIM_JOINT_NUM; ++i)
    {
      msg.data[i] = torque[i];
    }
    jointTorque_pub.publish(msg);
  }

  bool VrepMotion::init()
  {
    return simInit();
  }

  bool VrepMotion::exit()
  {
    return simStop();
  }

  bool VrepMotion::ping(uint16_t id)
  {
    return false;
  }

  bool VrepMotion::setJointPosition(std::vector<uint16_t> &ids, std::vector<Motion::JointParam_t> &data)
  {
    for (uint16_t i = 0; i < ids.size(); i++)
    {
      jointPosCmd[ids[i] - 1] = data[i].position;
    }
    sendJointPos(jointPosCmd);
    simTrigNext();
    return true;
  }

  bool VrepMotion::setJointVelocity(std::vector<uint16_t> &ids, std::vector<Motion::JointParam_t> &data)
  {
    for (uint16_t i = 0; i < ids.size(); i++)
    {
      jointVelCmd[ids[i] - 1] = data[i].velocity;
    }
    sendJointVel(jointVelCmd);
    simTrigNext();
    return true;
  }

  bool VrepMotion::setJointTorque(std::vector<uint16_t> &ids, std::vector<Motion::JointParam_t> &data)
  {
    for (uint16_t i = 0; i < ids.size(); i++)
    {
      jointTorqueCmd[ids[i] - 1] = data[i].torque;
    }
    sendJointTorque(jointTorqueCmd);
    simTrigNext();
    return true;
  }

  bool VrepMotion::getJointData(std::vector<uint16_t> &ids, std::vector<Motion::JointParam_t> &data)
  {
    while (!mtxJointDataI && ros::ok())
    {
      usleep(1);
    }
    if (!ros::ok())
      return false;
    for (uint16_t i = 0; i < ids.size(); i++)
    {
      data[i].position = jointData[ids[i] - 1].position;
      data[i].velocity = jointData[ids[i] - 1].velocity;
      data[i].torque = jointData[ids[i] - 1].torque;
    }
    mtxJointDataI = false;
    return true;
  }

  bool VrepMotion::getForce(std::vector<Motion::ForceParam_t> &data)
  {
    data[0].force = {lFootFT[0], lFootFT[1], lFootFT[2]};
    data[0].moment = {lFootFT[3], lFootFT[4], lFootFT[5]};
    data[1].force = {rFootFT[0], rFootFT[1], rFootFT[2]};
    data[1].moment = {rFootFT[3], rFootFT[4], rFootFT[5]};
    return true;
  }

  bool VrepMotion::getImu(std::vector<Motion::ImuParam_t> &data)
  {
    data[0].angularVel = {imuGyro[0], imuGyro[1], imuGyro[2]};
    data[0].linearAcc = {imuAcc[0], imuAcc[1], imuAcc[2]};
    data[0].eulerAngle = {imuEuler[0], imuEuler[1], imuEuler[2]};
    return true;
  }

  bool VrepMotion::getSensor(Motion::SensorParam &sensor)
  {
    sensor.joint = jointData;
    getForce(sensor.force);
    getImu(sensor.imu);
  }
} // namespace vrepBridge
