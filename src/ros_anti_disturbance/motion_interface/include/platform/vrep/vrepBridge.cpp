#include "vrepBridge.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>

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

  ros::Publisher jointCmd_pub;
  ros::Publisher footCop_pub;

  ros::Subscriber jointAngle_sub;
  ros::Subscriber jointVel_sub;
  ros::Subscriber leftFT_sub;
  ros::Subscriber rightFT_sub;

  ros::Subscriber basePos_sub;
  ros::Subscriber cop_sub;
  ros::Subscriber com_sub;
  ros::Subscriber comv_sub;
  ros::Subscriber imu_sub;

  bool stepDone; //一步仿真完成标志

  Eigen::VectorXd jointValueI;
  Eigen::VectorXd jointVelocityI;
  Eigen::VectorXd jointValueO;
  Eigen::VectorXd jointVelocityO;
  Eigen::Matrix<double, 6, 1> lFootForce; // torque[x y z] force[x y z]
  Eigen::Matrix<double, 6, 1> rFootForce;
  Eigen::Matrix<double, 7, 1> basePos; // quaternion[w x y z]  translation[x y z]
  Eigen::Vector3d cop;
  Eigen::Vector3d com;
  Eigen::Vector3d comv;
  Eigen::Vector3d torsoGyro;
  Eigen::Vector3d torsoAcc;
  Eigen::Vector3d torsoEuler;
  Eigen::Vector3d baseEuler;

  double_t timeStep = 0.02;

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

  void jointAngleCallback(const std_msgs::Float64MultiArray::ConstPtr &joint)
  {
    for (int i = 0; i < SIM_JOINT_NUM; ++i)
      jointValueI[i] = joint->data[i] * TO_DEGREE;
  }

  void jointVelCallback(const std_msgs::Float64MultiArray::ConstPtr &vel)
  {
    for (int i = 0; i < SIM_JOINT_NUM; ++i)
      jointVelocityI[i] = vel->data[i] * TO_DEGREE;
  }

  void lFootForceCallback(const std_msgs::Float64MultiArray::ConstPtr &ft)
  {
    for (int i = 0; i < 6; ++i)
      lFootForce[i] = ft->data[i];
  }

  void rFootForceCallback(const std_msgs::Float64MultiArray::ConstPtr &ft)
  {
    for (int i = 0; i < 6; ++i)
      rFootForce[i] = ft->data[i];
  }

  void basePosCallback(const std_msgs::Float64MultiArray::ConstPtr &pos)
  {
    for (int i = 0; i < 7; ++i)
      basePos[i] = pos->data[i];

    Eigen::Quaterniond Q(basePos[0], basePos[1], basePos[2], basePos[3]);
    baseEuler = Q.toRotationMatrix().eulerAngles(2, 1, 0);
    baseEuler(2) = baseEuler(2) > M_PI / 2 ? baseEuler(2) - M_PI : (baseEuler(2) < -M_PI / 2 ? baseEuler(2) + M_PI : baseEuler(2));
    baseEuler(1) = baseEuler(1) > M_PI / 2 ? M_PI - baseEuler(1) : (baseEuler(1) < -M_PI / 2 ? -M_PI - baseEuler(1) : baseEuler(1));
    // baseEuler(0) = 0.001 * (M_PI / 180.0); //set Yaw to 0.001 for stable reason
  }

  void copCallback(const std_msgs::Float64MultiArray::ConstPtr &fcop)
  {
    for (int i = 0; i < 3; ++i)
      cop[i] = fcop->data[i];
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

  void torsoImuCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    torsoGyro << msg->data[0], msg->data[1], msg->data[2];
    torsoAcc << msg->data[3], msg->data[4], msg->data[5];
    torsoEuler << msg->data[6], msg->data[7], msg->data[8];
  }

  bool simInit()
  {
    jointValueI.resize(SIM_JOINT_NUM);
    jointVelocityI.resize(SIM_JOINT_NUM);
    jointValueO.resize(SIM_JOINT_NUM);

    ros::NodeHandle node;
    simStepDone_sub = node.subscribe<std_msgs::Bool>("simulationStepDone", 1, &simStepDoneCallback);
    simState_sub = node.subscribe<std_msgs::Int32>("simulationState", 1, &simStateCallback);
    simStart_pub = node.advertise<std_msgs::Bool>("startSimulation", 1);
    simStop_pub = node.advertise<std_msgs::Bool>("stopSimulation", 1);
    simPause_pub = node.advertise<std_msgs::Bool>("pauseSimulation", 1);
    simEnSync_pub = node.advertise<std_msgs::Bool>("enableSyncMode", 1);
    simTrigNext_pub = node.advertise<std_msgs::Bool>("triggerNextStep", 1);

    jointAngle_sub = node.subscribe<std_msgs::Float64MultiArray>("/sim/joint/angle", 10, &jointAngleCallback);
    jointVel_sub = node.subscribe<std_msgs::Float64MultiArray>("/sim/joint/velocity", 10, &jointVelCallback);
    leftFT_sub = node.subscribe<std_msgs::Float64MultiArray>("/sim/force/leftFoot", 10, &lFootForceCallback);
    rightFT_sub = node.subscribe<std_msgs::Float64MultiArray>("/sim/force/rightFoot", 10, &rFootForceCallback);
    basePos_sub = node.subscribe<std_msgs::Float64MultiArray>("/sim/pose/torso", 10, &basePosCallback);
    cop_sub = node.subscribe<std_msgs::Float64MultiArray>("/sim/cop", 10, &copCallback);
    com_sub = node.subscribe<std_msgs::Float64MultiArray>("/sim/com", 10, &comCallback);
    comv_sub = node.subscribe<std_msgs::Float64MultiArray>("/sim/comv", 10, &comvCallback);
    imu_sub = node.subscribe<std_msgs::Float64MultiArray>("/sim/imu", 10, &torsoImuCallback);

    jointCmd_pub = node.advertise<std_msgs::Float64MultiArray>("/sim/joint/command", 1);
    footCop_pub = node.advertise<std_msgs::Float64MultiArray>("/sim/cop/cmd", 1);

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

  void simTrigNext()
  {
    std_msgs::Bool msg;
    msg.data = 1;
    stepDone = false;
    simTrigNext_pub.publish(msg); //vrep仿真一步  //大约要150ms才能收到simulationStepDone消息
    while (stepDone == false && ros::ok())
    {
      // ros::spinOnce();
      usleep(1);
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

  void simControll(Eigen::VectorXd jointV)
  {
    std_msgs::Float64MultiArray jointCmd;
    std_msgs::Float64MultiArray footCop;
    jointCmd.data.resize(SIM_JOINT_NUM);

    for (int i = 0; i < SIM_JOINT_NUM; ++i)
    {
      jointCmd.data[i] = jointV[i] * TO_RADIAN;
    }

    jointCmd_pub.publish(jointCmd);
    // footCop_pub.publish(footCop);

    // while (jointAngle_sub.getNumPublishers() <= 0)
    //   ;
    // while (leftFT_sub.getNumPublishers() <= 0)
    //   ;
    // while (rightFT_sub.getNumPublishers() <= 0)
    //   ;
    // while (basePos_sub.getNumPublishers() <= 0)
    //   ;
    // while (jointCmd_pub.getNumSubscribers() <= 0)
    //   ;
    // while (footCop_pub.getNumSubscribers() <= 0)
    //   ;
  }

  bool simJointSetPosition(std::vector<uint16_t> &ids, std::vector<double_t> &goalPosition)
  {
    for (uint16_t i = 0; i < ids.size(); i++)
    {
      jointValueO[ids[i] - 1] = goalPosition[i];
    }
    simControll(jointValueO);
    simTrigNext();
    return true;
  }

  bool simJointGetPosition(std::vector<uint16_t> &ids, std::vector<double_t> &currentPosition)
  {
    for (uint16_t i = 0; i < ids.size(); i++)
    {
      currentPosition[i] = jointValueI[ids[i] - 1];
    }
    return true;
  }

  bool simForceRead(std::vector<ForceParam_t> &data)
  {
    data[0].force = {lFootForce[0], lFootForce[1], lFootForce[2]};
    data[0].moment = {lFootForce[3], lFootForce[4], lFootForce[5]};
    data[1].force = {rFootForce[0], rFootForce[1], rFootForce[2]};
    data[1].moment = {rFootForce[3], rFootForce[4], rFootForce[5]};
    return true;
  }

  bool simImuRead(std::vector<ImuParam_t> &data)
  {
    data[0].angularVel = {torsoGyro[0], torsoGyro[1], torsoGyro[2]};
    data[0].linearAcc = {torsoAcc[0], torsoAcc[1], torsoAcc[2]};
    data[0].eulerAngle = {torsoEuler[0], torsoEuler[1], torsoEuler[2]};
    return true;
  }

  bool simSensorRead(SensorParam &sensor)
  {
    sensor.joint.position.assign(jointValueI.data(), jointValueI.data() + jointValueI.rows() + jointValueI.cols());
    simForceRead(sensor.force);
    simImuRead(sensor.imu);
  }
} // namespace vrepBridge
