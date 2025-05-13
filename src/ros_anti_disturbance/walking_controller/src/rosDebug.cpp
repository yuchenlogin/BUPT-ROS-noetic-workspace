
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Float64MultiArray.h>
#include "walking_controller/walking.h"

extern Walking::Walking mWalk;

ros::Publisher jointValueRefPub;
ros::Publisher jointValueMeaPub;
ros::Publisher jointVelocityRefPub;
ros::Publisher jointVelocityMeaPub;

ros::Publisher stepPhasePub;
ros::Publisher torsoPosStatePub;
ros::Publisher torsoVelStatePub;
ros::Publisher comStatePub;
ros::Publisher comvStatePub;
ros::Publisher lfootStatePub;
ros::Publisher rfootStatePub;
ros::Publisher jointPosStatePub;

ros::Publisher torsoPosParamPub;
ros::Publisher torsoVelParamPub;
ros::Publisher comParamPub;
ros::Publisher comvParamPub;
ros::Publisher lfootParamPub;
ros::Publisher rfootParamPub;
ros::Publisher jointPosParamPub;

ros::Publisher dataVizPub;

static void statePublish()
{
  std_msgs::Float64 f64Msg;
  std_msgs::Float64MultiArray f64MultiMsg;

  f64Msg.data = mWalk.stepState.legS * 0.2;
  stepPhasePub.publish(f64Msg);

  // state
  f64MultiMsg.data.resize(6);
  for (uint8_t i = 0; i < 3; i++)
  {
    f64MultiMsg.data[i] = mWalk.botState.torso.pos.rotation().eulerAngles(2, 1 ,0)[i] * Util::TO_DEGREE;
    f64MultiMsg.data[i + 3] = mWalk.botState.torso.pos.translation()[i];
  }
  torsoPosStatePub.publish(f64MultiMsg);

  f64MultiMsg.data.resize(6);
  for (uint8_t i = 0; i < 3; i++)
  {
    f64MultiMsg.data[i] = mWalk.botState.torso.vel.angular()[i] * Util::TO_DEGREE;
    f64MultiMsg.data[i + 3] = mWalk.botState.torso.vel.linear()[i];
  }
  torsoVelStatePub.publish(f64MultiMsg);

  f64MultiMsg.data.resize(3);
  for (uint8_t i = 0; i < 3; i++)
  {
    f64MultiMsg.data[i] = mWalk.botState.CoM.pos.translation()[i];
  }
  comStatePub.publish(f64MultiMsg);

  f64MultiMsg.data.resize(3);
  for (uint8_t i = 0; i < 3; i++)
  {
    f64MultiMsg.data[i] = mWalk.botState.CoM.vel.linear()[i];
  }
  comvStatePub.publish(f64MultiMsg);

  f64MultiMsg.data.resize(6);
  for (uint8_t i = 0; i < 3; i++)
  {
    f64MultiMsg.data[i] = mWalk.botState.lFoot.pos.rotation().eulerAngles(2, 1 ,0)[i] * Util::TO_DEGREE;
    f64MultiMsg.data[i + 3] = mWalk.botState.lFoot.pos.translation()[i];
  }
  lfootStatePub.publish(f64MultiMsg);

  f64MultiMsg.data.resize(6);
  for (uint8_t i = 0; i < 3; i++)
  {
    f64MultiMsg.data[i] = mWalk.botState.rFoot.pos.rotation().eulerAngles(2, 1 ,0)[i] * Util::TO_DEGREE;
    f64MultiMsg.data[i + 3] = mWalk.botState.rFoot.pos.translation()[i];
  }
  rfootStatePub.publish(f64MultiMsg);

  f64MultiMsg.data.resize(JOINT_NUM);
  for (uint8_t i = 0; i < JOINT_NUM; i++)
  {
    f64MultiMsg.data[i] = mWalk.float_q[FLOATING_CONFIG_NUM + i];
  }
  jointPosStatePub.publish(f64MultiMsg);
}

static void paramPublish()
{
  std_msgs::Float64 f64Msg;
  std_msgs::Float64MultiArray f64MultiMsg;
  // param
  f64MultiMsg.data.resize(6);
  for (uint8_t i = 0; i < 3; i++)
  {
    f64MultiMsg.data[i] = mWalk.botParam.torso.pos.rotation().eulerAngles(2, 1 ,0)[i] * Util::TO_DEGREE;
    f64MultiMsg.data[i + 3] = mWalk.botParam.torso.pos.translation()[i];
  }
  torsoPosParamPub.publish(f64MultiMsg);

  f64MultiMsg.data.resize(6);
  for (uint8_t i = 0; i < 3; i++)
  {
    f64MultiMsg.data[i] = mWalk.botParam.torso.vel.angular()[i] * Util::TO_DEGREE;
    f64MultiMsg.data[i + 3] = mWalk.botParam.torso.vel.linear()[i];
  }
  torsoVelParamPub.publish(f64MultiMsg);

  f64MultiMsg.data.resize(3);
  for (uint8_t i = 0; i < 3; i++)
  {
    f64MultiMsg.data[i] = mWalk.botParam.CoM.pos.translation()[i];
  }
  comParamPub.publish(f64MultiMsg);

  f64MultiMsg.data.resize(3);
  for (uint8_t i = 0; i < 3; i++)
  {
    f64MultiMsg.data[i] = mWalk.botParam.CoM.vel.linear()[i];
  }
  comvParamPub.publish(f64MultiMsg);

  f64MultiMsg.data.resize(6);
  for (uint8_t i = 0; i < 3; i++)
  {
    f64MultiMsg.data[i] = mWalk.botParam.lFoot.pos.rotation().eulerAngles(2, 1 ,0)[i] * Util::TO_DEGREE;
    f64MultiMsg.data[i + 3] = mWalk.botParam.lFoot.pos.translation()[i];
  }
  lfootParamPub.publish(f64MultiMsg);

  f64MultiMsg.data.resize(6);
  for (uint8_t i = 0; i < 3; i++)
  {
    f64MultiMsg.data[i] = mWalk.botParam.rFoot.pos.rotation().eulerAngles(2, 1 ,0)[i] * Util::TO_DEGREE;
    f64MultiMsg.data[i + 3] = mWalk.botParam.rFoot.pos.translation()[i];
  }
  rfootParamPub.publish(f64MultiMsg);

  f64MultiMsg.data.resize(JOINT_NUM);
  for (uint8_t i = 0; i < JOINT_NUM; i++)
  {
    f64MultiMsg.data[i] = mWalk.float_q_ref[FLOATING_CONFIG_NUM + i];
  }
  jointPosParamPub.publish(f64MultiMsg);
}

void debugPublish()
{
  statePublish();
  paramPublish();

  std_msgs::Float64MultiArray f64MultiMsg;
  f64MultiMsg.data.resize(5);
  // for (uint8_t i = 0; i < 3; i++)
  // {
    f64MultiMsg.data[0] = mWalk.com_highpass[0];
    f64MultiMsg.data[1] = mWalk.com_lowpass[0];
  // }

  f64MultiMsg.data[2] = mWalk.thetafoot;
  f64MultiMsg.data[3] = mWalk.theta;
  f64MultiMsg.data[4] = mWalk.thetaVel;
  dataVizPub.publish(f64MultiMsg);

}

void rosDebugInit()
{
  ros::NodeHandle nh;
  
  stepPhasePub = nh.advertise<std_msgs::Float64>("/state/stepPhase", 1000);

  torsoPosStatePub = nh.advertise<std_msgs::Float64MultiArray>("/state/torso/pos", 1000);
  torsoVelStatePub = nh.advertise<std_msgs::Float64MultiArray>("/state/torso/vel", 1000);
  comStatePub = nh.advertise<std_msgs::Float64MultiArray>("/state/com/pos", 1000);
  comvStatePub = nh.advertise<std_msgs::Float64MultiArray>("/state/com/vel", 1000);
  lfootStatePub = nh.advertise<std_msgs::Float64MultiArray>("/state/lFoot/pos", 1000);
  rfootStatePub = nh.advertise<std_msgs::Float64MultiArray>("/state/rFoot/pos", 1000);
  jointPosStatePub = nh.advertise<std_msgs::Float64MultiArray>("/state/joint/pos", 1000);

  torsoPosParamPub = nh.advertise<std_msgs::Float64MultiArray>("/param/torso/pos", 1000);
  torsoVelParamPub = nh.advertise<std_msgs::Float64MultiArray>("/param/torso/vel", 1000);
  comParamPub = nh.advertise<std_msgs::Float64MultiArray>("/param/com/pos", 1000);
  comvParamPub = nh.advertise<std_msgs::Float64MultiArray>("/param/com/vel", 1000);
  lfootParamPub = nh.advertise<std_msgs::Float64MultiArray>("/param/lFoot/pos", 1000);
  rfootParamPub = nh.advertise<std_msgs::Float64MultiArray>("/param/rFoot/pos", 1000);
  jointPosParamPub = nh.advertise<std_msgs::Float64MultiArray>("/param/joint/pos", 1000);

  dataVizPub = nh.advertise<std_msgs::Float64MultiArray>("/dataViz", 1000);
}