#ifndef _motionInterface_h_
#define _motionInterface_h_

#include "iostream"
#include "unistd.h"
#include "math.h"
#include "vector"
#include "sensorDataStruct.h"

typedef struct
{
  bool (*init)();
  bool (*exit)();
  bool (*ping)(uint16_t id);

  bool (*setJoint)(std::vector<uint16_t> &ids, JointParam &data);
  bool (*setJointPosition)(std::vector<uint16_t> &ids, std::vector<double_t> &position);
  bool (*setJointVelocity)(std::vector<uint16_t> &ids, std::vector<double_t> &velocity);
  bool (*setJointTorque)(std::vector<uint16_t> &ids, std::vector<double_t> &torque);

  bool (*getJoint)(std::vector<uint16_t> &ids, JointParam &data);
  bool (*getJointPosition)(std::vector<uint16_t> &ids, std::vector<double_t> &position);
  bool (*getJointVelocity)(std::vector<uint16_t> &ids, std::vector<double_t> &velocity);
  bool (*getJointTorque)(std::vector<uint16_t> &ids, std::vector<double_t> &torque);
  bool (*getForce)(std::vector<ForceParam_t> &data);
  bool (*getImu)(std::vector<ImuParam_t> &data);
  bool (*getSensor)(SensorParam &data);
} motionPhy_t;

bool motionInterfaceSetup(std::string platform);

extern motionPhy_t motionPhy;
// motionPhy_t motionPhy;

#endif
