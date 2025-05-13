#ifndef _motionInterface_h_
#define _motionInterface_h_

#include "iostream"
#include "unistd.h"
#include "math.h"
#include "vector"
#include "sensorDataStruct.h"

namespace Motion
{
  class MotionInterface
  {
  public:
    virtual bool init() = 0;
    virtual bool exit() = 0;
    virtual bool ping(uint16_t id) = 0;

    virtual bool setJointPosition(std::vector<uint16_t> &ids, std::vector<JointParam_t> &position) = 0;
    virtual bool setJointVelocity(std::vector<uint16_t> &ids, std::vector<JointParam_t> &velocity) = 0;
    virtual bool setJointTorque(std::vector<uint16_t> &ids, std::vector<JointParam_t> &torque) = 0;

    virtual bool getJointData(std::vector<uint16_t> &ids, std::vector<JointParam_t> &data) = 0;
    virtual bool getForce(std::vector<ForceParam_t> &data) = 0;
    virtual bool getImu(std::vector<ImuParam_t> &data) = 0;
    virtual bool getSensor(SensorParam &data) = 0;
  };

  bool motionInterfaceSetup(std::string platform, MotionInterface *&interfacePtr);
} // namespace Motion

#endif
