#ifndef _vrepBridge_h_
#define _vrepBridge_h_

#include "sensorDataStruct.h"
#include "motionInterface.h"

namespace vrepBridge
{
  class VrepMotion : public Motion::MotionInterface
  {
  public:
    bool init();
    bool exit();
    bool ping(uint16_t id);

    bool setJointPosition(std::vector<uint16_t> &ids, std::vector<Motion::JointParam_t> &data);
    bool setJointVelocity(std::vector<uint16_t> &ids, std::vector<Motion::JointParam_t> &data);
    bool setJointTorque(std::vector<uint16_t> &ids, std::vector<Motion::JointParam_t> &data);

    bool getJointData(std::vector<uint16_t> &ids, std::vector<Motion::JointParam_t> &data);
    bool getForce(std::vector<Motion::ForceParam_t> &data);
    bool getImu(std::vector<Motion::ImuParam_t> &data);
    bool getSensor(Motion::SensorParam &sensor);
  };
} // namespace vrepBridge

#endif