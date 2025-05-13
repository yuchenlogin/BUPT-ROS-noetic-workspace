#include "motionInterface.h"

#include "vrepBridge.h"
#include "dxlOperate.h"

motionPhy_t motionPhy;

bool motionInterfaceSetup(std::string platform)
{
  if (strcmp(platform.c_str(), "sim") == 0)
  {
    motionPhy.init = vrepBridge::simInit;
    motionPhy.exit = vrepBridge::simStop;
    motionPhy.ping = NULL;

    motionPhy.setJoint = NULL;
    motionPhy.setJointPosition = vrepBridge::simJointSetPosition;
    motionPhy.setJointVelocity = NULL;
    motionPhy.setJointTorque = NULL;

    motionPhy.getJoint = NULL;
    motionPhy.getJointPosition = vrepBridge::simJointGetPosition;
    motionPhy.getJointVelocity = NULL;
    motionPhy.getJointTorque = NULL;
    motionPhy.getForce = vrepBridge::simForceRead;
    motionPhy.getImu = vrepBridge::simImuRead;
    motionPhy.getSensor = vrepBridge::simSensorRead;
  }
  else if (strcmp(platform.c_str(), "roban") == 0)
  {
    motionPhy.init = &DxlDevice::init;
    motionPhy.exit = &DxlDevice::exit;
    motionPhy.ping = &DxlDevice::ping;

    motionPhy.setJoint = NULL;
    motionPhy.setJointPosition = &RobanServo::setPosition;
    motionPhy.setJointVelocity = NULL;
    motionPhy.setJointTorque = NULL;

    motionPhy.getJoint = NULL;
    motionPhy.getJointPosition = &RobanServo::getPosition;
    motionPhy.getJointVelocity = NULL;
    motionPhy.getJointTorque = NULL;
    motionPhy.getForce = &RobanFsr::getData;
    motionPhy.getImu = &RobanImu::getData;
    motionPhy.getSensor = &RobanSensor::getData;
  }
  return true;
}
