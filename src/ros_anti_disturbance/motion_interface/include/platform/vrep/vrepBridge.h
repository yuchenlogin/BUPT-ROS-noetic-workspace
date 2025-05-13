#ifndef _vrepBridge_h_
#define _vrepBridge_h_

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include "sensorDataStruct.h"

namespace vrepBridge
{
  bool simInit();
  void simStart();
  void simTrigNext();
  bool simStop();
  void simControll(Eigen::VectorXd jointV);

  bool simJointSetPosition(std::vector<uint16_t> &ids, std::vector<double_t> &goalPosition);
  bool simJointGetPosition(std::vector<uint16_t> &ids, std::vector<double_t> &currentPosition);
  bool simForceRead(std::vector<ForceParam_t> &data);
  bool simImuRead(std::vector<ImuParam_t> &data);
  bool simSensorRead(SensorParam &sensor);
} // namespace vrepBridge

#endif