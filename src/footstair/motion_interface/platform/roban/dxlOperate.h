#ifndef _dxlOperate_h_
#define _dxlOperate_h_

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <vector>
#include <mutex>
#include "sensorDataStruct.h"
#include "motionInterface.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

namespace Roban
{
#define DXL_DEVICE_NUMBER_MAX 22

  class DxlOperate : public Motion::MotionInterface
  {
  public:
    DxlOperate();
    ~DxlOperate();

    bool init();
    bool exit();
    bool ping(uint16_t id);

    bool setJointPosition(std::vector<uint16_t> &ids, std::vector<Motion::JointParam_t> &data);
    bool setJointVelocity(std::vector<uint16_t> &ids, std::vector<Motion::JointParam_t> &data);
    bool setJointTorque(std::vector<uint16_t> &ids, std::vector<Motion::JointParam_t> &data);

    bool getJointData(std::vector<uint16_t> &ids, std::vector<Motion::JointParam_t> &data);
    bool getForce(std::vector<Motion::ForceParam_t> &data);
    bool getImu(std::vector<Motion::ImuParam_t> &data);
    bool getSensor(Motion::SensorParam &data);

  private:
    bool loadCfg();
    bool dynamixelHandlersInit(uint8_t *ids);
    bool dxlBulkRead(uint8_t *ids, uint16_t number, uint16_t *addr, uint16_t *length, int32_t *rawData);
    bool initParam();
    void convertIds(std::vector<uint16_t> &vecIds, uint8_t *arrayIds);
    double_t convertValueTOAngle(uint8_t dxlID, int32_t motoValue);
    int32_t convertAngleTOValue(uint8_t dxlID, double_t motoAngle);
    void rawDataToImu(int32_t *rawData, uint8_t number, std::vector<Motion::ImuParam_t> &data);
    void rawDataToJoint(int32_t *rawData, uint8_t *idArray, uint8_t number, std::vector<Motion::JointParam_t> &data);
    void rawDataToForce(int32_t *rawData, uint8_t number, std::vector<Motion::ForceParam_t> &data);

  public:
    uint8_t deviceNumber;
    uint8_t deviceIds[DXL_DEVICE_NUMBER_MAX];

  protected:
    DynamixelWorkbench dxlWb;
    std::string _portName;
    uint32_t _baudrate;
    std::mutex mtxRW;
  };
} // namespace Roban

#endif