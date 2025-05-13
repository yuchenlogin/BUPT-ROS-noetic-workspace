#include "dxlOperate.h"
#include "ros/ros.h"
#include <yaml-cpp/yaml.h>

namespace Roban
{
#define TO_INT16(a, b) ((int16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))

#define TABLE_SPEED_TO_DPS(v) (0) // to do
#define TABLE_LOAD_TO_MA(v) (0)   // to do

#define DXL_SW_INDEX_POSITION 0

#define JOINT_DATA_TABLE_ADDR 36
#define JOINT_DATA_TABLE_LEN 6

#define FSR_LEFT_ID 112
#define FSR_RIGHT_ID 111
#define FSR_TABLE_ADDR 90
#define FSR_TABLE_LEN 4

#define IMU_ID 200
#define IMU_TABLE_ADDR 38
#define IMU_TABLE_LEN (18 + 1)

#define BASE_BOARD_ID 200
#define BASE_BOARD_ADDR 24
#define BASE_BOARD_LEN 56

#define FORCE_COEFFICIENT (0.2)
#define GYRO_COEFFICIENT ((1000.0 * (M_PI / 180.0)) / 32768)
#define ACC_COEFFICIENT (8.0 * 9.8 / 32768)
#define MAG_COEFFICIENT (1.0)

#define MediMotoAlpha 12.80
#define SmalMotoAlpha 18.61

  static float AngleAlpha[DXL_DEVICE_NUMBER_MAX] = {
      MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha,
      MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha,
      MediMotoAlpha, SmalMotoAlpha, SmalMotoAlpha,
      MediMotoAlpha, SmalMotoAlpha, SmalMotoAlpha,
      SmalMotoAlpha, SmalMotoAlpha,
      SmalMotoAlpha, SmalMotoAlpha};

  double_t assembleOffset[DXL_DEVICE_NUMBER_MAX] = {0};
  int8_t assembleDirection[DXL_DEVICE_NUMBER_MAX] = {
      1, -1, -1, -1, 1, 1,
      1, -1, 1, 1, -1, 1,
      -1, -1, -1,
      1, -1, -1,
      -1, -1,
      1, 1};

  static uint8_t motorIds[DXL_DEVICE_NUMBER_MAX] = {
      1, 2, 3, 4, 5, 6,
      7, 8, 9, 10, 11, 12,
      16, 17, 18, 19,
      13, 14, 15, 20,
      21, 22};

  DxlOperate::DxlOperate()
  {
    _portName = "/dev/usb_servo";
    _baudrate = 1000000;
  }

  DxlOperate::~DxlOperate()
  {
  }

  bool DxlOperate::loadCfg()
  {
    YAML::Node config;
    std::string filePath;
    filePath = __FILE__;
    filePath.erase(filePath.rfind("/"));
    filePath.erase(filePath.rfind("/"));
    filePath.erase(filePath.rfind("/"));
    filePath = filePath + "/config/config.yaml";
    try
    {
      config = YAML::LoadFile(filePath.c_str());
    }
    catch (const std::exception &e)
    {
      ROS_WARN("failed to load config yaml.");
      return false;
    }
    if (config["port"].size() == 0)
    {
      ROS_WARN("item port no content.");
      return false;
    }
    _portName = config["port"]["name"].as<std::string>();
    _baudrate = config["port"]["baudrate"].as<uint32_t>();
    std::cout << "portName: " << _portName << "\nbaudrate: " << _baudrate << "\n";

    filePath = __FILE__;
    filePath.erase(filePath.rfind("/"));
    filePath.erase(filePath.rfind("/"));
    filePath.erase(filePath.rfind("/"));
    filePath = filePath + "/config/offset.yaml";

    std::string sysOffsetPath = "/home/lemon/.lejuconfig/offset.yaml";
    if ((config["offset_path"].as<int>() == 1) && (access(sysOffsetPath.c_str(), F_OK) == 0))
    {
      filePath = sysOffsetPath;
    }
    std::cout << "offsetPath: " << filePath << "\n";
    
    try
    {
      config = YAML::LoadFile(filePath.c_str());
    }
    catch (const std::exception &e)
    {
      ROS_WARN("failed to load config yaml.");
      return false;
    }

    if (config["offset"].size() == 0)
    {
      ROS_WARN("item offset no content.");
    }
    else
    {
      std::map<std::string, double_t> offsetMap;
      std::string nameStr;
      for (YAML::const_iterator it = config["offset"].begin(); it != config["offset"].end(); it++)
      {
        offsetMap[it->first.as<std::string>()] = it->second.as<double_t>();
      }
      std::cout << "offset: ";
      for (uint8_t i = 0; i < DXL_DEVICE_NUMBER_MAX; i++)
      {
        nameStr = "ID" + std::to_string(i + 1);
        if (offsetMap.find(nameStr) == offsetMap.end())
          std::cout << "null  ";
        else
        {
          assembleOffset[i] = offsetMap[nameStr];
          std::cout << assembleOffset[i] << "  ";
        }
      }
      std::cout << "\n";
    }

    return true;
  }

  bool DxlOperate::dynamixelHandlersInit(uint8_t *ids)
  {
    bool result = false;
    const char *log = NULL;

    result = dxlWb.addSyncWriteHandler(ids[0], "Goal_Position", &log); // 至少存在一个舵机
    if (result == false)
    {
      ROS_ERROR("addSyncWriteHandler Goal_Position failed, %s", log);
      return false;
    }

    result = dxlWb.addSyncWriteHandler(ids[0], "Moving_Speed", &log);
    if (result == false)
    {
      ROS_ERROR("addSyncWriteHandler Moving_Speed failed, %s", log);
      return false;
    }

    result = dxlWb.initBulkRead(&log);
    if (result == false)
    {
      ROS_ERROR("initBulkRead failed, %s", log);
      return false;
    }

    return true;
  }

  bool DxlOperate::dxlBulkRead(uint8_t *ids, uint16_t number, uint16_t *addr, uint16_t *length, int32_t *rawData)
  {
    bool result = false;
    const char *log = NULL;
    mtxRW.lock();
    dxlWb.clearBulkReadParam();
    for (uint8_t i = 0; i < number; i++)
    {
      result = dxlWb.addBulkReadParam(ids[i], addr[i], length[i], &log);
      if (result == false)
      {
        mtxRW.unlock();
        ROS_ERROR("addBulkReadParam failed, %s", log);
        return false;
      }
    }
    result = dxlWb.bulkRead(&log);
    mtxRW.unlock();
    if (result == false)
    {
      ROS_ERROR("bulkRead failed, %s", log);
      return false;
    }
    mtxRW.lock();
    result = dxlWb.getRawBulkReadData(ids, number, addr, length, rawData, &log);
    mtxRW.unlock();
    if (result == false)
    {
      ROS_ERROR("getRawBulkReadData failed, %s", log);
      return false;
    }
    return true;
  }

  bool DxlOperate::initParam()
  {
    bool result = false;
    const char *log = NULL;
    int32_t pGain[DXL_DEVICE_NUMBER_MAX] = {
        30, 30, 30, 30, 30, 30,
        30, 30, 30, 30, 30, 30,
        15, 50, 50,
        15, 50, 50,
        50, 50,
        50, 50};

    int32_t temperatureMax[DXL_DEVICE_NUMBER_MAX] = {
        65, 65, 65, 65, 65, 65,
        65, 65, 65, 65, 65, 65,
        65, 65, 65, 65,
        65, 65, 65, 65,
        65, 65};

    std::cout << "set P_gain\n";
    for (uint8_t i = 0; i < deviceNumber; i++)
    {
      result = dxlWb.writeRegister(deviceIds[i], "P_gain", pGain[deviceIds[i] - 1], &log);
      if (result == false)
      {
        ROS_WARN("%d set P_gain failed, %s", deviceIds[i], log);
      }
    }

    std::cout << "set Temperature_Limit\n";
    for (uint8_t i = 0; i < deviceNumber; i++)
    {
      result = dxlWb.writeRegister(deviceIds[i], "Temperature_Limit", temperatureMax[deviceIds[i] - 1], &log);
      if (result == false)
      {
        ROS_WARN("%d set Temperature_Limit failed, %s", deviceIds[i], log);
      }
    }

    std::cout << "set torque\n";
    for (uint8_t i = 0; i < deviceNumber; i++)
    {
      result = dxlWb.torqueOn(deviceIds[i], &log);
      if (result == false)
      {
        ROS_WARN("%d set torqueOn failed, %s", deviceIds[i], log);
        return false;
      }
    }
    return true;
  }

  void DxlOperate::convertIds(std::vector<uint16_t> &vecIds, uint8_t *arrayIds)
  {
    for (uint16_t i = 0; i < vecIds.size(); i++)
    {
      arrayIds[i] = motorIds[vecIds[i] - 1];
    }
  }

  double_t DxlOperate::convertValueTOAngle(uint8_t dxlID, int32_t motoValue) // value_to_angle
  {
    return ((motoValue - 2048) / AngleAlpha[dxlID]);
  }

  int32_t DxlOperate::convertAngleTOValue(uint8_t dxlID, double_t motoAngle) // angle_to_value
  {
    return (motoAngle * AngleAlpha[dxlID] + 2048);
  }

  void DxlOperate::rawDataToImu(int32_t *rawData, uint8_t number, std::vector<Motion::ImuParam_t> &data)
  {
    for (uint8_t i = 0; i < number; i++)
    {
      data[i].angularVel.y = GYRO_COEFFICIENT * TO_INT16(rawData[i * IMU_TABLE_LEN + 1], rawData[i * IMU_TABLE_LEN + 2]);
      data[i].angularVel.x = GYRO_COEFFICIENT * TO_INT16(rawData[i * IMU_TABLE_LEN + 3], rawData[i * IMU_TABLE_LEN + 4]);
      data[i].angularVel.z = -GYRO_COEFFICIENT * TO_INT16(rawData[i * IMU_TABLE_LEN + 5], rawData[i * IMU_TABLE_LEN + 6]);
      data[i].linearAcc.y = ACC_COEFFICIENT * TO_INT16(rawData[i * IMU_TABLE_LEN + 7], rawData[i * IMU_TABLE_LEN + 8]);
      data[i].linearAcc.x = ACC_COEFFICIENT * TO_INT16(rawData[i * IMU_TABLE_LEN + 9], rawData[i * IMU_TABLE_LEN + 10]);
      data[i].linearAcc.z = -ACC_COEFFICIENT * TO_INT16(rawData[i * IMU_TABLE_LEN + 11], rawData[i * IMU_TABLE_LEN + 12]);
      data[i].magnetic.x = MAG_COEFFICIENT * TO_INT16(rawData[i * IMU_TABLE_LEN + 14], rawData[i * IMU_TABLE_LEN + 15]);
      data[i].magnetic.y = MAG_COEFFICIENT * TO_INT16(rawData[i * IMU_TABLE_LEN + 16], rawData[i * IMU_TABLE_LEN + 17]);
      data[i].magnetic.z = MAG_COEFFICIENT * TO_INT16(rawData[i * IMU_TABLE_LEN + 18], rawData[i * IMU_TABLE_LEN + 19]);
    }
  }

  void DxlOperate::rawDataToJoint(int32_t *rawData, uint8_t *idArray, uint8_t number, std::vector<Motion::JointParam_t> &data)
  {
    uint16_t value = 0;
    for (uint8_t i = 0; i < number; i++)
    {
      value = DXL_MAKEWORD(rawData[JOINT_DATA_TABLE_LEN * i + 0], rawData[JOINT_DATA_TABLE_LEN * i + 1]);
      data[i].position = convertValueTOAngle(idArray[i] - 1, value - assembleOffset[idArray[i] - 1]) * assembleDirection[idArray[i] - 1];
      data[i].velocity = TABLE_SPEED_TO_DPS((int16_t)((rawData[JOINT_DATA_TABLE_LEN * i + 2] & 0xff) | (uint16_t)(rawData[JOINT_DATA_TABLE_LEN * i + 3] << 8)));
      data[i].torque = TABLE_LOAD_TO_MA((int16_t)((rawData[JOINT_DATA_TABLE_LEN * i + 4] & 0xff) | (uint16_t)(rawData[JOINT_DATA_TABLE_LEN * i + 5] << 8)));
    }
  }

  void DxlOperate::rawDataToForce(int32_t *rawData, uint8_t number, std::vector<Motion::ForceParam_t> &data)
  {
    for (uint8_t i = 0; i < number; i++)
    {
      data[i].force.x = rawData[i * FSR_TABLE_LEN] * FORCE_COEFFICIENT;
      data[i].force.y = rawData[i * FSR_TABLE_LEN + 1] * FORCE_COEFFICIENT;
      data[i].force.z = rawData[i * FSR_TABLE_LEN + 2] * FORCE_COEFFICIENT;
      data[i].moment.x = rawData[i * FSR_TABLE_LEN + 3] * FORCE_COEFFICIENT;
    }
  }

  bool DxlOperate::init()
  {
    const char *log = NULL;
    bool result = false;
    result = loadCfg();
    if (result == false)
    {
      ROS_ERROR("loadCfg failed");
      return false;
    }
    result = dxlWb.init(_portName.c_str(), _baudrate, &log); // initWorkbench
    if (result == false)
    {
      ROS_ERROR("open port failed, log: %s", log);
      return false;
    }
    else
      ROS_INFO("open port succeed, baudrate: %d", _baudrate);

    ROS_INFO("scan dynamixel device...");
    dxlWb.scan(deviceIds, &deviceNumber, DXL_DEVICE_NUMBER_MAX, &log);
    if (deviceNumber > 0)
    {
      ROS_INFO("number of devices: %d", deviceNumber);
      std::cout << "device id:" << std::endl;
      for (uint8_t i = 0; i < deviceNumber; i++)
      {
        std::cout << (uint16_t)deviceIds[i] << "  ";
      }
      std::cout << std::endl;

      result = dynamixelHandlersInit(deviceIds);
      if (result == false)
      {
        ROS_ERROR("initDxlHandlers failed!");
        return false;
      }
    }
    else
    {
      ROS_ERROR("no dynamixel device found!");
      return false;
    }
    initParam();

    return true;
  }

  bool DxlOperate::exit()
  {
  }

  bool DxlOperate::ping(uint16_t id)
  {
    bool result = false;
    const char *log = NULL;

    mtxRW.lock();
    result = dxlWb.ping(id, &log);
    mtxRW.unlock();
    if (result == false)
    {
      ROS_WARN("ping %d failed, %s", id, log);
      return false;
    }
    return true;
  }

  bool DxlOperate::setJointPosition(std::vector<uint16_t> &ids, std::vector<Motion::JointParam_t> &data)
  {

    uint16_t number = ids.size();
    for (uint8_t i = 0; i < number; i++)
    {
      if ((ids[i] < 1) || (ids[i] > deviceNumber + 1))
      {
        ROS_ERROR("%s illegal parameter!", __func__);
        return false;
      }
    }

    bool result = false;
    const char *log = NULL;
    uint8_t idArray[number] = {0};
    int32_t wData[number] = {0};

    convertIds(ids, idArray);
    for (uint8_t i = 0; i < number; i++)
    {
      wData[i] = convertAngleTOValue(idArray[i] - 1, data[i].position * assembleDirection[idArray[i] - 1]) + assembleOffset[idArray[i] - 1];
    }
    mtxRW.lock();
    result = dxlWb.syncWrite(DXL_SW_INDEX_POSITION, idArray, number, wData, 1, &log); //同步写指令
    mtxRW.unlock();
    if (result == false)
    {
      ROS_ERROR("Failed to call service, syncWrite failed, %s ,in file %s line %d", log , __FILE__, __LINE__);
      return false;
    }
    return true;
  }

  bool DxlOperate::setJointVelocity(std::vector<uint16_t> &ids, std::vector<Motion::JointParam_t> &data)
  {
    return false;
  }

  bool DxlOperate::setJointTorque(std::vector<uint16_t> &ids, std::vector<Motion::JointParam_t> &data)
  {
    return false;
  }

  bool DxlOperate::getJointData(std::vector<uint16_t> &ids, std::vector<Motion::JointParam_t> &data)
  {
    uint8_t number = ids.size();
    for (uint8_t i = 0; i < number; i++)
    {
      if ((ids[i] < 1) || (ids[i] > deviceNumber + 1))
      {
        ROS_ERROR("%s illegal parameter!", __func__);
        return false;
      }
    }

    uint8_t idArray[number] = {0};
    uint16_t addr[number] = {0};
    uint16_t length[number] = {0};
    int32_t rData[JOINT_DATA_TABLE_LEN * number] = {0};
    bool result = false;
    convertIds(ids, idArray);
    for (uint8_t i = 0; i < number; i++)
    {
      addr[i] = JOINT_DATA_TABLE_ADDR;
      length[i] = JOINT_DATA_TABLE_LEN;
    }
    result = dxlBulkRead(idArray, number, addr, length, rData);
    if (result == false)
    {
      ROS_ERROR("getJointData fail in file %s line %d", __FILE__, __LINE__);
      return false;
    }
    rawDataToJoint(rData, idArray, number, data);
    return true;
  }

  bool DxlOperate::getForce(std::vector<Motion::ForceParam_t> &data)
  {
    uint8_t number = 2;
    uint8_t idArray[number] = {FSR_LEFT_ID, FSR_RIGHT_ID};
    uint16_t addr[number] = {FSR_TABLE_ADDR, FSR_TABLE_ADDR};
    uint16_t length[number] = {FSR_TABLE_LEN, FSR_TABLE_LEN};
    int32_t rData[FSR_TABLE_LEN * number] = {0};
    bool result = false;
    result = dxlBulkRead(idArray, number, addr, length, rData);
    if (result == false)
    {
      ROS_ERROR("in file %s, line %d", __FILE__, __LINE__);
      return false;
    }
    rawDataToForce(rData, number, data);
    return true;
  }

  bool DxlOperate::getImu(std::vector<Motion::ImuParam_t> &data)
  {
    uint8_t number = 1;
    uint8_t idArray[number] = {IMU_ID};
    uint16_t addr[number] = {IMU_TABLE_ADDR};
    uint16_t length[number] = {IMU_TABLE_LEN};
    int32_t rData[IMU_TABLE_LEN] = {0};
    bool result = false;
    result = dxlBulkRead(idArray, number, addr, length, rData);
    if (result == false)
    {
      ROS_ERROR("in file %s, line %d", __FILE__, __LINE__);
      return false;
    }
    rawDataToImu(rData, number, data);
    return true;
  }

  bool DxlOperate::getSensor(Motion::SensorParam &data)
  {
    bool result = false;
    const char *log = NULL;
    uint8_t imuNum = data.imu.size();
    uint8_t jointNum = data.joint.size();
    uint8_t fsrNum = data.force.size();
    uint8_t sensorNum = imuNum + jointNum + fsrNum;
    uint8_t idArray[sensorNum] = {0};
    uint16_t addr[sensorNum] = {0};
    uint16_t length[sensorNum] = {0};
    int32_t rawData[jointNum * JOINT_DATA_TABLE_LEN + fsrNum * FSR_TABLE_LEN + imuNum * IMU_TABLE_ADDR] = {0};
    uint8_t idx = 0;
    for (idx = 0; idx < imuNum; idx++)
    {
      idArray[idx] = IMU_ID;
      addr[idx] = IMU_TABLE_ADDR;
      length[idx] = IMU_TABLE_LEN;
    }
    for (idx = imuNum; idx < (imuNum + jointNum); idx++)
    {
      idArray[idx] = motorIds[idx - imuNum];
      addr[idx] = JOINT_DATA_TABLE_ADDR;
      length[idx] = JOINT_DATA_TABLE_LEN;
    }
    for (idx = (imuNum + jointNum); idx < (imuNum + jointNum + fsrNum); idx++)
    {
      idArray[idx] = FSR_LEFT_ID - (idx - imuNum + jointNum);
      addr[idx] = FSR_TABLE_ADDR;
      length[idx] = FSR_TABLE_LEN;
    }
    result = dxlBulkRead(idArray, sensorNum, addr, length, rawData);
    if (result == false)
    {
      ROS_ERROR("in file %s, line %d", __FILE__, __LINE__);
      return false;
    }
    if (imuNum != 0)
      rawDataToImu(&rawData[0], imuNum, data.imu);
    if (jointNum != 0)
      rawDataToJoint(&rawData[imuNum * IMU_TABLE_LEN], &idArray[imuNum], jointNum, data.joint);
    if (fsrNum != 0)
      rawDataToForce(&rawData[imuNum * IMU_TABLE_LEN + jointNum * JOINT_DATA_TABLE_LEN], fsrNum, data.force);
    return true;
  }
} // namespace Roban
