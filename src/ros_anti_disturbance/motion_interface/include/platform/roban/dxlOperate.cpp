#include "dxlOperate.h"
#include "ros/ros.h"
#include <mutex>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include <yaml-cpp/yaml.h>

#define TO_INT16(a, b) ((int16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))

#define SERVO_POSITION 0
#define SERVO__VELOCITY 1
#define SERVO_POSITION_ADDR 36
#define SERVO_POSITION_LEN 2

#define FSR_LEFT_ID 111
#define FSR_RIGHT_ID 112
#define FSR_ADDR 90
#define FSR_LEN 4

#define BASE_BOARD_ID 200
#define BASE_BOARD_ADDR 24
#define BASE_BOARD_LEN 56

#define IMU_ADDR 38
#define IMU_LEN (18 + 1)

#define GYRO_COEFFICIENT ((1000.0 * (M_PI / 180.0)) / 32768)
#define ACC_COEFFICIENT (8.0 * 9.8 / 32768)
#define MAG_COEFFICIENT (1.0)

#define FORCE_COEFFICIENT (0.2)

#define FSR_NUM 2
#define IMU_NUM 1

#define MediMotoAlpha 12.80
#define SmalMotoAlpha 18.61
#define UseSystemOffset 1

double_t assembleOffset[SERVO_NUMBER_MAX] = {0};
int8_t assembleDirection[SERVO_NUMBER_MAX] = {
  1,-1,-1,-1,1,1,
  1,-1,1,1,-1,1,
  -1,-1,-1,
  1,-1,-1,
  -1,-1,
  1,1,
};

static float AngleAlpha[SERVO_NUMBER_MAX] = {
    MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha,
    MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha,
    MediMotoAlpha, SmalMotoAlpha, SmalMotoAlpha,
    MediMotoAlpha, SmalMotoAlpha, SmalMotoAlpha,
    SmalMotoAlpha, SmalMotoAlpha,
    SmalMotoAlpha, SmalMotoAlpha};

static uint8_t motorIds[SERVO_NUMBER_MAX] = {
    1, 2, 3, 4, 5, 6,
    7, 8, 9, 10, 11, 12,
    16, 17, 18, 19,
    13, 14, 15, 20,
    21, 22};

static std::mutex mtxDxl;
static DynamixelWorkbench dxlWb;
static ServoInfo_s servoInfo;

double_t convertValueTOAngle(uint8_t dxlID, int32_t motoValue) // value_to_angle
{
  return ((motoValue - 2048) / AngleAlpha[dxlID]);
}

int32_t convertAngleTOValue(uint8_t dxlID, double_t motoAngle) // angle_to_value
{
  return (motoAngle * AngleAlpha[dxlID] + 2048);
}

void loadAssembleOffset()
{
  std::map<std::string, double> offsetMap;
  std::string stringID = "ID";
  std::string IDNameStr;
  YAML::Node offsetDoc;

  std::string yamlPath = __FILE__;
  if (UseSystemOffset == 1) {
    yamlPath = "/home/lemon/.lejuconfig/offset.yaml";
  } else {
    yamlPath.erase(yamlPath.rfind("/"));
    yamlPath = yamlPath + "/config/offset.yaml";
  }

  try
  {
    offsetDoc = YAML::LoadFile(yamlPath.c_str());
  }
  catch (const std::exception &e)
  {
    ROS_WARN("Fail to load offset yaml.");
    return;
  }

  YAML::Node itemData = offsetDoc["offset"];
  if (itemData.size() == 0)
    return;

  for (YAML::const_iterator itItemNum = itemData.begin(); itItemNum != itemData.end(); itItemNum++)
  {
    std::string IDName = itItemNum->first.as<std::string>();
    double offsetValue = itItemNum->second.as<double>();

    offsetMap[IDName] = offsetValue;
  }

  std::cout << "joint offset\n";
  for (uint8_t i = 0; i < SERVO_NUMBER_MAX; i++)
  {
    IDNameStr = stringID + std::to_string(i + 1);
    if (offsetMap.find(IDNameStr) == offsetMap.end())
      ROS_WARN("\nwithout find offset of %s ", IDNameStr.c_str());
    else
      assembleOffset[i] = offsetMap[IDNameStr];
      std::cout << assembleOffset[i] << "  ";
  }
  std::cout << "\n";
}

bool RobanServo::initHandlers(void)
{
  bool result = false;
  const char *log = NULL;

  result = dxlWb.addSyncWriteHandler(servoInfo.ids[0], "Goal_Position", &log); // 至少存在一个舵机
  if (result == false)
  {
    ROS_ERROR("addSyncWriteHandler pos fail, %s", log);
    return false;
  }

  result = dxlWb.addSyncWriteHandler(servoInfo.ids[0], "Moving_Speed", &log);
  if (result == false)
  {
    ROS_ERROR("addSyncWriteHandler spd fail, %s", log);
    return false;
  }

  result = dxlWb.initBulkRead(&log);
  if (result == false)
  {
    ROS_ERROR("initBulkRead fail, %s", log);
    return false;
  }

  result = dxlWb.initBulkWrite(&log);
  if (result == false)
  {
    ROS_ERROR("initBulkWrite fail, %s", log);
    return false;
  }

  return true;
}

void RobanServo::setParam(uint8_t *ids, uint8_t number)
{
  bool result = false;
  const char *log = NULL;
  int32_t pGain[SERVO_NUMBER_MAX] = {
      30, 30, 30, 30, 30, 30,
      30, 30, 30, 30, 30, 30,
      15, 50, 50,
      15, 50, 50,
      50, 50,
      50, 50};

  std::cout << "set P_gain\n";
  for (uint8_t i = 0; i < number; i++)
  {
    result = dxlWb.writeRegister(ids[i], "P_gain", pGain[ids[i] - 1], &log);
    if (result == false)
    {
      ROS_WARN("in %s function, writeRegister %d fail, %s", __func__, ids[i], log);
    }
  }
  std::cout << std::endl;
  std::cout << "set torque\n";
  for (uint8_t i = 0; i < number; i++)
  {
    result = dxlWb.torqueOn(ids[i], &log);
    if (result == false)
    {
      ROS_WARN("in %s function, torqueOn %d fail, %s", __func__, ids[i], log);
    }
  }
  std::cout << std::endl;
}

bool RobanServo::init(void)
{
  const char *log = NULL;
  bool result = false;

  dxlWb.scan(servoInfo.ids, &servoInfo.number, SERVO_NUMBER_MAX, &log);
  ROS_INFO("number of servo devices: %d", servoInfo.number);

  if (servoInfo.number > 0)
  {
    std::cout << "device id:" << std::endl;
    for (uint8_t i = 0; i < servoInfo.number; i++)
    {
      std::cout << (int)servoInfo.ids[i] << "  ";
    }
    std::cout << std::endl;

    result = initHandlers();
    if (result == false)
    {
      ROS_ERROR("initDxlHandlers failed!");
      return false;
    }
    setParam(servoInfo.ids, servoInfo.number);
  }
  else
  {
    ROS_ERROR("no dynamixel device found!");
    return false;
  }
  return true;
}

void RobanServo::convertIds(std::vector<uint16_t> &joint, uint8_t *motor)
{
  for (uint16_t i = 0; i < joint.size(); i++)
  {
    motor[i] = motorIds[joint[i] - 1];
  }
}

void RobanServo::convertRawData(int32_t *rawData, uint8_t jointNum, JointParam &data)
{
  uint16_t value = 0;
  for (uint8_t i = 0; i < jointNum; i++)
  {
    value = DXL_MAKEWORD(rawData[i * SERVO_POSITION_LEN], rawData[i * SERVO_POSITION_LEN + 1]);
    data.position[i] = convertValueTOAngle(i, value - assembleOffset[i]) * assembleDirection[i];
  }
}

bool RobanServo::getPosition(std::vector<uint16_t> &ids, std::vector<double_t> &position)
{
  uint16_t number = ids.size();
  for (uint8_t i = 0; i < number; i++)
  {
    if ((ids[i] < 1) || (ids[i] > SERVO_NUM_READ))
    {
      ROS_ERROR("in %s function, illegal parameter!", __func__);
      return false;
    }
  }

#if 1
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[number] = {0};
  uint16_t addrList[number] = {0};
  uint16_t lenghtList[number] = {0};
  int32_t valueList[number * SERVO_POSITION_LEN] = {0};

  RobanServo::convertIds(ids, idArray);
  dxlWb.clearBulkReadParam();
  for (uint8_t i = 0; i < number; i++)
  {
    addrList[i] = SERVO_POSITION_ADDR;
    lenghtList[i] = SERVO_POSITION_LEN;

    result = dxlWb.addBulkReadParam(idArray[i], addrList[i], lenghtList[i], &log);
    if (result == false)
    {
      ROS_ERROR("addBulkReadParam fail, %s", log);
      ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
      return false;
    }
  }
  result = dxlWb.bulkRead(&log);
  if (result == false)
  {
    ROS_ERROR("bulkRead fail, %s", log);
    ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
    return false;
  }
  result = dxlWb.getRawBulkReadData(idArray, number, addrList, lenghtList, valueList, &log);
  if (result == false)
  {
    ROS_ERROR("getRawBulkReadData fail, %s", log);
    ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
    return false;
  }

  uint16_t value = 0;
  for (uint8_t i = 0; i < number; i++)
  {
    value = DXL_MAKEWORD(valueList[i * SERVO_POSITION_LEN], valueList[i * SERVO_POSITION_LEN + 1]);
    position[i] = convertValueTOAngle(idArray[i] - 1, value - assembleOffset[idArray[i] - 1]) * assembleDirection[idArray[i] - 1];
  }

#else

  uint16_t number = ids.size();
  for (uint8_t i = 0; i < number; i++)
  {
    if ((ids[i] < 1) || (ids[i] > SERVO_NUM_READ))
    {
      ROS_ERROR("in %s function, illegal parameter!", __func__);
      return false;
    }
  }

  bool result = false;
  const char *log = NULL;
  int32_t rawData;

  std::vector<uint16_t> motorids(ids.size());
  for (uint16_t i = 0; i < ids.size(); i++)
  {
    motorids[i] = motorIds[joint[i] - 1];
  }
  for (uint8_t i = 0; i < number; i++)
  {
    mtxDxl.lock();
    result = dxlWb.readRegister(motorids[i], "Present_Position", &rawData, &log);
    mtxDxl.unlock();
    if (result == false)
    {
      ROS_ERROR("readRegister %d fail %s", motorids[i], log);
      ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
      return false;
    }
    position[i] = convertValueTOAngle(motorids[i] - 1, rawData - assembleOffset[motorids[i] - 1]) * assembleDirection[motorids[i] - 1];
  }
#endif

  return true;
}

bool RobanServo::setPosition(std::vector<uint16_t> &ids, std::vector<double_t> &position)
{
  uint16_t number = ids.size();
  for (uint8_t i = 0; i < number; i++)
  {
    if ((ids[i] < 1) || (ids[i] > SERVO_NUM_WRITE))
    {
      ROS_ERROR("in %s function, illegal parameter!", __func__);
      return false;
    }
  }

  bool result = false;
  const char *log = NULL;
  uint8_t idArray[number] = {0};
  int32_t valueList[number] = {0};

  RobanServo::convertIds(ids, idArray);
  for (uint8_t i = 0; i < number; i++)
  {
    valueList[i] = convertAngleTOValue(idArray[i] - 1, position[i] * assembleDirection[idArray[i] - 1]) + assembleOffset[idArray[i] - 1];
  }
  mtxDxl.lock();
  result = dxlWb.syncWrite(SERVO_POSITION, idArray, number, valueList, 1, &log); //同步写指令
  mtxDxl.unlock();
  if (result == false)
  {
    ROS_ERROR("syncWrite fail, %s", log);
    ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
    return false;
  }
  return true;
}

void RobanFsr::convertRawData(int32_t *rawData, std::vector<ForceParam_t> &data)
{
  data[0].force.x = rawData[1 * FSR_LEN] * FORCE_COEFFICIENT;
  data[0].force.y = rawData[1 * FSR_LEN + 1] * FORCE_COEFFICIENT;
  data[0].force.z = rawData[1 * FSR_LEN + 2] * FORCE_COEFFICIENT;
  data[0].moment.x = rawData[1 * FSR_LEN + 3] * FORCE_COEFFICIENT;

  data[1].force.x = rawData[0 * FSR_LEN] * FORCE_COEFFICIENT;
  data[1].force.y = rawData[0 * FSR_LEN + 1] * FORCE_COEFFICIENT;
  data[1].force.z = rawData[0 * FSR_LEN + 2] * FORCE_COEFFICIENT;
  data[1].moment.x = rawData[0 * FSR_LEN + 3] * FORCE_COEFFICIENT;
}

bool RobanFsr::getData(std::vector<ForceParam_t> &data)
{
  bool result = false;
  const char *log = NULL;
  uint8_t ids[FSR_NUM] = {0};
  uint16_t addrList[FSR_NUM] = {0};
  uint16_t lenghtList[FSR_NUM] = {0};
  int32_t valueList[FSR_NUM * FSR_LEN] = {0};

  mtxDxl.lock();
  dxlWb.clearBulkReadParam();
  for (uint8_t i = 0; i < FSR_NUM; i++)
  {
    ids[i] = FSR_LEFT_ID + i;
    addrList[i] = FSR_ADDR;
    lenghtList[i] = FSR_LEN;

    result = dxlWb.addBulkReadParam(ids[i], addrList[i], lenghtList[i], &log);
    if (result == false)
    {
      ROS_ERROR("addBulkReadParam fail, %s", log);
      ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
      mtxDxl.unlock();
      return false;
    }
  }
  result = dxlWb.bulkRead(&log);
  if (result == false)
  {
    ROS_ERROR("bulkRead fail, %s", log);
    ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
    mtxDxl.unlock();
    return false;
  }
  result = dxlWb.getRawBulkReadData(ids, FSR_NUM, addrList, lenghtList, valueList, &log);
  mtxDxl.unlock();
  if (result == false)
  {
    ROS_ERROR("getRawBulkReadData fail, %s", log);
    ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
    return false;
  }
  convertRawData(valueList, data);
  return true;
}

void RobanImu::convertRawData(int32_t *rawData, std::vector<ImuParam_t> &data)
{
  uint16_t index = 0;
  data[0].angularVel.x = GYRO_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
  data[0].angularVel.y = GYRO_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
  data[0].angularVel.z = GYRO_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
  data[0].linearAcc.x = ACC_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
  data[0].linearAcc.y = ACC_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
  data[0].linearAcc.z = ACC_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
  index += 1;
  data[0].magnetic.x = MAG_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
  data[0].magnetic.y = MAG_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
  data[0].magnetic.z = MAG_COEFFICIENT * TO_INT16(rawData[index++], rawData[index++]);
}

bool RobanImu::getData(std::vector<ImuParam_t> &data)
{
  bool result = false;
  const char *log = NULL;
  uint32_t rawData[IMU_LEN];
  mtxDxl.lock();
  result = dxlWb.readRegister(BASE_BOARD_ID, IMU_ADDR, IMU_LEN, rawData, &log);
  mtxDxl.unlock();
  if (result == false)
  {
    ROS_ERROR("readRegister fail, %s", log);
    ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
    return false;
  }
  convertRawData((int32_t *)rawData, data);
  return true;
}

bool DxlDevice::init()
{
  const char *portName = "/dev/usb_servo";
  int baudrate = 1000000;

  const char *log = NULL;
  bool result = false;

  result = dxlWb.init(portName, baudrate, &log); // initWorkbench
  if (result == false)
  {
    ROS_ERROR("open port failed, log: %s", log);
    return false;
  }
  else
    ROS_INFO("open port succeed, baudrate: %d", baudrate);

  if (RobanServo::init() == false)
  {
    return false;
  }
  loadAssembleOffset();
  return true;
}

bool DxlDevice::exit()
{
}

bool DxlDevice::ping(uint16_t id)
{
  bool result = false;
  const char *log = NULL;

  mtxDxl.lock();
  result = dxlWb.ping(id, &log);
  mtxDxl.unlock();
  if (result == false)
  {
    ROS_WARN("ping %d failed, %s", id, log);
    return false;
  }
  return true;
}

bool RobanSensor::getData(SensorParam &sensor)
{
  bool result = false;
  const char *log = NULL;
  uint8_t boardNum = 0;
  uint8_t servoNum = SERVO_NUM_READ;
  uint8_t fsrNum = FSR_NUM;
  uint8_t sensorNum = boardNum + servoNum + fsrNum;
  uint8_t id[sensorNum] = {0};
  uint16_t addr[sensorNum] = {0};
  uint16_t lenght[sensorNum] = {0};
  int32_t rawData[servoNum * SERVO_POSITION_LEN + fsrNum * FSR_LEN + boardNum * BASE_BOARD_LEN] = {0};

  uint8_t idx = 0;
  if (boardNum != 0)
  {
    id[idx] = BASE_BOARD_ID;
    addr[idx] = BASE_BOARD_ADDR;
    lenght[idx] = BASE_BOARD_LEN;
  }
  for (idx = boardNum; idx < boardNum + servoNum; idx++)
  {
    id[idx] = motorIds[idx - boardNum];
    addr[idx] = SERVO_POSITION_ADDR;
    lenght[idx] = SERVO_POSITION_LEN;
  }
  for (idx = boardNum + servoNum; idx < boardNum + servoNum + fsrNum; idx++)
  {
    id[idx] = FSR_LEFT_ID + (idx - boardNum + servoNum);
    addr[idx] = FSR_ADDR;
    lenght[idx] = FSR_LEN;
  }

  mtxDxl.lock();
  dxlWb.clearBulkReadParam();
  for (uint8_t i = 0; i < sensorNum; i++)
  {
    result = dxlWb.addBulkReadParam(id[i], addr[i], lenght[i], &log);
    if (result == false)
    {
      ROS_ERROR("addBulkReadParam fail, %s", log);
      ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
      mtxDxl.unlock();
      return false;
    }
  }
  result = dxlWb.bulkRead(&log);
  if (result == false)
  {
    ROS_ERROR("bulkRead fail, %s", log);
    ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
    mtxDxl.unlock();
    return false;
  }
  result = dxlWb.getRawBulkReadData(id, sensorNum, addr, lenght, rawData, &log);
  mtxDxl.unlock();
  if (result == false)
  {
    ROS_ERROR("getRawBulkReadData fail, %s", log);
    ROS_ERROR("position in %s function, line %d", __func__, __LINE__);
    return false;
  }
  if (boardNum != 0)
    RobanImu::convertRawData(&rawData[IMU_ADDR - BASE_BOARD_ADDR], sensor.imu);
  if (servoNum != 0)
    RobanServo::convertRawData(&rawData[boardNum * BASE_BOARD_LEN], servoNum, sensor.joint);
  if (fsrNum != 0)
    RobanFsr::convertRawData(&rawData[boardNum * BASE_BOARD_LEN + servoNum * SERVO_POSITION_LEN], sensor.force);
  return true;
}
