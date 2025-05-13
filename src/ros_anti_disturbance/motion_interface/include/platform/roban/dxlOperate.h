#ifndef _dxlOperate_h_
#define _dxlOperate_h_

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <vector>
#include "sensorDataStruct.h"

#define SERVO_NUMBER_MAX 22
#define SERVO_NUM_READ 22
#define SERVO_NUM_WRITE 22

struct ServoInfo_s
{
  uint8_t number;
  uint8_t ids[SERVO_NUMBER_MAX];
};

class RobanServo
{
public:
  static bool init();
  static bool getPosition(std::vector<uint16_t> &ids, std::vector<double_t> &position);
  static bool setPosition(std::vector<uint16_t> &ids, std::vector<double_t> &position);
  static void convertRawData(int32_t *rawData, uint8_t jointNum, JointParam &data);

private:
  static bool initHandlers(void);
  static void setParam(uint8_t *ids, uint8_t number);
  static void convertIds(std::vector<uint16_t> &joint, uint8_t *motor);
};

class RobanFsr
{
public:
  static bool getData(std::vector<ForceParam_t> &data);
  static void convertRawData(int32_t *rawData, std::vector<ForceParam_t> &data);
};

class RobanImu
{
public:
  static bool getData(std::vector<ImuParam_t> &data);
  static void convertRawData(int32_t *rawData, std::vector<ImuParam_t> &data);
};

class DxlDevice
{
public:
  static bool init();
  static bool exit();
  static bool ping(uint16_t id);
};

class RobanSensor
{
public:
  static bool getData(SensorParam &sensor);
};

#endif
