#ifndef _sensorDataStruct_h_
#define _sensorDataStruct_h_

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <vector>

typedef struct
{
  double_t x;
  double_t y;
  double_t z;
} Axis_t;

typedef struct
{
  double_t roll;
  double_t pitch;
  double_t yaw;
} EulerAngle_t;

typedef struct
{
  Axis_t force;
  Axis_t moment;
} ForceParam_t;

typedef struct
{
  Axis_t angularVel;
  Axis_t linearAcc;
  Axis_t magnetic;
  EulerAngle_t eulerAngle;
} ImuParam_t;

class JointParam
{
public:
  JointParam(uint16_t num = 12) : position(num),
                                  velocity(num),
                                  torque(num){};
  std::vector<double_t> position;
  std::vector<double_t> velocity;
  std::vector<double_t> torque;
};

class SensorParam
{
public:
  SensorParam(uint16_t jointNum = 12, uint16_t fsNum = 2, uint16_t imuNum = 1) : joint(jointNum),
                                                                                 force(fsNum),
                                                                                 imu(imuNum){};
  JointParam joint;
  std::vector<ForceParam_t> force;
  std::vector<ImuParam_t> imu;
};

#endif
