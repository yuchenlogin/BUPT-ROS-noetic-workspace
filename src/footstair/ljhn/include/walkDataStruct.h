#ifndef _walkDataStruct_h_
#define _walkDataStruct_h_

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

namespace GaitManager
{
  class Posture
  {
  public:
    double_t x;
    double_t y;
    double_t z;
    double_t roll;
    double_t pitch;
    double_t yaw;

  public:
    Posture();
    ~Posture();
    void zero();
  };

  //定义右手系，机器人前方为x轴正方形，左方为y轴正方向，上方为z轴正方向
  struct ParamOfPosture
  {
    Posture tosro;
    Posture lFoot;
    Posture rFoot;
    double LArm_R, LArm_P, LArm_elbow, RArm_R, RArm_P, RArm_elbow;
  };

  enum PhaseOfStep
  {
    DoubleSupport = 0,
    LeftStance = -1,
    RightStance = 1
  };
} // namespace GaitManager

#endif
