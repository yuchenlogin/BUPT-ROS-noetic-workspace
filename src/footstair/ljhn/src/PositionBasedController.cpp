#include <time.h>
#include <iostream>
#include <iomanip>
#include <limits.h>
#include "Util.h"

#include <Tasks/QPTasks.h>
#include <RBDyn/EulerIntegration.h>
#include <Tasks/Bounds.h>
#include <Tasks/QPConstr.h>
#include <Tasks/QPContactConstr.h>
#include <Tasks/QPMotionConstr.h>

#include "RobotDynamics.h"
#include "RobotDimensions.h"
#include "PositionBasedController.h"
#include "LIP.h"

#define LIMITING(v, min, max) ((v) > (max) ? (max) : ((v) < (min) ? (min) : (v)))

using namespace Util;
using namespace Human;
using namespace std;
using namespace rbd;
// using namespace tasks;

PositionBasedController *PositionBasedController::m_UniqueInstance = new PositionBasedController();

double lefthip = 0, righthip = 0;

PositionBasedController::PositionBasedController()
{
  timeStep = DcmPlanner.timeStep;
  lFootCtlValue = sva::PTransformd::Identity();
  rFootCtlValue = sva::PTransformd::Identity();
}

PositionBasedController::~PositionBasedController()
{
}

void PositionBasedController::ControllerInit(int targetStep)
{
  double steplength = DcmPlanner.stepLength;
  double stepWidth = DcmPlanner.stepWidth;
  OnlineFP.push_back(Vector2d(0., -stepWidth));
  OnlineFP.push_back(Vector2d(0., stepWidth));
  OnlineFP.push_back(Vector2d(steplength, -stepWidth));
  OnlineFP.push_back(Vector2d(2 * steplength, stepWidth));
  OnlineFP.push_back(Vector2d(2 * steplength, -stepWidth));
  OnlineFP.push_back(Vector2d(1.5 * steplength, stepWidth));
  OnlineFP.push_back(Vector2d(1. * steplength, -stepWidth));
  OnlineFP.push_back(Vector2d(0.5 * steplength, stepWidth));
  OnlineFP.push_back(Vector2d(0 * steplength, -stepWidth));
  OnlineFP.push_back(Vector2d(0 * steplength, stepWidth));

  DCMBasedPlanning::RobotState robot;
  robot.cp = Vector2d(0., 0.);
  robot.cpv = Vector2d(0., 0.);
  robot.com = Vector3d(0., 0., DcmPlanner.TorsoHeightWalk);
  robot.comv = Vector3d(0., 0., 0.);
  robot.coma = Vector3d(0., 0., 0.);
  robot.timecount = 0;
  robot.legs = DCMBasedPlanning::stand;
  DcmPlanner.computeCpTraj(OnlineFP, &robot);

  mbcSensor = rbd::MultiBodyConfig(LjhnDyn.FloatMb);
  mbcSensor.gravity << 0., 0., 9.81;

  rightFP.push_back(Vector3d(OnlineFP[0](0), OnlineFP[0](1), 0.));
  rightFP.push_back(Vector3d(OnlineFP[2](0), OnlineFP[2](1), 0.));
  leftFP.push_back(Vector3d(OnlineFP[1](0), OnlineFP[1](1), 0.));

  comDesir << 0., 0., DcmPlanner.TorsoHeightWalk;
  floatbaseJointCommand.block(0, 0, 6, 1) << 0, 0, 0, comDesir;
  comPos = sva::PTransformd(comDesir);
  lFootPos = sva::PTransformd(leftFP.front());
  rFootPos = sva::PTransformd(rightFP.front());
  lTouchPos = lFootPos;
  rTouchPos = rFootPos;

  lFootxbp = sva::PTransformd::Identity();
  rFootxbp = sva::PTransformd::Identity();
  //TODO: init ljdyn.FloatMbc
  updateTasksPara();
  updateJointAngleWithQP(comPos, lFootPos, rFootPos);
  updateJointAngleWithQP(comPos, lFootPos, rFootPos);

  LegS = rightStance;
  LegS_old = doublesupport;
  LegSref = rightStance;
  LegS_oldref = doublesupport;
  copDesir = Vector3d(0., -DcmPlanner.stepWidth, 0.);
  nextFP = Vector3d(0., DcmPlanner.stepWidth, 0.);
  stanceFP = Vector3d(0., -DcmPlanner.stepWidth, 0.);
  lastFP = Vector3d(0., DcmPlanner.stepWidth, 0.);

  timeCount = 0;
  timeCountDouble = 0;
  StepCount = 0;

  StopFlag = false;
  StartFlag = false;
  StepFlag = false;
  RequestStartCommand = false;

  torsofilterx = new Kalman(3, 3);
  double Qx[] = {7, 7, 7};
  double Rx[] = {1, 1, 1};
  torsofilterx->setQR(Qx, Rx);

  torsofiltery = new Kalman(3, 3);
  double Qy[] = {7, 7, 7};
  double Ry[] = {1, 1, 1};
  torsofiltery->setQR(Qy, Ry);

  torsofilter = new Kalman(15, 15);
  double torsoQ[] = {7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 1, 1, 1};
  double torsoR[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  torsofilter->setQR(torsoQ, torsoR);

  IMUQuaterniond = Eigen::Quaterniond(1., 0., 0., 0.);

  StepCountTarget = targetStep;

  ArmPos << 40, 25, -30, -15;
  ArmApt << -15, -55, 15, 55;

}

bool PositionBasedController::getFirstJointPosition(std::vector<double_t> &jointPosition)
{
  updateJointAngleWithQP(comPos, lFootPos, rFootPos);
  for (uint16_t i = 0; i < 12; i++)
  {
    jointPosition[i] = floatbaseJointCommand(i + 6) * Util::TO_DEGREE;
  }
  return true;
}

void PositionBasedController::Controller()
{
  static struct timeval start;
  static struct timeval end;
  gettimeofday(&start, NULL);

  torsoStabilizer();
  if (StartFlag == false)
  {
    updateJointAngleWithQP(comPos, lFootPos, rFootPos);
    return;
  }

  if (OneStepStair(0))
  {
    StepCount++;
    OneStepStair(1);
    StartFlag = false;
    exit(0);
  }

  gettimeofday(&end, NULL);
  double Controllertime = (end.tv_usec - start.tv_usec) / 1000.;
}

// double getTrapezoid(double t){
//  double a=0.2, b=0.2;
//  double S;
//  if(t<a)
//    S = 2.5*pow(t,2);
//  else if(t<1-b)
//    S = 0.1+1*(t-a);
//  else
//    S = 0.8-2.5*pow(1-t,2);
//  double tS = (S/0.8)<1?(S/0.8):1;
//  return tS;
// }

double getTrapezoid(double t)
{
  double a = 0.3, b = 0.4;
  double S;
  double Sm = 1 - a / 2 - b / 2;
  if (t < a)
    S = 1 / (2 * a) * pow(t, 2);
  else if (t < 1 - b)
    S = a / 2 + 1 * (t - a);
  else
    S = Sm - 1 / (2 * b) * pow(1 - t, 2);
  double tS = (S / Sm) < 1 ? (S / Sm) : 1;
  return tS;
}

double getTrapezoid3(double t)
{
  double at = 0.3, a1 = 0.4, bt = 0.4, b1 = 0.5;
  double S;
  double Sm = 1 - at * (1 - a1) / 2 - bt * (1 - b1) / 2;
  if (t < at)
    S = a1 * t + (1 - a1) / at * t * t / 2;
  else if (t < 1 - bt)
    S = at - at * (1 - a1) / 2 + 1 * (t - at);
  else
    S = Sm - ((1 - b1) / bt * (1 - t) + b1 + b1) / 2 * (1 - t);
  double tS = (S / Sm) < 1 ? (S / Sm) : 1;
  return tS;
}

double getTrapezoid4(double t)
{
  double at = 0.4, a1 = 0.2, bt = 0.4, b1 = 0.5;
  double S;
  double Sm = 1 - at * (1 - a1) / 2 - bt * (1 - b1) / 2;
  if (t < at)
    S = a1 * t + (1 - a1) / at * t * t / 2;
  else if (t < 1 - bt)
    S = at - at * (1 - a1) / 2 + 1 * (t - at);
  else
    S = Sm - ((1 - b1) / bt * (1 - t) + b1 + b1) / 2 * (1 - t);
  double tS = (S / Sm) < 1 ? (S / Sm) : 1;
  return tS;
}

void PositionBasedController::HipOffset(int legs, double t, double roffset = 6, double loffset = 6)
{

  if (t <= 0 || t >= 1)
    return;
  // cout<<"Entry in Hipoffset" <<endl;
  double rhythm = t;

  double RoffsetAngle = roffset / 57.3;
  double LoffsetAngle = loffset / 57.3;
  double a = 0.2; //1/6;
  double b = 0.7; //5/6;
  if (rhythm < a)
  {
    if (legs == 1) //左脚迈步 补偿右hip
    {
      righthip = -RoffsetAngle * rhythm / a;
    }
    if (legs == -1) //右脚迈步 补偿左hip
    {
      lefthip = LoffsetAngle * rhythm / a;
    }
  }
  if (rhythm > a && rhythm < b)
  {
    if (legs == 1)
    {
      righthip = -RoffsetAngle;
    }
    if (legs == -1)
    {
      lefthip = LoffsetAngle;
    }
  }
  if (rhythm >= b)
  {
    if (legs == 1)
    {
      righthip = -RoffsetAngle * (1 - rhythm) / (1 - b);
    }
    if (legs == -1)
    {
      lefthip = LoffsetAngle * (1 - rhythm) / (1 - b);
    }
  }
}

void PositionBasedController::ascendPhase(int count, int legs)
{
  Bline fBline, comBline;
  Vector3d compoint, swingpoint, stancepoint;

  const Bline::Real comkeys[] = {
      0.25 * step_x,
      legs * 0.055,
      com_h,
      0.40 * step_x,
      legs * 0.04,
      com_h + 0.1 * stairH,
      0.65 * step_x,
      -legs * 0.01,
      com_h + 0.25 * stairH,
      0.83 * step_x,
      -legs * 0.04,
      com_h + 0.55 * stairH,
      0.9 * step_x,
      -legs * 0.06,
      com_h + 0.75 * stairH,
      1.0 * step_x,
      -legs * 0.065,
      com_h + 0.8 * stairH,
  };
  comBline.build(comkeys, sizeof(comkeys) / (3 * sizeof(comkeys[0])));
  comBline.getPoint(getTrapezoid3(count * timeStep / swingT), comBline.point, comBline.tan);
  compoint = Vector3d(comBline.point.x, comBline.point.y, comBline.point.z);

  if (count * timeStep > 0.1 * swingT && count * timeStep < 0.5 * swingT)
  {
    double t = getTrapezoid((count * timeStep - 0.1 * swingT) / (0.4 * swingT));
    double torsopitch = sin(t * M_PI / 2) * TorsoPitch / 57.3;
    comPos.rotation() = sva::RotY(torsopitch);
  }

  swingpoint = Vector3d(0., legs * FootD, 0.);

  if (count * timeStep > 0.5 * swingT && count * timeStep < 1. * swingT)
  {
    double swingptich = getTrapezoid4((count * timeStep - 0.5 * swingT) / (0.5 * swingT)) * AnklePitch / 57.3;
    if (legs == 1)
    {
      lFootPos.rotation() = sva::RotY(swingptich);
    }
    else
    {
      rFootPos.rotation() = sva::RotY(swingptich);
    }
  }

  double stairoffset = 0;
  if (legs == 1)
  {
    stairoffset = 0.01;
  }
  if (legs == -1)
  {
    stairoffset = 0.01;
  }
  if (count * timeStep < 0.5 * swingT)
  {
    stancepoint = Vector3d(step_x, -legs * FootD, stairH + stairoffset * (1. - 1. * count * timeStep / (0.5 * swingT)));
  }
  else
  {
    stancepoint = Vector3d(step_x, -legs * FootD, stairH);
  }

  comPos.translation() = compoint;
  if (legs == 1)
  {
    lFootPos.translation() = swingpoint;
    rFootPos.translation() = stancepoint;
  }
  else
  {
    lFootPos.translation() = stancepoint;
    rFootPos.translation() = swingpoint;
  }
  lFootxbp.translation() = Vector3d(solelength / 2, 0., 0.);
  rFootxbp.translation() = Vector3d(solelength / 2, 0., 0.);
  updateJointAngleWithQP(comPos, lFootPos, rFootPos, lFootxbp, rFootxbp);

  if (legs == 1)
  {
    leftArm = ArmPos(0) * TO_RADIAN + ArmApt(0) * TO_RADIAN * getTrapezoid(1. * count * timeStep / swingT);
    rightArm = ArmPos(2) * TO_RADIAN + ArmApt(2) * TO_RADIAN * getTrapezoid(1. * count * timeStep / swingT);
  }
  else
  {
    leftArm = ArmPos(2) * TO_RADIAN + ArmApt(2) * TO_RADIAN * getTrapezoid(1. * count * timeStep / swingT);
    rightArm = ArmPos(0) * TO_RADIAN + ArmApt(0) * TO_RADIAN * getTrapezoid(1. * count * timeStep / swingT);
  }
}

void PositionBasedController::swingPhase(int count, int legs)
{
  Bline fBline, comBline;
  Vector3d compoint, swingpoint, stancepoint;

  const Bline::Real comkeys[] = {
      1.0 * step_x,
      -legs * 0.065,
      com_h + 0.80 * stairH,
      // 1.03*step_x, -legs*0.065,com_h+0.95*stairH,
      1.1 * step_x,
      -legs * 0.06,
      com_h + 1.1 * stairH,
      1.2 * step_x,
      -legs * 0.055,
      com_h + stairH,
      1.25 * step_x,
      -legs * 0.05,
      com_h + stairH,

  };
  comBline.build(comkeys, sizeof(comkeys) / (3 * sizeof(comkeys[0])));
  comBline.getPoint(count * timeStep / swingT, comBline.point, comBline.tan);
  compoint = Vector3d(comBline.point.x, comBline.point.y, comBline.point.z);

  if (count * timeStep > 0.3 * swingT && count * timeStep < 0.8 * swingT)
  {
    double t = getTrapezoid((count * timeStep - 0.3 * swingT) / (0.5 * swingT));
    double torsopitch = (1 - sin(t * M_PI / 2)) * TorsoPitch / 57.3;
    comPos.rotation() = sva::RotY(torsopitch);
  }

  double stairoffset = 0;
  if (legs == 1)
  {
    stairoffset = 0.01;
  }
  if (legs == -1)
  {
    stairoffset = 0.01;
  }
  const Bline::Real fkeys[] = {
      0.0,
      legs * FootD,
      0.,
      -0.2 * step_x,
      legs * FootD,
      stairH + 0.025,
      0.3 * step_x,
      legs * FootD,
      1.8 * stairH + 0.025,
      step_x,
      legs * FootD,
      2 * stairH + 0.035,
      2. * step_x,
      legs * FootD,
      2 * stairH + 0.015,
      2. * step_x,
      legs * FootD,
      2 * stairH + stairoffset,
  };
  fBline.build(fkeys, sizeof(fkeys) / (3 * sizeof(fkeys[0])));
  fBline.getPoint(getTrapezoid(count * timeStep / swingT), fBline.point, fBline.tan);
  swingpoint = Vector3d(fBline.point.x, fBline.point.y, fBline.point.z);

  if (count * timeStep > 0.3 * swingT && count * timeStep < 0.7 * swingT)
  {
    double swingptich = (1 - (count * timeStep - 0.3 * swingT) / (0.4 * swingT)) * AnklePitch / 57.3;
    if (legs == 1)
    {
      lFootPos.rotation() = sva::RotY(swingptich);
    }
    else
    {
      rFootPos.rotation() = sva::RotY(swingptich);
    }
  }

  stancepoint = Vector3d(step_x, -legs * FootD, stairH);

  comPos.translation() = compoint;
  if (legs == 1)
  {
    lFootPos.translation() = swingpoint;
    rFootPos.translation() = stancepoint;
  }
  else
  {
    lFootPos.translation() = stancepoint;
    rFootPos.translation() = swingpoint;
  }
  lFootxbp.translation() = Vector3d(solelength / 2, 0., 0.);
  rFootxbp.translation() = Vector3d(solelength / 2, 0., 0.);
  updateJointAngleWithQP(comPos, lFootPos, rFootPos, lFootxbp, rFootxbp);

  if (legs == 1)
  {
    leftArm = ArmPos(1) * TO_RADIAN + ArmApt(1) * TO_RADIAN * getTrapezoid(1. * count * timeStep / swingT);
    rightArm = ArmPos(3) * TO_RADIAN + ArmApt(3) * TO_RADIAN * getTrapezoid(1. * count * timeStep / swingT);
  }
  else
  {
    leftArm = ArmPos(3) * TO_RADIAN + ArmApt(3) * TO_RADIAN * getTrapezoid(1. * count * timeStep / swingT);
    rightArm = ArmPos(1) * TO_RADIAN + ArmApt(1) * TO_RADIAN * getTrapezoid(1. * count * timeStep / swingT);
  }

  HipOffset(legs, count * timeStep / swingT);
}

bool PositionBasedController::OneStepStair(int reset)
{

  static double phase = 0;
  static double count = 0;

  // updateSensor();
  // cout<<count<<"  "<<LegSref<<"  "<<leftFT(5)<<" | "<<rightFT(5)<<endl;
  // sva::PTransformd lFootFK = mbcSensor.bodyPosW[LjhnDyn.FloatMb.bodyIndexByName("leftLegLinkSole")];
  // sva::PTransformd rFootFK = mbcSensor.bodyPosW[LjhnDyn.FloatMb.bodyIndexByName("rightLegLinkSole")];
  // cout<<lFootFK.translation().transpose()<<" | "<<lFootPos.translation().transpose()<<endl;
  // cout<<rFootFK.translation().transpose()<<" | "<<rFootPos.translation().transpose()<<endl;
  // cout<<endl;

  if (reset == 1)
  {
    phase = 0;
    count = 0;
    return false;
  }

  com_h = DcmPlanner.TorsoHeightWalk;
  swingT = 1.;
  swingH = 0.08;
  stairH = 0.037;
  FootD = DcmPlanner.stepWidth;
  step_x = 0.15;
  solelength = 0.18;
  TorsoPitch = 10.;
  AnklePitch = 40;

  for (int i = 1; i <= 2 * StepCountTarget - 1; i++)
  {
    if (i % 2 == 1)
    {
      swingT = 1.3;
    }
    else
    {
      swingT = 1.4;
    }
  }
  if (phase == 2 * StepCountTarget - 1)
  {
    swingT = 2.;
  }

  if (phase == 0)
  {
    swingT = 2.;
    step_x = 0.153;
  }

  static int asendphasewaitflag = 0;
  if (count * timeStep < swingT)
  {
    if (asendphasewaitflag >= 0)
    {
      asendphasewaitflag++;
    }
    if (asendphasewaitflag >= 10)
    {
      asendphasewaitflag = -1;
    }
    if (asendphasewaitflag == -1)
    {
      count++;
    }
  }
  else
  {
    if (phase < 2 * StepCountTarget - 1)
    {
      phase++;
      asendphasewaitflag = 0;
      // StartFlag = false;
    }
    else
    {
      //stay here
      return true;
    }
    count = 0;
  }

  if (phase == 0)
  {
    //can not build straight line
    const Bline::Real comkeys[] = {
        0.0,
        0.,
        com_h,
        0.0,
        0.11,
        com_h,
        0.25 * step_x,
        0.05,
        com_h,
    };
    comBline.build(comkeys, sizeof(comkeys) / (3 * sizeof(comkeys[0])));
    comBline.getPoint(count * timeStep / swingT, comBline.point, comBline.tan);
    compoint = Vector3d(comBline.point.x, comBline.point.y, comBline.point.z);

    const Bline::Real rfkeys[] = {
        0.0,
        0.,
        -FootD,
        -0.2 * step_x,
        stairH + 0.025,
        -FootD,
        0.1 * step_x,
        stairH + 0.025,
        -FootD,
        step_x,
        stairH + 0.025,
        -FootD,
        step_x,
        stairH + 0.01,
        -FootD,
    };
    rfBline.build(rfkeys, sizeof(rfkeys) / (3 * sizeof(rfkeys[0])));
    if (count * timeStep < 0.4 * swingT)
    {
      rfpoint = Vector3d(0., -FootD, 0.);
    }
    else
    {
      rfBline.getPoint(getTrapezoid((count * timeStep - 0.4 * swingT) / (0.6 * swingT)), rfBline.point, rfBline.tan);
      rfpoint = Vector3d(rfBline.point.x, rfBline.point.z, rfBline.point.y);
      HipOffset(-1, (count * timeStep - 0.4 * swingT) / (0.6 * swingT), 6., 4.);
    }

    lfpoint = Vector3d(0., FootD, 0.);

    comPos.translation() = compoint;
    lFootPos.translation() = lfpoint;
    rFootPos.translation() = rfpoint;
    updateJointAngleWithQP(comPos, lFootPos, rFootPos);

    if (count * timeStep > 0.5 * swingT)
    {
      leftArm = ArmPos(0) * TO_RADIAN * getTrapezoid((count * timeStep - 0.5 * swingT) / (0.5 * swingT));
      rightArm = ArmPos(2) * TO_RADIAN * getTrapezoid((count * timeStep - 0.5 * swingT) / (0.5 * swingT));
    }
  }

  if (phase > 0 && phase < 2 * StepCountTarget - 2)
  {
    if ((int)phase % 2 == 1)
    {
      if ((int)(phase / 2) % 2 == 0)
      { //right stance ascend
        ascendPhase(count, 1);
      }
      else
      {
        ascendPhase(count, -1);
      }
    }
    else
    {
      if ((int)(phase / 2) % 2 == 0)
      { //left stance swing
        swingPhase(count, -1);
      }
      else
      {
        swingPhase(count, 1);
      }
    }
  }

  int legs;
  // stop step
  if (phase == 2 * StepCountTarget - 2)
  {
    if (StepCountTarget % 2 == 0)
    {
      legs = 1;
    }
    else
    {
      legs = -1;
    }

    Bline fBline, comBline;
    Vector3d compoint, swingpoint, stancepoint;
    const Bline::Real comkeys[] = {
        1. * step_x,
        -legs * 0.065,
        com_h + 0.8 * stairH,
        step_x,
        -legs * 0.065,
        com_h + stairH,
        step_x,
        -legs * 0.06,
        com_h + stairH,
    };
    comBline.build(comkeys, sizeof(comkeys) / (3 * sizeof(comkeys[0])));
    comBline.getPoint(count * timeStep / swingT, comBline.point, comBline.tan);
    compoint = Vector3d(comBline.point.x, comBline.point.y, comBline.point.z);

    if (count * timeStep > 0.5 * swingT && count * timeStep < 1. * swingT)
    {
      double t = getTrapezoid((count * timeStep - 0.5 * swingT) / (0.5 * swingT));
      double torsopitch = (1 - sin(t * M_PI / 2)) * 10 / 57.3;
      comPos.rotation() = sva::RotY(torsopitch);
    }

    const Bline::Real fkeys[] = {
        0.0,
        legs * FootD,
        0.,
        0.0,
        legs * FootD,
        stairH + 0.035,
        0.1 * step_x,
        legs * FootD,
        stairH + 0.035,
        step_x,
        legs * FootD,
        stairH + 0.025,
        step_x,
        legs * FootD,
        stairH + 0.01,
    };
    fBline.build(fkeys, sizeof(fkeys) / (3 * sizeof(fkeys[0])));
    fBline.getPoint(getTrapezoid(count * timeStep / swingT), fBline.point, fBline.tan);
    swingpoint = Vector3d(fBline.point.x, fBline.point.y, fBline.point.z);

    if (count * timeStep > 0.3 * swingT && count * timeStep < 0.8 * swingT)
    {
      double swingptich = (1 - (count * timeStep - 0.3 * swingT) / (0.5 * swingT)) * AnklePitch / 57.3;
      if (legs == 1)
      {
        lFootPos.rotation() = sva::RotY(swingptich);
      }
      else
      {
        rFootPos.rotation() = sva::RotY(swingptich);
      }
    }

    comPos.translation() = compoint;
    if (legs == 1)
    {
      lFootPos.translation() = swingpoint;
    }
    else
    {
      rFootPos.translation() = swingpoint;
    }

    updateJointAngleWithQP(comPos, lFootPos, rFootPos, lFootxbp, rFootxbp);

    if (legs == 1)
    {
      leftArm = ArmPos(1) * TO_RADIAN - ArmPos(1) * TO_RADIAN * getTrapezoid(1. * count * timeStep / swingT);
      rightArm = ArmPos(3) * TO_RADIAN - ArmPos(3) * TO_RADIAN * getTrapezoid(1. * count * timeStep / swingT);
    }
    else
    {
      rightArm = ArmPos(1) * TO_RADIAN - ArmPos(1) * TO_RADIAN * getTrapezoid(1. * count * timeStep / swingT);
      leftArm = ArmPos(3) * TO_RADIAN - ArmPos(3) * TO_RADIAN * getTrapezoid(1. * count * timeStep / swingT);
    }

    HipOffset(-1, count * timeStep / swingT);
  }

  if (phase == 2 * StepCountTarget - 1)
  {
    if (StepCountTarget % 2 == 0)
    {
      legs = 1;
    }
    else
    {
      legs = -1;
    }

    compoint = Vector3d(step_x, -legs * 0.06 * (1 - count * timeStep / swingT), com_h + stairH);
    swingpoint = Vector3d(step_x, legs * FootD, stairH + 0.01 * (1 - count * timeStep / swingT));
    if (legs == 1)
    {
      lFootPos.translation() = swingpoint;
    }
    else
    {
      rFootPos.translation() = swingpoint;
    }

    comPos.translation() = compoint;
    updateJointAngleWithQP(comPos, lFootPos, rFootPos, lFootxbp, rFootxbp);
  }

  return false;
}

//TODO: 考虑torso的z方向旋转角，不能置为零，需由正解得到，或置为控制量，或置为多数据融合后的量
void PositionBasedController::updateSensor()
{

  //copMeasure
  {
    if (leftFT(5) > 20)
      lcopm << -leftFT(1) / leftFT(5) - Foot_X, leftFT(0) / leftFT(5), 0.;
    else
      lcopm << 0., 0., 0.;
    if (rightFT(5) > 20)
      rcopm << -rightFT(1) / rightFT(5) - Foot_X, rightFT(0) / rightFT(5), 0.;
    else
      rcopm << 0., 0.0, 0.;

    Vector3d lleg = lFootPos.translation(); //leftFP.back();
    Vector3d rleg = rFootPos.translation(); //rightFP.back();

    //TODO: transform form sole frame to world frame
    lcopm = Matrix3d::Identity() * lcopm + lleg;
    rcopm = Matrix3d::Identity() * rcopm + rleg;

    if (LegS == leftStance)
      copMeasure = lcopm;
    else if (LegS == rightStance)
      copMeasure = rcopm;
    else
    {
      if (leftFT(5) + rightFT(5) < 50)
      {
        copMeasure = Vector3d(0., 0., 0.);
      }
      else
      {
        copMeasure = (lcopm * leftFT(5) + rcopm * rightFT(5)) / (leftFT(5) + rightFT(5));
      }
    }
  }

  //float_q
  {
    rbd::MultiBodyConfig FKSensor(LjhnDyn.mb);
    FKSensor.q = sVectorToParam(LjhnDyn.mb, float_q.segment<JOINT_NUM>(FLOATING_CONFIG_NUM));
    FKSensor.alpha = sVectorToParam(LjhnDyn.mb, float_dq.segment<JOINT_NUM>(FLOATING_FREEDOM_NUM));
    rbd::forwardKinematics(LjhnDyn.mb, FKSensor);

    sva::PTransformd lFootFK = FKSensor.bodyPosW[LjhnDyn.FloatMb.bodyIndexByName("leftLegLinkSole")];
    sva::PTransformd rFootFK = FKSensor.bodyPosW[LjhnDyn.FloatMb.bodyIndexByName("rightLegLinkSole")];
    //foot rot
    Matrix3d comrot = IMUQuaterniond.toRotationMatrix().transpose();
    // comrot = comPos.rotation()*comrot;

    Matrix3d lfootrot = lFootFK.rotation() * comrot;
    Matrix3d rfootrot = rFootFK.rotation() * comrot;

    //TODO: set double support phase foot trans
    sva::PTransformd lFoot(lfootrot, lFootPos.translation());
    sva::PTransformd rFoot(rfootrot, rFootPos.translation());

    sva::PTransformd comFK;
    if (LegSref == rightStance)
    {
      double t = 0.3 + 0.7 * timeCount / 3.;
      t = t > 1 ? 1 : (t < 0 ? 0 : t);
      comFK = sva::interpolate(rFootFK.inv() * rFoot, lFootFK.inv() * lFoot, t);
      // cout<<"in right "<<rFoot.translation().transpose()<<endl;
    }
    else if (LegSref == leftStance)
    {
      double t = 0.3 + 0.7 * timeCount / 3.;
      t = t > 1 ? 1 : (t < 0 ? 0 : t);
      comFK = sva::interpolate(lFootFK.inv() * lFoot, rFootFK.inv() * rFoot, t);
      // cout<<"in left "<<lFoot.translation().transpose()<<endl;
    }
    // else{
    //  double t = DSCount/((DcmPlanner.Tdsini+DcmPlanner.Tdsend)/timeStep);
    //  t = t>1 ? 1: (t<0?0:t);
    //  if(LegS_old == rightStance)
    //    comFK = sva::interpolate(lFootFK.inv()*lFoot,rFootFK.inv()*rFoot,t);
    //  else if(LegS_old == leftStance)
    //    comFK = sva::interpolate(rFootFK.inv()*rFoot,lFootFK.inv()*lFoot,t);
    // }

    float_q.segment<3>(4) = comFK.translation();
    if (StartFlag == true)
    {
      float_q.segment<4>(0) << IMUQuaterniond.w(), IMUQuaterniond.x(), IMUQuaterniond.y(), IMUQuaterniond.z();
    }
    else
    {
      float_q.segment<4>(0) << 1, 0, 0, 0;
    }

    lFootH = comFK.translation()(1);
  }

  //float_dq
  {
    float_dq.segment<JOINT_NUM>(FLOATING_FREEDOM_NUM) = (float_q.segment<12>(7) - float_qold.segment<12>(7)) / timeStep;

    //TODO add torso kalman filter
    float_dq.segment<3>(3) = (float_q.segment<3>(4) - float_qold.segment<3>(4)) / timeStep;
    float_dq.segment<3>(0) = imuGyro_InW;

    if (StartFlag == false)
    {
      for (int i = 0; i < 18; i++)
        float_dq(i) = 0;
    }
  }

  //cpComputed
  {
    torsofilter_measure << float_q.segment<3>(4), imuEuler,
        float_dq.segment<3>(3), float_dq.segment<3>(0),
        Vector3d(0, 0, Gravity);

    //is it needed to set zero?
    mbcSensor.zero(LjhnDyn.FloatMb);
    mbcSensor.q = sVectorToParam(LjhnDyn.FloatMb, float_q);
    rbd::forwardKinematics(LjhnDyn.FloatMb, mbcSensor);

    mbcSensor.alpha = sVectorToDof(LjhnDyn.FloatMb, float_dq);
    rbd::forwardVelocity(LjhnDyn.FloatMb, mbcSensor);
    float_qold = float_q;

    mbcSensor.alphaD = sVectorToDof(LjhnDyn.FloatMb, (float_dq - float_dqold) / timeStep);
    rbd::forwardAcceleration(LjhnDyn.FloatMb, mbcSensor);
    float_dqold = float_dq;

    //TODO: unknow coordinate error
    CoMcomputed = rbd::computeCoM(LjhnDyn.FloatMb, mbcSensor);
    auto CoMVcomputed_old = CoMVcomputed;
    CoMVcomputed = rbd::computeCoMVelocity(LjhnDyn.FloatMb, mbcSensor);
    if ((CoMVcomputed - CoMVcomputed_old).norm() > 0.3)
      CoMVcomputed = CoMVcomputed_old;
    //TODO use filtered state to get CoMa
    CoMacomputed = rbd::computeCoMAcceleration(LjhnDyn.FloatMb, mbcSensor);

    if (StartFlag == false)
      CoMacomputed = Vector3d(0., 0., 0.);

    MatrixXd F(3, 3);
    F << 1., timeStep, 0.,
        0., 1., timeStep,
        0., 0., 1.;
    Matrix3d H = Matrix3d::Identity();

    torsofilterx->update_F_H(F, H);
    torsofilter_xresult = torsofilterx->updateData(
        Vector3d(CoMcomputed(0), CoMVcomputed(0), CoMacomputed(0)));

    torsofiltery->update_F_H(F, H);
    torsofilter_yresult = torsofiltery->updateData(
        Vector3d(CoMcomputed(1), CoMVcomputed(1), CoMacomputed(1)));

    //TODO: unknow cp error limits
    Vector2d cpComputed_old = cpComputed;

    // if(StartFlag == false)
    cpComputed = CoMcomputed.segment(0, 2) + CoMVcomputed.segment(0, 2) / DcmPlanner.TimeCon_x;
    // else
    //  cpComputed= Vector2d(torsofilter_xresult(0),torsofilter_yresult(0))
    //   + Vector2d(torsofilter_xresult(1),torsofilter_yresult(1))/DcmPlanner.TimeCon_x;
  }

  //CentroidalMomentum
  {

    rbd::CentroidalMomentumMatrix centroidM(LjhnDyn.FloatMb);
    centroidM.computeMatrixAndMatrixDot(LjhnDyn.FloatMb, mbcSensor, CoMcomputed, CoMVcomputed);
    CMcomputed = centroidM.matrix() * float_dq;

    // comV = CMcomputed(0);
    comV = CMcomputed(1);
  }
}

void PositionBasedController::updateTasksPara()
{

  comweight << 10, 10, 10;
  lfweight << 10., 10., 10., 100., 100., 100.;
  rfweight << 10., 10., 10., 100., 100., 100.;

  //TODO: 速度约束无效？
  double inf = std::numeric_limits<double>::infinity();

  uBound = {{inf, inf, inf, inf, inf, inf, inf},
            {30. * TO_RADIAN},
            {25. * TO_RADIAN},
            {90. * TO_RADIAN},
            {-5. * TO_RADIAN},
            {60. * TO_RADIAN},
            {30. * TO_RADIAN},
            {},
            {30. * TO_RADIAN},
            {70. * TO_RADIAN},
            {90. * TO_RADIAN},
            {-5. * TO_RADIAN},
            {60. * TO_RADIAN},
            {30. * TO_RADIAN},
            {}};

  lBound = {{-inf, -inf, -inf, -inf, -inf, -inf, -inf},
            {-30. * TO_RADIAN},
            {-70. * TO_RADIAN},
            {-90. * TO_RADIAN},
            {-110. * TO_RADIAN},
            {-60. * TO_RADIAN},
            {-30. * TO_RADIAN},
            {},
            {-30. * TO_RADIAN},
            {-25. * TO_RADIAN},
            {-90. * TO_RADIAN},
            {-110. * TO_RADIAN},
            {-60. * TO_RADIAN},
            {-30. * TO_RADIAN},
            {}};

  uVel = {{inf, inf, inf, inf, inf, inf},
          {inf},
          {inf},
          {inf},
          {inf},
          {inf},
          {inf},
          {},
          {inf},
          {inf},
          {inf},
          {inf},
          {inf},
          {inf},
          {}};
  lVel = {{-inf, -inf, -inf, -inf, -inf, -inf},
          {-inf},
          {-inf},
          {-inf},
          {-inf},
          {-inf},
          {-inf},
          {},
          {-inf},
          {-inf},
          {-inf},
          {-inf},
          {-inf},
          {-inf},
          {}};
}

void PositionBasedController::updateJointAngleWithQP(sva::PTransformd com,
                                                     sva::PTransformd lf, sva::PTransformd rf,
                                                     sva::PTransformd lf_xbp, sva::PTransformd rf_xbp)
{
  lf.rotation() = lFootCtlValue.rotation() * lf.rotation();
  rf.rotation() = rFootCtlValue.rotation() * rf.rotation();

  rbd::MultiBody mb = LjhnDyn.FloatMb;
  rbd::MultiBodyConfig mbc = LjhnDyn.FloatMbc;

  std::vector<MultiBody> mbs = {mb};
  std::vector<MultiBodyConfig> mbcs = {mbc};

  struct timeval start;
  struct timeval end;
  gettimeofday(&start, NULL);

  tasks::qp::QPSolver solver;
  // com.translation()(0) -= 0.008;
  tasks::qp::CoMTask comTask(mbs, 0, com.translation());
  tasks::qp::OrientationTask oriTask(mbs, 0, "Torso", com.rotation());

  lf.translation() += lf_xbp.translation();
  rf.translation() += rf_xbp.translation();
  tasks::qp::TransformTask lposTask(mbs, 0, "leftLegLinkSole", lf, lf_xbp);
  tasks::qp::TransformTask rposTask(mbs, 0, "rightLegLinkSole", rf, rf_xbp);

  tasks::qp::PostureTask postureTask(mbs, 0, mbc.q, 0.5, 1.);

  tasks::qp::SetPointTask comTaskSp(mbs, 0, &comTask, 10., comweight, 100);
  tasks::qp::SetPointTask lposTaskSp(mbs, 0, &lposTask, 10., lfweight, 100);
  tasks::qp::SetPointTask rposTaskSp(mbs, 0, &rposTask, 10., rfweight, 100);
  tasks::qp::SetPointTask oriTaskSp(mbs, 0, &oriTask, 10., 100.);

  // tasks::qp::DamperJointLimitsConstr dampJointConstr(mbs, 0, {lBound, uBound},
  //  {lVel, uVel}, 0.125, 0.025, 1., 0.3);
  // dampJointConstr.addToSolver(solver);
  solver.nrVars(mbs, {}, {});
  solver.updateConstrSize();

  solver.addTask(&comTaskSp);
  solver.addTask(&lposTaskSp);
  solver.addTask(&rposTaskSp);
  solver.addTask(&postureTask);
  solver.addTask(&oriTaskSp);
  solver.updateTasksNrVars(mbs);

  int maxcount = 100;
  int count = 0;
  double alpha = 1;
  double tasknorm = 0;
  double step = 0.3;

  while (/*fabs(alpha)>0.01*/ count < 6)
  {
    count++;

    solver.solve(mbs, mbcs);
    rbd::eulerIntegration(mb, mbcs[0], step);
    rbd::forwardKinematics(mb, mbcs[0]);
    rbd::forwardVelocity(mb, mbcs[0]);

    VectorXd q = rbd::paramToVector(mbs[0], mbcs[0].q);
    VectorXd dq = rbd::dofToVector(mbs[0], mbcs[0].alpha);

    double tasknorm_old = tasknorm;
    tasknorm = comTask.eval().norm() + lposTask.eval().norm() + rposTask.eval().norm();
    alpha = tasknorm - tasknorm_old;
    // cout<<"com IK:"<<count<<" "<<tasknorm<<" "<<alpha<<endl;
  }
  VectorXd solved_q = rbd::paramToVector(mbs[0], mbcs[0].q);
  LjhnDyn.FloatMbc.q = sVectorToParam(LjhnDyn.FloatMb, solved_q);
  rbd::forwardKinematics(LjhnDyn.FloatMb, LjhnDyn.FloatMbc);

  comPosIK = sva::PTransformd(rbd::computeCoM(LjhnDyn.FloatMb, mbcs[0]));
  comPosIK.rotation() = (mbcs[0].bodyPosW[mb.bodyIndexByName("Torso")]).rotation();
  lFootPosIK = mbcs[0].bodyPosW[mb.bodyIndexByName("leftLegLinkSole")];
  rFootPosIK = mbcs[0].bodyPosW[mb.bodyIndexByName("rightLegLinkSole")];

  gettimeofday(&end, NULL);
  double time = 1000000 * (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec);
  // std::cout<<"time(ms):"<<time/1000<<std::endl;

  // cout<<"comrot: "<<sva::rotationVelocity(comPosIK.rotation()).transpose()*57.3<<endl;
  // cout<<"rfrot: "<<rfrot.transpose()*57.3<<endl;
  // cout<<endl;

  float_qref = solved_q;

  double normvalue = (floatbaseJointCommand.segment(6, 12) - solved_q.segment(7, 12)).norm();
  if (StartFlag == true && normvalue > 0.5)
  {
    cout << "Tasks IK failed!!!: " << normvalue << endl;
    cout << "com:" << comPos.translation().transpose()
         << " lf:" << lFootPos.translation().transpose()
         << " rf:" << rFootPos.translation().transpose()
         << endl;
    exit(0);
  }

  float_qref = solved_q;
  floatbaseJointCommand.segment(6, 12) = solved_q.segment(7, 12);

  floatbaseJointCommand[7] += lefthip;
  floatbaseJointCommand[13] += righthip;
}

void PositionBasedController::torsoStabilizer()
{
  if (!torsoStabilizerOn)
    return;

  double_t ctlRoll, ctlPitch;
  ctlRoll = imuGyro[0] * coeffStabilizer[0];
  ctlPitch = imuGyro[1] * coeffStabilizer[1];
  LIMITING(ctlRoll, radian(-5), radian(5));
  LIMITING(ctlPitch, radian(-5), radian(5));

  lFootCtlValue.rotation() = sva::RotY(ctlPitch) * sva::RotX(ctlRoll);
  rFootCtlValue.rotation() = sva::RotY(ctlPitch) * sva::RotX(ctlRoll);
}
