
#ifndef POSITION_BASED_CONTROLLER_H
#define POSITION_BASED_CONTROLLER_H

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>
#include "RobotDynamics.h"

#include "LIPMWalk.h"
#include "DCMBasedPlanning.h"
#include "Kalman.h"
#include "bl_line.h"

#define Gravity 9.7925
#define TotalWeight 430

namespace Human
{

  typedef boost::shared_mutex Lock;
  typedef boost::unique_lock<Lock> WriteLock;
  typedef boost::shared_lock<Lock> ReadLock;

  class PositionBasedController
  {

  public:
    PositionBasedController();
    virtual ~PositionBasedController();

    Lock myLocker;

    RobotDynamics LjhnDyn;

    void ControllerInit(int targetStep);
    void Controller();
    bool getFirstJointPosition(std::vector<double_t> &jointPosition);

    void PerformMotion();

    int timeCount;
    double timeCountDouble, timecount_v;
    int StepCount;
    bool ReplanFlag;
    bool ReplanFlagLR;
    bool ReplanFlagDS;
    bool StopFlag;
    bool StartFlag;
    bool StepFlag;
    bool RequestStartCommand;

    int StepCountTarget = 10;

    DCMBasedPlanning DcmPlanner;

    std::vector<GaitManager::LIPMWalk::WholeBodyMotion> WholeBodyTraj;
    bool isLeftlegTouched, isRightlegTouched;
    Eigen::Vector3d comDesir;
    Eigen::Vector3d comvDesir;
    Eigen::Vector3d comaDesir;
    Eigen::Vector3d copDesir;
    Eigen::Vector3d nextFP;
    Eigen::Vector3d stanceFP;
    Eigen::Vector3d lastFP;
    Eigen::Vector3d stanceCoM;

    Eigen::Vector3d copMeasure;
    Eigen::Vector2d cpDesir;
    Eigen::Vector2d cpMeasure;
    Eigen::Vector2d cperr, cperr_old;
    Eigen::Vector2d cperr_sum;
    Eigen::Vector2d cperr0;
    Eigen::Vector2d lastcperr0;

    Eigen::Vector2d cpComputed;
    Eigen::Vector2d cpvComputed;
    Eigen::Vector3d CoMacomputed;
    Eigen::Vector3d CoMcomputed;
    Eigen::Vector3d CoMVcomputed;
    Eigen::Vector6d CMcomputed;

    sva::PTransformd comPos;
    sva::PTransformd lFootPos;
    sva::PTransformd rFootPos;
    sva::PTransformd lFootxbp;
    sva::PTransformd rFootxbp;

    sva::PTransformd lFootCtlValue;
    sva::PTransformd rFootCtlValue;

    sva::PTransformd comPosIK;
    sva::PTransformd lFootPosIK;
    sva::PTransformd rFootPosIK;

    Eigen::Vector3d lcopm;
    Eigen::Vector3d rcopm;

    //sensor data
    Eigen::Quaterniond IMUQuaterniond;
    Eigen::Vector3d imuGyro, imuAcc;
    Eigen::Vector3d imuGyro_InW, imuAcc_InW, imuEuler;
    Eigen::Matrix<double, 6, 1> leftFT;
    Eigen::Matrix<double, 6, 1> rightFT;

    bool OneStepStair(int rest);
    double leftArm, rightArm;
    Eigen::Matrix<double, 4, 1> ArmPos;
    Eigen::Matrix<double, 4, 1> ArmApt;

    double com_h, swingT, swingH, stairH, FootD, step_x, solelength;
    double TorsoPitch, AnklePitch;
    Bline lfBline, rfBline, fBline, comBline;
    Vector3d lfpoint, rfpoint, compoint, swingpoint, stancepoint;

    void ascendPhase(int count, int legs);
    void swingPhase(int count, int legs);

    void stopPhase1(int count, int legs);
    void stopPhase2(int count, int legs);

    void HipOffset(int legs, double t, double roffset, double loffset);

    void updateFootPrint();

    void updateNextStep(double remaintime, double command_x,
                        Vector2d currentcop, Vector2d currentcp);

    void updateThreeStep();

    void cpStayPositon(Vector2d cp);

    Eigen::Vector2d StandPosition;
    std::vector<Eigen::Vector2d> OnlineFP;
    std::vector<Eigen::Vector3d> leftFP;
    std::vector<Eigen::Vector3d> rightFP;

    std::vector<double> OnlineFProt;
    std::vector<double> leftFProt;
    std::vector<double> rightFProt;

    void SwingLegContorller();

    void updateFootPos();

    void updateImpendance();

    void updateLegState();
    enum LegState
    {
      rightStance = -1,
      doublesupport = 0,

      leftStance = 1,

      leftTouching,
      rightTouching,

      leftLifting,
      rightLifting
    };
    LegState LegS, LegS_old, LegSref, LegS_oldref;

    int LSref, RSref, DSref;
    int LSCount, RSCount, DSCount;
    sva::PTransformd lTouchPosref, rTouchPosref;
    sva::PTransformd lTouchPos, rTouchPos;

    void updateSensor();
    rbd::MultiBodyConfig mbcSensor;
    rbd::MultiBodyConfig leftLegMbc;
    rbd::MultiBodyConfig rightLegMbc;

    Kalman *torsofilterx;
    Kalman *torsofiltery;
    Eigen::Matrix<double, 3, 1> torsofilter_xresult;
    Eigen::Matrix<double, 3, 1> torsofilter_yresult;

    Kalman *torsofilter;
    Eigen::Matrix<double, 15, 1> torsofilter_result;
    Eigen::Matrix<double, 15, 1> torsofilter_measure;

    Kalman *CoMfilter;

    //test variable
    double soleRollv, solePitchv;
    double lFootH, rFootH, comH;
    double lFootV, rFootV, comV;

    void updateTasksPara();
    Eigen::Vector6d lfweight, rfweight;
    Eigen::Vector3d comweight;
    std::vector<std::vector<double>> uBound, lBound, uVel, lVel;

    void updateJointAngleWithQP(sva::PTransformd com,
                                sva::PTransformd lf, sva::PTransformd rf,
                                sva::PTransformd lf_xbp = sva::PTransformd::Identity(),
                                sva::PTransformd rf_xbp = sva::PTransformd::Identity());

    void torsoStabilizer();
    bool torsoStabilizerOn;
    Eigen::Vector3d coeffStabilizer;

    static PositionBasedController *GetInstance()
    {
      return m_UniqueInstance;
    }

    enum jointId
    {
      LLEG_JOINT_START = 0,
      LLEG_JOINT_END = 6,
      LLEG_JOINT_NUM = 6,

      RLEG_JOINT_START = 6,
      RLEG_JOINT_END = 12,
      RLEG_JOINT_NUM = 6,

      LARM_JOINT_START = 12,
      LARM_JOINT_END = 15,
      LARM_JOINT_NUM = 3,

      RARM_JOINT_START = 15,
      RARM_JOINT_END = 18,
      RARM_JOINT_NUM = 3,

      JOINT_NUM = 12,

      FLOATING_FREEDOM_NUM = 6,
      FLOATING_CONFIG_NUM = 7,
      JOINT_FREEDOM_NUM = 18,
      JOINT_CONFIG_NUM = 19,

      LLEG_JOINT_FREEDOM_NUM = 12,
      LLEG_JOINT_CONFIG_NUM = 13,

    };

    double timeStep;

    Eigen::Matrix<double, JOINT_NUM, 1> motorPositon_eth;
    Eigen::Matrix<double, JOINT_NUM, 1> jointCommand;

    Eigen::Matrix<double, LLEG_JOINT_FREEDOM_NUM, 1> leftLeg_jointCommand;

    Eigen::Matrix<double, JOINT_FREEDOM_NUM, 1> floatbaseJointCommand;

    Eigen::Vector3d pl;

    enum linkId
    {
      TORSO_LINK = 0,

      LLEG_LINK_1 = 1,
      LLEG_LINK_2 = 2,
      LLEG_LINK_3 = 3,
      LLEG_LINK_4 = 4,
      LLEG_LINK_5 = 5,
      LLEG_LINK_6 = 6,
      LLEG_LINK_SOLE = 7,
      LLEG_LINK_NUM = 7,

      RLEG_LINK_1 = 8,
      RLEG_LINK_2 = 9,
      RLEG_LINK_3 = 10,
      RLEG_LINK_4 = 11,
      RLEG_LINK_5 = 12,
      RLEG_LINK_6 = 13,
      RLEG_LINK_SOLE = 14,
      RLEG_LINK_NUM = 7,

      LARM_LINK_1 = 15,
      LARM_LINK_2 = 16,
      LARM_LINK_3 = 17,
      LARM_LINK_HAND = 18,
      LARM_LINK_NUM = 4,

      RARM_LINK_1 = 19,
      RARM_LINK_2 = 20,
      RARM_LINK_3 = 21,
      RARM_LINK_HAND = 22,
      RARM_LINK_NUM = 4,

      LINK_NUM = 23,
    };

    // private:

    ///fixed base
    Eigen::Matrix<double, JOINT_NUM, 1> uPD;
    Eigen::Matrix<double, JOINT_NUM, 1> err;
    Eigen::Matrix<double, JOINT_NUM, 1> errV;

    Eigen::Matrix<double, JOINT_NUM, 1> qref;
    Eigen::Matrix<double, JOINT_NUM, 1> qrefOld;
    Eigen::Matrix<double, JOINT_NUM, 1> q;
    Eigen::Matrix<double, JOINT_NUM, 1> qold;
    Eigen::Matrix<double, JOINT_NUM, 1> dqref;
    Eigen::Matrix<double, JOINT_NUM, 1> dqrefOld;
    Eigen::Matrix<double, JOINT_NUM, 1> dq;

    static PositionBasedController *m_UniqueInstance;

    ///left leg debug robot
    Eigen::Matrix<double, LLEG_JOINT_FREEDOM_NUM, 1> leftLeg_uPD;
    Eigen::Matrix<double, LLEG_JOINT_FREEDOM_NUM, 1> leftLeg_err;
    Eigen::Matrix<double, LLEG_JOINT_FREEDOM_NUM, 1> leftLeg_errV;

    Eigen::Matrix<double, LLEG_JOINT_CONFIG_NUM, 1> leftLeg_qref;
    Eigen::Matrix<double, LLEG_JOINT_CONFIG_NUM, 1> leftLeg_qrefOld;
    Eigen::Matrix<double, LLEG_JOINT_CONFIG_NUM, 1> leftLeg_q;
    Eigen::Matrix<double, LLEG_JOINT_CONFIG_NUM, 1> leftLeg_qold;
    Eigen::Matrix<double, LLEG_JOINT_FREEDOM_NUM, 1> leftLeg_dqref;
    Eigen::Matrix<double, LLEG_JOINT_FREEDOM_NUM, 1> leftLeg_dqrefOld;
    Eigen::Matrix<double, LLEG_JOINT_FREEDOM_NUM, 1> leftLeg_dq;

    Eigen::Matrix<double, LLEG_JOINT_NUM, 1> dqd, qd;

    ///floating base
    Eigen::Matrix<double, JOINT_FREEDOM_NUM, 1> float_uPD;
    Eigen::Matrix<double, JOINT_FREEDOM_NUM, 1> float_err;
    Eigen::Matrix<double, JOINT_FREEDOM_NUM, 1> float_errV;

    Eigen::Matrix<double, JOINT_CONFIG_NUM, 1> float_qref;
    Eigen::Matrix<double, JOINT_CONFIG_NUM, 1> float_qrefOld;
    Eigen::Matrix<double, JOINT_CONFIG_NUM, 1> float_q;
    Eigen::Matrix<double, JOINT_CONFIG_NUM, 1> float_qold;
    Eigen::Matrix<double, JOINT_FREEDOM_NUM, 1> float_dqref;
    Eigen::Matrix<double, JOINT_FREEDOM_NUM, 1> float_dqrefOld;
    Eigen::Matrix<double, JOINT_FREEDOM_NUM, 1> float_dq;
    Eigen::Matrix<double, JOINT_FREEDOM_NUM, 1> float_dqold;
  };

} // namespace Human

#endif
