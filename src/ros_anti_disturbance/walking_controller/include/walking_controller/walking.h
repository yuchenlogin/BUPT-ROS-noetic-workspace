#ifndef _walking_h_
#define _walking_h_

#include <iostream>
#include "walkingDataStruct.h"
#include "RobotDynamics.h"
#include "CoMIK.h"
#include "Kalman.h"
#include "DCMBasedPlanning.h"
#include "trajPlan.h"

namespace Walking
{

#define LFOOT_FSR_THRESHOLD 6.5
#define RFOOT_FSR_THRESHOLD 6.5
#define LFOOT_FSR_MASK 0X0F
#define RFOOT_FSR_MASK 0XF0
#define GRAVITY 9.81

  class Walking
  {
  private:
    void (*exit_fun)(void);

    Model::RobotDynamics *robotModel;
    rbd::MultiBodyConfig mbcMeasure;
    rbd::MultiBodyConfig FMbcMeasure;
    Eigen::Vector3d copMeasure;
    Eigen::Quaterniond imuQuat;

    Kalman *torsoFilter;
    Eigen::Matrix<double, 15, 1> torsofilter_result;
    Eigen::Matrix<double, 15, 1> torsofilter_measure;

    std::vector<Eigen::Vector6d> gaitCommand;
    std::vector<Eigen::Vector2d> footprints;
    QuinticSpline spline;

    Eigen::Vector6d swingFootPx, swingFootPy, swingFootPz1, swingFootPz2; // bound
    Eigen::Vector6d swingFootAx, swingFootAy, swingFootAz1, swingFootAz2; // coef

    StateOfContact contactState;

    Eigen::Vector2d hipOffsetV;
    Eigen::Vector2d currHipOffset;

  public:
    CoMIK comIK;
    DCM::DCMBasedPlanning dcmplanner;
    Eigen::Matrix<double, JOINT_CONFIG_NUM, 1> float_q;
    Eigen::Matrix<double, JOINT_CONFIG_NUM, 1> float_q_old;
    Eigen::Matrix<double, JOINT_FREEDOM_NUM, 1> float_dq;
    Eigen::Matrix<double, JOINT_FREEDOM_NUM, 1> float_dq_old;
    Eigen::Matrix<double, JOINT_CONFIG_NUM, 1> float_q_ref;
    Eigen::Vector6d lFootFT, rFootFT;
    Eigen::Vector3d imuGyro, imuAcc;
    Eigen::Vector3d imuGyroInW, imuAccInW;
    Eigen::Vector3d imuEuler;

    StepState_t stepState;
    StepParam_t stepParam;
    WholeBodyState_t botState;
    WholeBodyState_t liftState;
    WholeBodyState_t touchState;
    WholeBodyState_t endState;
    WholeBodyState_t initParam;
    WholeBodyState_t botParam;

    uint32_t stepCount;
    uint32_t stepCountTarget;
    bool running;
    bool lastRunning;

  public:
    Walking();
    ~Walking();

    void WalkingInit();
    void setTimeStep(double_t s);
    void setHipOffset(double_t left, double_t right);
    void setExit(void (*exit)(void));
    bool getInitJointValue(std::vector<double_t> &jv);
    std::vector<double_t> getInitJointValue();
    void start();
    void end();
    void imuUpdate();
    void computerCop();
    MatrixXd getTorsoF(Quaterniond quat, Vector3d acc, Vector3d gyro, double dt);
    void stateUpdate();
    uint16_t contactGroundDetect();
    void legPhaseUpdate();
    bool footPrintUpdate(Eigen::Vector6d d_fp);
    void nextStepUpdate();
    void planSwingFoot();
    void computerSwingFoot();
    void paramUpdate();
    void timeUpdate();
    void hipOffset();
    void deHipOffset();
    void stepCountUpdate();
    void planner();

    void torsoBalance();
    double theta, thetaVel, thetafoot;
    bool enabletorsobalance;
    Vector3d com_decay;
    Vector3d com_highpass;
    Vector3d com_lowpass;

    void run();
  };
}; // namespace Walking

#endif
