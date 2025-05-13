
#ifndef DCM_BASED_PLANNING_H
#define DCM_BASED_PLANNING_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Util.h"

namespace Eigen
{
  typedef Matrix<double, 6, 1> Vector6d;
}

namespace Human
{

  class Interpolation
  {
  public:
    Interpolation()
    {
    }

    Interpolation(Eigen::Vector6d bound, double t)
    {

      bound_ = bound;
      t_ = t;

      Eigen::Matrix<double, 6, 6> m6X6;
      Eigen::Vector6d coef; //coef--(a0,a1,a2,a3,a4,a5)

      m6X6 << 1, 0, 0, 0, 0, 0,
          1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5),
          0, 1, 0, 0, 0, 0,
          0, 1, 2 * t, 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4),
          0, 0, 2, 0, 0, 0,
          0, 0, 2, 6 * t, 12 * pow(t, 2), 20 * pow(t, 3);

      coef_ = m6X6.inverse() * bound;
    };

    virtual ~Interpolation(){

    };

    double getPos(double t)
    {
      t = t > t_ ? t_ : t;

      Eigen::Vector6d tVec;
      tVec << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);
      double p = tVec.dot(coef_);

      return p;
    };

    double getVel(double t)
    {
      t = t > t_ ? t_ : t;

      Eigen::Vector6d tVec;
      tVec << 0, 1, t, pow(t, 2), pow(t, 3), pow(t, 4);
      Eigen::Vector6d vcoef;
      vcoef << 0, coef_(1), 2 * coef_(2), 3 * coef_(3), 4 * coef_(4), 5 * coef_(5);
      double v = tVec.dot(coef_);

      return v;
    };

    double getAcc(double t)
    {
      t = t > t_ ? t_ : t;

      Eigen::Vector6d tVec;
      tVec << 0, 0, 1, t, pow(t, 2), pow(t, 3);
      Eigen::Vector6d vcoef;
      vcoef << 0, 0, 2 * coef_(2), 6 * coef_(3), 12 * coef_(4), 20 * coef_(5);
      double a = tVec.dot(coef_);

      return a;
    };

    // private:

    Vector6d coef_, bound_;
    double t_;
  };

  class DCMBasedPlanning
  {

  public:
    DCMBasedPlanning();
    virtual ~DCMBasedPlanning();

    static DCMBasedPlanning *GetInstance()
    {
      return m_UniqueInstance;
    }
    static DCMBasedPlanning *m_UniqueInstance;

    enum LegState
    {
      rightStance = -1,
      doublesupport = 0,

      leftStance = 1,

      leftTouching,
      rightTouching,

      leftLifting,
      rightLifting,

      stand
    };

    void clear();

    double Tstep;
    double Tdsini;
    double Tdsend;
    double swingH;
    double stepLength;
    double stepWidth;
    double lFootHoffset;
    double rFootHoffset;

    struct RobotState
    {
      Vector2d cp;
      Vector2d cpv;
      Vector3d com;
      Vector3d comv;
      Vector3d coma;
      double DSrate;
      double LSRSrate;
      LegState legs;
      int timecount;
    };
    void computeCpTraj(std::vector<Vector2d> footprint, RobotState *robot);
    std::vector<Vector2d> cprefvec;
    std::vector<Vector2d> coprefvec;
    std::vector<Vector3d> comRefTrajvec;
    std::vector<LegState> legStatevec;
    std::vector<double> WrightDistvec;

    std::vector<Vector3d> CoMavec;
    std::vector<Vector3d> CoMVvec;
    std::vector<Vector3d> CoMvec;

    Eigen::Vector6d fifthPolyInterpInit(Eigen::Vector6d bound, double t);
    double fifthPolyInterp(Eigen::Vector6d coef, double t);

    double timeStep;
    Eigen::VectorXd jointValue;

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

      LLEG_JOINT_FREEDOM_NUM = 12, //6,//
      LLEG_JOINT_CONFIG_NUM = 13,  //6,//
    };

    // private:

    double TimeCon_x;
    double TimeCon_y;

    double TorsoHeight;     //torso height from forward kinematics
    double TorsoHeightWalk; //torso height at walking
    double gra_g;           //gravity g
  };

} // namespace Human

#endif
