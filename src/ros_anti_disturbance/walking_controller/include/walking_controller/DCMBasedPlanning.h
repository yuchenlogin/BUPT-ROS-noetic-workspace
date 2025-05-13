
#ifndef DCM_BASED_PLANNING_H
#define DCM_BASED_PLANNING_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "util/Util.h"
#include "TaskSolver.h"

#define Gravity 9.7925
#define TotalWeight 430
#define Pi 3.14159265358979323846

template <class V>
constexpr V sqr(const V &a) { return a * a; }

namespace Eigen
{
  typedef Matrix<double, 6, 1> Vector6d;
}

namespace DCM
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

    void setTimeStep(double_t s);
    void clear();

    double timeStep;
    double T_step;
    double T_dsBegin;
    double T_dsEnd;
    double swingH;
    double stepLength;
    double stepWidth;
    double lFootHoffset;
    double rFootHoffset;

    struct RobotState
    {
      Vector2d cp;
      Vector2d cpv;
      Vector2d cop;
      Vector2d cop1;
      Vector2d cop2;
      Vector3d com;
      Vector3d comv;
      Vector3d coma;
      double DSrate;
      double LSRSrate;
      LegState legs;
      LegState legs_old;
      int timecount;
    };
    void computeCpTraj(std::vector<Vector2d> footprint, RobotState *robot);
    std::vector<Vector2d> cprefvec;
    std::vector<Vector2d> coprefvec;
    std::vector<Vector3d> comRefTrajvec;
    std::vector<double> WrightDistvec;
    std::vector<Vector3d> CoMavec;
    std::vector<Vector3d> CoMVvec;
    std::vector<Vector3d> CoMvec;

    std::vector<Vector2d> copvec;

    std::vector<Vector3d> lFootRefTrajvec;
    std::vector<Vector3d> rFootRefTrajvec;
    std::vector<LegState> legStatevec;

    void plannerInit(std::vector<Vector2d> FootPrint);
    void plan(std::vector<Vector2d> FootPrint);
    int ReplanStepNum;

    void computeOptimalFootprint(RobotState *robot);
    std::vector<Vector2d> Footprint;

    void computeCoPTraj(std::vector<Vector2d> footprint, RobotState *robot);
    void computelegSTraj(std::vector<Vector2d> footprint, RobotState *robot);

    std::vector<Vector2d> optcopref;
    void computeOptimalTrajIterative(RobotState *robot);

    OSQPTasks::TaskSolver DynSolver;
    std::vector<c_float> lvecglobal, uvecglobal;
    SparseMatrix<double> Acostglobal, Pglobal, Aglobal;
    void GetSolver(DCMBasedPlanning *dcmplanner);

    void OptimalTrajInit(RobotState *robot);
    void computeOptimalTraj(RobotState *robot);

    Eigen::VectorXd jointValue;

    // private:
    double ss_cop_len;

    double omega_x;
    double omega_y;
    double CoMH;

    double TorsoHeight;     //torso height from forward kinematics
    double TorsoHeightWalk; //torso height at walking
    double gra_g;           //gravity g
  };

} // namespace DCM

#endif
