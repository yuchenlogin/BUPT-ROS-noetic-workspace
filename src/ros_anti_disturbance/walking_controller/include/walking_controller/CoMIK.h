#ifndef _CoMIK_h_
#define _CoMIK_h_

#include <sys/time.h>
#include <iostream>
#include <iomanip>
#include <limits.h>

#include <Tasks/QPTasks.h>
#include <RBDyn/EulerIntegration.h>
#include <Tasks/Bounds.h>
#include <Tasks/QPConstr.h>
#include <Tasks/QPContactConstr.h>
#include <Tasks/QPMotionConstr.h>

#include "util/Util.h"
#include "RobotDynamics.h"

using namespace std;
using namespace Eigen;
using namespace rbd;
using namespace Util;

class CoMIK
{
private:
  Model::RobotDynamics *robotModel;
  Eigen::Vector3d comWeight;
  Eigen::Vector6d lfWeight, rfWeight;
  std::vector<std::vector<double>> uPosBound, lPosBound, uVelBound, lVelBound;
  double_t dt;

  MultiBody mb;
  MultiBodyConfig mbc;
  std::vector<MultiBody> mbs;
  std::vector<MultiBodyConfig> mbcs;

  Matrix3d rot, rotold;
  VectorXd solved_q;
  VectorXd solved_qold;
  VectorXd solved_dq;
  VectorXd delta_q;

  int maxSolvCount;
  double step;

public:
  sva::PTransformd comRef, torsoRef, lFootRef, rFootRef;
  sva::MotionVecd comvRef;

public:
  CoMIK()
  {
    robotModel = Model::RobotDynamics::getInstance();
    comWeight << 10, 10, 10;
    lfWeight << 10., 10., 10., 100., 100., 100.;
    rfWeight << 10., 10., 10., 100., 100., 100.;
    dt = 0.001;
    defaultBound();

    mb = robotModel->floatMb;
    mbc = robotModel->floatMbc;
    mbs = {mb};
    mbcs = {mbc};

    solved_q.resize(19);
    solved_qold.resize(19);
    solved_dq.resize(18);
    delta_q.resize(12);

    maxSolvCount = 10;
    step = 0.3;
  }
  ~CoMIK() {}

  void setTimeStep(double t)
  {
    dt = t;
  }

  void defaultBound()
  {
    double inf = std::numeric_limits<double>::infinity();

    uPosBound = {{inf, inf, inf, inf, 0.2, 0.2, 0.4},
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
    lPosBound = {{-inf, -inf, -inf, -inf, -0.2, -0.2, 0.4},
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

    uVelBound = {{inf, inf, inf, inf, inf, inf},
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
    lVelBound = {{-inf, -inf, -inf, -inf, -inf, -inf},
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

  void setWeight(Eigen::Vector3d com, Eigen::Vector6d lf, Eigen::Vector6d rf)
  {
    comWeight = com;
    lfWeight = lf;
    rfWeight = rf;
  }

  void setPosBound(std::vector<std::vector<double>> lower, std::vector<std::vector<double>> upper)
  {
    lPosBound = lower;
    uPosBound = upper;
  }

  void setVelBound(std::vector<std::vector<double>> lower, std::vector<std::vector<double>> upper)
  {
    lVelBound = lower;
    uVelBound = upper;
  }

  bool computerJointValue(Eigen::Matrix<double, JOINT_CONFIG_NUM, 1> &float_q_ref,
                          sva::PTransformd com,
                          sva::PTransformd lf, sva::PTransformd rf,
                          sva::PTransformd lf_xbp = sva::PTransformd::Identity(),
                          sva::PTransformd rf_xbp = sva::PTransformd::Identity())
  {
    struct timeval start, end;
    gettimeofday(&start, NULL);

    tasks::qp::QPSolver solver;
    tasks::qp::CoMTask comTask(mbs, 0, com.translation());
    tasks::qp::OrientationTask oriTask(mbs, 0, "Torso", com.rotation());

    lf.translation() += lf_xbp.translation();
    rf.translation() += rf_xbp.translation();
    tasks::qp::TransformTask lposTask(mbs, 0, "leftLegLinkSole", lf, lf_xbp);
    tasks::qp::TransformTask rposTask(mbs, 0, "rightLegLinkSole", rf, rf_xbp);
    tasks::qp::PostureTask postureTask(mbs, 0, mbc.q, 1e-6, 1.);

    tasks::qp::SetPointTask comTaskSp(mbs, 0, &comTask, 10., comWeight, 100);
    tasks::qp::SetPointTask lposTaskSp(mbs, 0, &lposTask, 10., lfWeight, 100);
    tasks::qp::SetPointTask rposTaskSp(mbs, 0, &rposTask, 10., rfWeight, 100);
    tasks::qp::SetPointTask oriTaskSp(mbs, 0, &oriTask, 10., 100.);

    // tasks::qp::DamperJointLimitsConstr dampJointConstr(mbs, 0, {lPosBound, uPosBound}, {lVelBound, uVelBound}, 0.125, 0.025, 1., 0.3);
    // dampJointConstr.addToSolver(solver);
    solver.nrVars(mbs, {}, {});
    solver.updateConstrSize();

    solver.addTask(&comTaskSp);
    solver.addTask(&lposTaskSp);
    solver.addTask(&rposTaskSp);
    solver.addTask(&postureTask);
    solver.addTask(&oriTaskSp);
    solver.updateTasksNrVars(mbs);

    int count = 0;
    double alpha = 1;
    double tasknorm = 0;
    double tasknorm_old = 0;

    while (/*fabs(alpha) > 0.01*/ count < maxSolvCount)
    {
      count++;

      solver.solve(mbs, mbcs);
      rbd::eulerIntegration(mb, mbcs[0], step);
      rbd::forwardKinematics(mb, mbcs[0]);
      rbd::forwardVelocity(mb, mbcs[0]);

      VectorXd q = rbd::paramToVector(mbs[0], mbcs[0].q);
      VectorXd dq = rbd::dofToVector(mbs[0], mbcs[0].alpha);

      tasknorm_old = tasknorm;
      tasknorm = comTask.eval().norm() + lposTask.eval().norm() + rposTask.eval().norm();
      alpha = tasknorm - tasknorm_old;

      solved_q = rbd::paramToVector(mbs[0], mbcs[0].q);
      delta_q = solved_q.segment(7, 12) - solved_qold.segment(7, 12);

      for (uint8_t i = 0; i < 12; i++)
      {
        if (fabs(delta_q[i]) > (90.0 * TO_RADIAN))
        {
          std::cerr << "\ntasks ik failed!\n";
          std::cerr << "joint " << (uint32_t)i << " variation out of range!\n";
          std::cerr << "joint delta_q: " << delta_q.transpose() * Util::TO_DEGREE << "\n";
          std::cerr << "com:" << com.translation().transpose()
                    << " lf:" << lf.translation().transpose()
                    << " rf:" << rf.translation().transpose()
                    << endl
                    << endl;
          return false;
        }
      }
      solved_qold = solved_q;
    }

    solved_qold = rbd::paramToVector(robotModel->floatMb, robotModel->floatMbc.q);
    solved_q = rbd::paramToVector(mbs[0], mbcs[0].q);
    Quaterniond Qold(solved_qold[0], solved_qold[1], solved_qold[2], solved_qold[3]);
    rotold = Qold.toRotationMatrix();
    Quaterniond Q(solved_q[0], solved_q[1], solved_q[2], solved_q[3]);
    rot = Q.toRotationMatrix();
    Vector3d rotvel = sva::rotationError(rot, rotold);
    solved_dq << rotvel, (solved_q.segment(4, 15) - solved_qold.segment(4, 15)) / dt; //set dt virable

    robotModel->floatMbc.q = mbcs[0].q;
    robotModel->floatMbc.alpha = sVectorToDof(robotModel->floatMb, solved_dq);
    rbd::forwardKinematics(robotModel->floatMb, robotModel->floatMbc);
    rbd::forwardVelocity(robotModel->floatMb, robotModel->floatMbc);

    comRef = sva::PTransformd(rbd::computeCoM(robotModel->floatMb, robotModel->floatMbc));
    comvRef = sva::MotionVecd(Vector3d(0., 0., 0.), rbd::computeCoMVelocity(robotModel->floatMb, robotModel->floatMbc));

    torsoRef = mbcs[0].bodyPosW[mb.bodyIndexByName("Torso")];
    lFootRef = mbcs[0].bodyPosW[mb.bodyIndexByName("leftLegLinkSole")];
    rFootRef = mbcs[0].bodyPosW[mb.bodyIndexByName("rightLegLinkSole")];

    float_q_ref = solved_q;

    gettimeofday(&end, NULL);
    double time = (end.tv_sec + end.tv_usec * 1e-6) - (start.tv_sec + start.tv_usec * 1e-6);
    // std::cout << "time: " << time * 1000.0 << "\n";
    return true;
  }

  Eigen::Matrix<double, JOINT_CONFIG_NUM, 1> computerJointValue(bool &result,
                                                                sva::PTransformd com,
                                                                sva::PTransformd lf, sva::PTransformd rf,
                                                                sva::PTransformd lf_xbp = sva::PTransformd::Identity(),
                                                                sva::PTransformd rf_xbp = sva::PTransformd::Identity())
  {
    Eigen::Matrix<double, JOINT_CONFIG_NUM, 1> float_q_ref;
    float_q_ref.resize(JOINT_CONFIG_NUM);
    if (!computerJointValue(float_q_ref, com, lf, rf, lf_xbp, rf_xbp))
    {
      result = false;
    }
    result = true;
    return float_q_ref;
  }
};

#endif
