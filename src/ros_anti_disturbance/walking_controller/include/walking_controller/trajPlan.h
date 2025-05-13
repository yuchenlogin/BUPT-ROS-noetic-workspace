#ifndef _trajPlan_h_
#define _trajPlan_h_

#include <iostream>
#include <math.h>
#include <vector>
#include <Eigen/Dense>

namespace Eigen
{
  typedef Matrix<double, 6, 1> Vector6d;
}

class QuinticSpline
{
public:
  QuinticSpline(){}
  ~QuinticSpline(){}

  void computerCoeff(Eigen::Vector6d &bound, Eigen::Vector6d &coef, double_t t);
  double_t computerPosition(Eigen::Vector6d &coef, double_t t);
  double_t computerVelocity(Eigen::Vector6d &coef, double_t t);
  double_t computerAccel(Eigen::Vector6d &coef, double_t t);
};

#endif
