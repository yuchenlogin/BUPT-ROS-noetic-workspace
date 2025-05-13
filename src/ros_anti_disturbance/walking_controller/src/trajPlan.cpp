#include "trajPlan.h"

void QuinticSpline::computerCoeff(Eigen::Vector6d &bound, Eigen::Vector6d &coef, double_t t)
{
  Eigen::Matrix<double, 6, 6> m6X6;
  m6X6 << 1, 0, 0, 0, 0, 0,
      1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5),
      0, 1, 0, 0, 0, 0,
      0, 1, 2 * t, 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4),
      0, 0, 2, 0, 0, 0,
      0, 0, 2, 6 * t, 12 * pow(t, 2), 20 * pow(t, 3);

  coef = m6X6.inverse() * bound;
}

double_t QuinticSpline::computerPosition(Eigen::Vector6d &coef, double_t t)
{
  return (coef(0) + coef(1) * t + coef(2) * pow(t, 2) + coef(3) * pow(t, 3) + coef(4) * pow(t, 4) + coef(5) * pow(t, 5));
}

double_t QuinticSpline::computerVelocity(Eigen::Vector6d &coef, double_t t)
{
  return (coef(1) + 2 * coef(2) * t + 3 * coef(3) * pow(t, 2) + 4 * coef(4) * pow(t, 3) + 5 * coef(5) * pow(t, 4));
}

double_t QuinticSpline::computerAccel(Eigen::Vector6d &coef, double_t t)
{
  return (2 * coef(2) + 6 * coef(3) * t + 12 * (coef(4)) * pow(t, 2) + 20 * (coef(5)) * pow(t, 3));
}
