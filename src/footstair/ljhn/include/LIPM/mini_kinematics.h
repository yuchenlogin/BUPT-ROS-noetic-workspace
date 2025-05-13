
#ifndef __MINI_KINEMATICS__
#define  __MINI_KINEMATICS__

  Eigen::Matrix4d forward_kinematics(Eigen::Matrix<double,6,1> t,int chain);
  Eigen::Matrix<double,6,1> inverse_kinematics(Eigen::Matrix4d p,int chain);

#endif

