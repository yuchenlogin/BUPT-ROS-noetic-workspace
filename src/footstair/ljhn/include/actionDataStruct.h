#ifndef _actionDataStruct_h_
#define _actionDataStruct_h_

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <SpaceVecAlg/SpaceVecAlg>

#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>

namespace Action
{
  typedef struct
  {
    sva::PTransformd pos;
    sva::MotionVecd vel;
    sva::MotionVecd acc;
  } bodyAttr_t;

  typedef struct
  {
    bodyAttr_t CoM;
    bodyAttr_t torso;
    bodyAttr_t lFoot;
    bodyAttr_t rFoot;
    bodyAttr_t lHand;
    bodyAttr_t rHand;
  } WholeMotion_t;
}; // namespace Action

#endif
