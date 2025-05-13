#ifndef _walkingDataStruct_h_
#define _walkingDataStruct_h_

#include <iostream>
#include <Eigen/Core>
#include <SpaceVecAlg/SpaceVecAlg>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>

namespace Walking
{
  typedef struct
  {
    sva::PTransformd pos;
    sva::MotionVecd vel;
    sva::MotionVecd acc;
  } BodyAttr_t;

  typedef struct
  {
    BodyAttr_t CoM;
    BodyAttr_t torso;
    BodyAttr_t lFoot;
    BodyAttr_t rFoot;
    BodyAttr_t lHand;
    BodyAttr_t rHand;
  } WholeBodyState_t;
  
  enum LegState
  {
    RIGHT_STANCE = -1,
    DOUBLE_STANCE = 0,
    LEFT_STANCE = 1,
  };

  enum StateOfContact
  {
    RIGHT_CONTACT = -1,
    DOUBLE_CONTACT = 0,
    LEFT_CONTACT = 1,
    UNSTABLE_CONTACT = 2,
  };

  enum ChangeOfPhase
  {
    NoChange = 0,
    DtoL,
    DtoR,
    LtoD,
    RtoD
  };

  typedef struct
  {
    uint32_t timeCount;
    uint32_t ssCount;
    uint32_t dsCount;
    LegState legS, legS_old;
    ChangeOfPhase phaseChange;
  } StepState_t;

  typedef struct
  {
    double_t dt;
    uint32_t ds1TotalCnt, ssTotalCnt, ds2TotalCnt; // double support, single support
    uint32_t totalCnt;
  } StepParam_t;

} // namespace Walking

#endif
