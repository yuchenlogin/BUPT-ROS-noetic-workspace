#include "walking.h"

namespace Walking
{
  Walking::Walking(/* args */)
  {
    robotModel = Model::RobotDynamics::getInstance();
    mbcMeasure = rbd::MultiBodyConfig(robotModel->mb);
    FMbcMeasure = rbd::MultiBodyConfig(robotModel->floatMb);
    mbcMeasure.gravity << 0., 0., GRAVITY;

    torsoFilter = new Kalman(15, 15);
    // double torsoQ[] = {7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 1, 1, 1};
    // double torsoR[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    double torsoQ[] = {1, 1, 1, 1, 1, 1, 7, 8, 7, 50, 50, 1.0, 1, 1, 1};
    double torsoR[] = {1, 1, 1, 1, 1, 1, 1, 0.5, 1, 8.0, 8.0, 8.0, 1, 1, 1};
    torsoFilter->setQR(torsoQ, torsoR);

    stepState.timeCount = 0;
    stepState.dsCount = 0;
    stepState.legS = DOUBLE_STANCE;
    stepState.legS_old = LEFT_STANCE;
    stepState.phaseChange = NoChange;

    running = false;
    lastRunning = false;

    setTimeStep(0.001);
    setHipOffset(0.0, 0.0);
  }

  Walking::~Walking()
  {
  }

  void Walking::WalkingInit()
  {
    double steplength = dcmplanner.stepLength;
    double stepWidth = dcmplanner.stepWidth;
    footprints.push_back(Vector2d(0., -stepWidth));
    footprints.push_back(Vector2d(0., stepWidth));
    footprints.push_back(Vector2d(1. * steplength, -stepWidth));
    footprints.push_back(Vector2d(2. * steplength, stepWidth));
    footprints.push_back(Vector2d(3. * steplength, -stepWidth));
    footprints.push_back(Vector2d(4. * steplength, stepWidth));
    footprints.push_back(Vector2d(5. * steplength, -stepWidth));
    footprints.push_back(Vector2d(5. * steplength, stepWidth));
    footprints.push_back(Vector2d(5. * steplength, -stepWidth));
    footprints.push_back(Vector2d(5. * steplength, stepWidth));
    footprints.push_back(Vector2d(5. * steplength, -stepWidth));
    footprints.push_back(Vector2d(5. * steplength, stepWidth));
    footprints.push_back(Vector2d(5. * steplength, -stepWidth));
    footprints.push_back(Vector2d(5. * steplength, stepWidth));
    footprints.push_back(Vector2d(5. * steplength, -stepWidth));
    footprints.push_back(Vector2d(5. * steplength, stepWidth));
    dcmplanner.plannerInit(footprints);

    initParam.CoM.pos = sva::PTransformd(dcmplanner.CoMvec[0]);
    initParam.lFoot.pos = sva::PTransformd(Eigen::Vector3d(0., dcmplanner.stepWidth, 0.));
    initParam.rFoot.pos = sva::PTransformd(Eigen::Vector3d(0., -dcmplanner.stepWidth, 0.));
    start();
    nextStepUpdate();
    enabletorsobalance = true;
  }

  void Walking::setTimeStep(double_t s)
  {
    dcmplanner.setTimeStep(s);
    comIK.setTimeStep(s);
    stepParam.dt = s;
  }

  void Walking::setExit(void (*fun)(void))
  {
    exit_fun = fun;
  }

  bool Walking::getInitJointValue(std::vector<double_t> &jv)
  {
    if (comIK.computerJointValue(float_q_ref, botParam.CoM.pos, botParam.lFoot.pos, botParam.rFoot.pos))
    {
      for (uint16_t i = 0; i < JOINT_NUM; i++)
      {
        jv[i] = float_q_ref[FLOATING_CONFIG_NUM + i] * Util::TO_DEGREE;
      }
      return true;
    }
    return false;
  }

  std::vector<double_t> Walking::getInitJointValue()
  {
    std::vector<double_t> jv;
    jv.resize(JOINT_NUM);
    getInitJointValue(jv);
    return jv;
  }

  void Walking::start()
  {
    stepCount = 0;
    stepCountTarget = footprints.size() - 1;

    stepState.timeCount = 0;
    stepState.dsCount = 0;
    stepState.legS = DOUBLE_STANCE;
    stepState.legS_old = LEFT_STANCE;
    stepState.phaseChange = NoChange;

    botParam = initParam;
    botState = botParam;
    liftState = botParam;
    touchState = botParam;
    endState = botParam;
    if (!comIK.computerJointValue(float_q_ref, botParam.CoM.pos, botParam.lFoot.pos, botParam.rFoot.pos))
    {
      exit_fun();
      return;
    }
    float_q = float_q_ref;
    float_q_old = float_q;
    float_dq_old = float_dq;
  }

  void Walking::end()
  {
    start();
  }

  void Walking::imuUpdate()
  {
    imuQuat = AngleAxisd(imuEuler[2], Vector3d::UnitZ()) * AngleAxisd(imuEuler[1], Vector3d::UnitY()) * AngleAxisd(imuEuler[0], Vector3d::UnitX());
    Matrix3d rot = imuQuat.toRotationMatrix();
    imuGyroInW = rot * imuGyro;
    imuAccInW = rot * imuAcc;
#if 0
    std::cout << "imuGyroInW: " << imuGyroInW.transpose() << ", "
              << "imuAccInW: " << imuAccInW.transpose() << ", "
              << "imuEuler: " << imuEuler.transpose() << "\n\n";
#endif
  }

  void Walking::computerCop() //copMeasure
  {
    Eigen::Vector3d lcopm;
    Eigen::Vector3d rcopm;
    if (lFootFT(5) > 20)
      lcopm << -lFootFT(1) / lFootFT(5) - Foot_X, lFootFT(0) / lFootFT(5), 0.;
    else
      lcopm << 0., 0., 0.;
    if (rFootFT(5) > 20)
      rcopm << -rFootFT(1) / rFootFT(5) - Foot_X, rFootFT(0) / rFootFT(5), 0.;
    else
      rcopm << 0., 0.0, 0.;

    Vector3d lleg = botParam.lFoot.pos.translation(); //leftFP.back();
    Vector3d rleg = botParam.rFoot.pos.translation(); //rightFP.back();

    //TODO: transform form sole frame to world frame
    lcopm = Matrix3d::Identity() * lcopm + lleg;
    rcopm = Matrix3d::Identity() * rcopm + rleg;

    if (stepState.legS == LEFT_STANCE)
      copMeasure = lcopm;
    else if (stepState.legS == RIGHT_STANCE)
      copMeasure = rcopm;
    else
    {
      if (lFootFT(5) + rFootFT(5) < 50)
      {
        copMeasure = Vector3d(0., 0., 0.);
      }
      else
      {
        copMeasure = (lcopm * lFootFT(5) + rcopm * rFootFT(5)) / (lFootFT(5) + rFootFT(5));
      }
    }
  }

  MatrixXd Walking::getTorsoF(Quaterniond quat, Vector3d acc, Vector3d gyro, double dt)
  {

    Matrix3d Identity3d = Matrix3d::Identity();
    Matrix3d zero3d = MatrixXd::Zero(3, 3);

    Vector3d v = dt * gyro;
    Matrix3d m1;
    m1 << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    m1 = Identity3d + m1 * sin(v.norm()) + m1 * m1 * (1 - cos(v.norm()));

    Matrix3d m2_1 = quat.toRotationMatrix().transpose();

    Vector3d g = Vector3d(0., 0., GRAVITY);
    // Vector3d v1 = acc-g;
    Vector3d v1 = m2_1 * (acc - g);
    Matrix3d m2_2;
    m2_2 << 0, -v1(2), v1(1),
        v1(2), 0, -v1(0),
        -v1(1), v1(0), 0;

    // m2_2 << v1(0), 0, 0,
    //     0, v1(1), 0,
    //     0, 0, v1(2);

    MatrixXd F(15, 15);
    F << Identity3d, zero3d, dt * Identity3d, zero3d, zero3d,
        zero3d, m1, zero3d, zero3d, zero3d,
        zero3d, dt * m2_2, Identity3d, zero3d, -dt * m2_1,
        zero3d, zero3d, zero3d, Identity3d, zero3d,
        zero3d, zero3d, zero3d, zero3d, Identity3d;

    return F;
  }

  void Walking::stateUpdate()
  {
    imuUpdate();
    computerCop();

    // float_q
    {
      if (running == true)
      {
        deHipOffset();
      }
      mbcMeasure.q = sVectorToParam(robotModel->mb, float_q.segment<JOINT_NUM>(FLOATING_CONFIG_NUM));
      mbcMeasure.alpha = sVectorToParam(robotModel->mb, float_dq.segment<JOINT_NUM>(FLOATING_FREEDOM_NUM));
      rbd::forwardKinematics(robotModel->mb, mbcMeasure);

      sva::PTransformd lFootInTorso = mbcMeasure.bodyPosW[robotModel->mb.bodyIndexByName("leftLegLinkSole")];
      sva::PTransformd rFootInTorso = mbcMeasure.bodyPosW[robotModel->mb.bodyIndexByName("rightLegLinkSole")];
      // foot rot
      Matrix3d torsoRotInW = imuQuat.toRotationMatrix().transpose();
      Matrix3d lFootRotInW = lFootInTorso.rotation() * torsoRotInW;
      Matrix3d rFootRotInW = rFootInTorso.rotation() * torsoRotInW;
      sva::PTransformd lFootInW(lFootRotInW, liftState.lFoot.pos.translation());
      sva::PTransformd rFootInW(rFootRotInW, liftState.rFoot.pos.translation());

      sva::PTransformd torsoInW;
      if (stepState.legS == RIGHT_STANCE || (stepState.legS == DOUBLE_STANCE && stepState.legS_old == RIGHT_STANCE))
      {
        torsoInW = rFootInTorso.inv() * rFootInW;
      }
      else if (stepState.legS == LEFT_STANCE || (stepState.legS == DOUBLE_STANCE && stepState.legS_old == LEFT_STANCE))
      {
        torsoInW = lFootInTorso.inv() * lFootInW;
      }

      float_q.segment<4>(0) << imuQuat.w(), imuQuat.x(), imuQuat.y(), imuQuat.z();
      float_q.segment<3>(4) = torsoInW.translation();

      botState.torso.pos = torsoInW;
    }

    // float_dq
    {
      float_dq.segment<3>(0) = imuGyroInW;
      float_dq.segment<3>(3) = (float_q.segment<3>(4) - float_q_old.segment<3>(4)) / stepParam.dt;
      float_dq.segment<JOINT_NUM>(FLOATING_FREEDOM_NUM) = (float_q.segment<JOINT_NUM>(7) - float_q_old.segment<JOINT_NUM>(7)) / stepParam.dt;

      botState.torso.vel.angular() = imuGyroInW;
      botState.torso.vel.linear() = float_dq.segment<3>(3);
    }

    // float_dq with torso filter
    {
      Vector3d gyro = torsofilter_result.segment<3>(9);
      MatrixXd F = getTorsoF(imuQuat, imuAccInW, gyro, stepParam.dt);
      MatrixXd H(15, 15);
      H.setIdentity();
      torsoFilter->update_F_H(F, H);

      // torsofilter_measure << float_q.segment<3>(4), imuEuler, float_dq.segment<3>(3), float_dq.segment<3>(0), Vector3d(0, 0, GRAVITY);
      torsofilter_measure << float_q.segment<3>(4), imuEuler, float_dq.segment<3>(3), float_dq.segment<3>(0), Vector3d(0., 0., 0.);
      torsofilter_result = torsoFilter->updateData(torsofilter_measure);

      botState.torso.pos.translation() = torsofilter_result.segment<3>(0);
      botState.torso.vel.linear() = torsofilter_result.segment<3>(6);
      botState.torso.vel.angular() = torsofilter_result.segment<3>(9);

      float_dq.segment<6>(0) << botState.torso.vel.angular(), botState.torso.vel.linear();
    }

    // computer CoM
    {
      FMbcMeasure.zero(robotModel->floatMb);
      FMbcMeasure.q = sVectorToParam(robotModel->floatMb, float_q);
      rbd::forwardKinematics(robotModel->floatMb, FMbcMeasure);

      FMbcMeasure.alpha = sVectorToDof(robotModel->floatMb, float_dq);
      rbd::forwardVelocity(robotModel->floatMb, FMbcMeasure);

      FMbcMeasure.alphaD = sVectorToDof(robotModel->floatMb, (float_dq - float_dq_old) / stepParam.dt);
      rbd::forwardAcceleration(robotModel->floatMb, FMbcMeasure);

      botState.CoM.pos.translation() = rbd::computeCoM(robotModel->floatMb, FMbcMeasure);
      botState.CoM.vel.linear() = rbd::computeCoMVelocity(robotModel->floatMb, FMbcMeasure);
      botState.CoM.acc.linear() = rbd::computeCoMAcceleration(robotModel->floatMb, FMbcMeasure);
    }

    botState.lFoot.pos = FMbcMeasure.bodyPosW[robotModel->mb.bodyIndexByName("leftLegLinkSole")];
    botState.rFoot.pos = FMbcMeasure.bodyPosW[robotModel->mb.bodyIndexByName("rightLegLinkSole")];

    float_q_old = float_q;
    float_dq_old = float_dq;
  }

  uint16_t Walking::contactGroundDetect()
  {
    uint16_t contactPoint = 0;

    for (uint16_t i = 0; i < 4; i++)
    {
      if (lFootFT[i] > LFOOT_FSR_THRESHOLD)
        contactPoint |= (1 << i);
      if (rFootFT[i] > RFOOT_FSR_THRESHOLD)
        contactPoint |= (1 << (i + 4));
    }

    if ((contactPoint & LFOOT_FSR_MASK) != 0 && (contactPoint & RFOOT_FSR_MASK) != 0)
    {
      contactState = DOUBLE_CONTACT;
    }
    else if ((contactPoint & LFOOT_FSR_MASK) != 0 && (contactPoint & RFOOT_FSR_MASK) == 0)
    {
      contactState = LEFT_CONTACT;
    }
    else if ((contactPoint & LFOOT_FSR_MASK) == 0 && (contactPoint & RFOOT_FSR_MASK) != 0)
    {
      contactState = RIGHT_CONTACT;
    }
    else
    {
      contactState = UNSTABLE_CONTACT;
    }

    return contactPoint;
  }

  void Walking::legPhaseUpdate()
  {
    static uint16_t legSwingTime = 0;
    uint16_t contactPoint = contactGroundDetect();
    stepState.phaseChange = NoChange;
    if (stepState.legS != DOUBLE_STANCE)
    {
      legSwingTime++;
      // if (legSwingTime > (stepParam.ssTotalCnt * 0.6))
      if (legSwingTime >= (stepParam.ssTotalCnt))
      {
        // if (stepState.legS == LEFT_STANCE && ((contactPoint & RFOOT_FSR_MASK) != 0))
        if (stepState.legS == LEFT_STANCE)
        {
          stepState.legS_old = stepState.legS;
          stepState.legS = DOUBLE_STANCE;
          stepState.phaseChange = LtoD;
          std::cout << "LtoD\n";
        }
        // else if (stepState.legS == RIGHT_STANCE && ((contactPoint & LFOOT_FSR_MASK) != 0))
        else if (stepState.legS == RIGHT_STANCE)
        {
          stepState.legS_old = stepState.legS;
          stepState.legS = DOUBLE_STANCE;
          stepState.phaseChange = RtoD;
          std::cout << "RtoD\n";
        }
      }
    }
    else if (stepState.timeCount == stepParam.ds1TotalCnt)
    {
      if (stepState.legS_old == LEFT_STANCE)
      {
        stepState.legS = RIGHT_STANCE;
        stepState.phaseChange = DtoR;
        std::cout << "DtoR\n";
      }
      else if (stepState.legS_old == RIGHT_STANCE)
      {
        stepState.legS = LEFT_STANCE;
        stepState.phaseChange = DtoL;
        std::cout << "DtoL\n";
      }
    }

    if (stepState.phaseChange == DtoL || stepState.phaseChange == DtoR)
    {
      legSwingTime = 0;
    }
  }

  bool Walking::footPrintUpdate(Eigen::Vector6d d_fp)
  {
  }

  void Walking::nextStepUpdate()
  {
    // footPrintUpdate(gaitCommand[stepCount]);
    // stepCountTarget = footprints.size() - 1;
    // dcmplanner.plan(footprints);
    stepParam.totalCnt = ceil(dcmplanner.T_step / stepParam.dt);
    stepParam.ds1TotalCnt = ceil(dcmplanner.T_dsBegin / stepParam.dt);
    stepParam.ds2TotalCnt = ceil(dcmplanner.T_dsEnd / stepParam.dt);
    stepParam.ssTotalCnt = stepParam.totalCnt - (stepParam.ds1TotalCnt + stepParam.ds2TotalCnt);
    std::cout << "stepParam cnt: " << stepParam.ds1TotalCnt << ", " << stepParam.ssTotalCnt << ", " << stepParam.ds2TotalCnt << "\n";
  }

  void Walking::planSwingFoot()
  {
    if (stepCount > 0 && stepCount < stepCountTarget)
    {
      swingFootPx << footprints[stepCount - 1].x(), footprints[stepCount + 1].x(), 0., 0., 0., 0.;
      swingFootPy << footprints[stepCount - 1].y(), footprints[stepCount + 1].y(), 0., 0., 0., 0.;
      swingFootPz1 << 0., dcmplanner.swingH, 0., 0., 0., 0.;
      swingFootPz2 << dcmplanner.swingH, 0., 0., 0., 0., 0.;
      spline.computerCoeff(swingFootPx, swingFootAx, stepParam.ssTotalCnt);
      spline.computerCoeff(swingFootPy, swingFootAy, stepParam.ssTotalCnt);
      spline.computerCoeff(swingFootPz1, swingFootAz1, stepParam.ssTotalCnt / 2.0);
      spline.computerCoeff(swingFootPz2, swingFootAz2, stepParam.ssTotalCnt / 2.0);
    }
  }

  void Walking::computerSwingFoot()
  {
    if (stepCount > 0 && stepCount < stepCountTarget)
    {
      Eigen::Vector6d footPos;
      if (stepState.ssCount <= stepParam.ssTotalCnt)
      {
        footPos[0] = spline.computerPosition(swingFootAx, stepState.ssCount);
        footPos[1] = spline.computerPosition(swingFootAy, stepState.ssCount);
      }
      else
      {
        footPos[0] = spline.computerPosition(swingFootAx, stepParam.ssTotalCnt);
        footPos[1] = spline.computerPosition(swingFootAy, stepParam.ssTotalCnt);
      }

      if (stepState.ssCount <= stepParam.ssTotalCnt / 2.0)
      {
        footPos[2] = spline.computerPosition(swingFootAz1, stepState.ssCount);
      }
      else if ((stepState.ssCount > stepParam.ssTotalCnt / 2.0) && (stepState.ssCount <= stepParam.ssTotalCnt))
      {
        footPos[2] = spline.computerPosition(swingFootAz2, stepState.ssCount - (stepParam.ssTotalCnt / 2.0));
      }
      else
      {
        footPos[2] = spline.computerPosition(swingFootAz2, stepParam.ssTotalCnt / 2.0);
      }

      if (stepState.legS == LEFT_STANCE)
      {
        botParam.rFoot.pos.translation() = footPos.segment<3>(0);
      }
      else if (stepState.legS == RIGHT_STANCE)
      {
        botParam.lFoot.pos.translation() = footPos.segment<3>(0);
      }
      // std::cout << "footPos: " << footPos.segment<3>(0).transpose() << "\n";
    }
  }

  void Walking::paramUpdate()
  {
    if (stepState.phaseChange == DtoL || stepState.phaseChange == DtoR)
    {
      liftState = botState;
#if 0
      std::cout << "lift lFoot pos:\n"
                << liftState.lFoot.pos.translation() << "\n";
      std::cout << "lift rFoot pos:\n"
                << liftState.rFoot.pos.translation() << "\n";
#endif
    }
    if (stepState.phaseChange == LtoD || stepState.phaseChange == RtoD)
    {
      touchState = botState;
    }

    if (stepState.phaseChange == DtoL || stepState.phaseChange == DtoR)
    {
      planSwingFoot();
    }

    if (stepState.legS == DOUBLE_STANCE)
    {
    }
    else
    {
      computerSwingFoot();
    }
    botParam.CoM.pos.translation() = dcmplanner.CoMvec[stepCount * stepParam.totalCnt + stepState.timeCount];
    botParam.CoM.vel.linear() = dcmplanner.CoMVvec[stepCount * stepParam.totalCnt + stepState.timeCount];
    botParam.torso.pos = comIK.torsoRef;
    // std::cout << "CoM: " << dcmplanner.CoMvec[stepCount * stepParam.totalCnt + stepState.timeCount].transpose() << "\n";
  }

  void Walking::timeUpdate()
  {
    if (stepState.phaseChange == DtoL || stepState.phaseChange == DtoR)
    {
      stepState.dsCount = 0;
    }
    else if (stepState.phaseChange == LtoD || stepState.phaseChange == RtoD)
    {
      stepState.ssCount = 0;
    }

    stepState.timeCount++;
    if (stepState.legS == DOUBLE_STANCE)
    {
      stepState.dsCount++;
    }
    else
    {
      if (stepState.ssCount >= stepParam.ssTotalCnt)
      {
        std::cout << "          waiting...\n\n";
        return;
      }
      stepState.ssCount++;
    }
    // std::cout << "stepState cnt: " << stepState.timeCount << ", " << stepState.dsCount << ", " << stepState.ssCount << "\n";
  }

  void Walking::setHipOffset(double_t left, double_t right)
  {
    hipOffsetV[0] = left * Util::TO_RADIAN;
    hipOffsetV[1] = right * Util::TO_RADIAN;
  }

  void Walking::hipOffset()
  {
    double rhythm = (double)stepState.ssCount / stepParam.ssTotalCnt;

    double t1 = 0.2;
    double t2 = 0.7;
    double t3 = 0.9;
    if (stepState.legS == LEFT_STANCE)
    {
      if (rhythm <= t1)
      {
        currHipOffset[0] = rhythm / t1 * hipOffsetV[0];
      }
      if (rhythm > t1 && rhythm <= t2)
      {
        currHipOffset[0] = hipOffsetV[0];
      }
      if (rhythm > t2 && rhythm <= t3)
      {
        currHipOffset[0] = (t3 - rhythm) / (t3 - t2) * hipOffsetV[0];
      }
      // std::cout << "left hipOffset: " << currHipOffset[0] * Util::TO_DEGREE << std::endl;
      float_q_ref[FLOATING_CONFIG_NUM + 1] += currHipOffset[0];
    }
    else if (stepState.legS == RIGHT_STANCE)
    {
      if (rhythm <= t1)
      {
        currHipOffset[1] = rhythm / t1 * hipOffsetV[1];
      }
      if (rhythm > t1 && rhythm <= t2)
      {
        currHipOffset[1] = hipOffsetV[1];
      }
      if (rhythm > t2 && rhythm <= t3)
      {
        currHipOffset[1] = (t3 - rhythm) / (t3 - t2) * hipOffsetV[1];
      }
      // std::cout << "right hipOffset: " << currHipOffset[1] * Util::TO_DEGREE << std::endl;
      float_q_ref[FLOATING_CONFIG_NUM + 7] -= currHipOffset[1];
    }
  }

  void Walking::deHipOffset()
  {
    if (stepState.legS == LEFT_STANCE)
    {
      float_q[FLOATING_CONFIG_NUM + 1] -= currHipOffset[0];
    }
    else if (stepState.legS == RIGHT_STANCE)
    {
      float_q[FLOATING_CONFIG_NUM + 7] += currHipOffset[1];
    }
  }

  void Walking::stepCountUpdate()
  {
    if (stepState.timeCount > (stepParam.ds1TotalCnt + 2) && stepState.dsCount >= stepParam.ds2TotalCnt)
    {
      std::cout << "single step over.\n\n";
      stepState.timeCount = 0;
      stepState.dsCount = 0;
      endState = botState;
      stepCount++;
      if (stepCount > stepCountTarget)
      {
        stepCount = 0;
        running = false;
        lastRunning = running;
        end();
        return;
      }
      std::cout << "stepCount: " << stepCount << "\n";
      // nextStepUpdate();
    }
  }

  void Walking::planner()
  {
  }

  void Walking::torsoBalance()
  {

    double dt = stepParam.dt;
    double H = dcmplanner.TorsoHeightWalk;
    double w = sqrt(9.81 / H);

    Vector3d comstate = botState.CoM.pos.translation();
    Vector3d comvstate = botState.CoM.vel.linear();
    if(fabs(comvstate(0))>0.5){
      comvstate(0) = 0;
    }

    Vector3d cp = comstate + comvstate / (0.5 * w);
    Vector3d cpvel = (comvstate + botState.CoM.acc.linear() / w);




    double k1, k2, k3, k4;
    k1 = 100.;
    k2 = 20.;
    k3 = -10.;
    k4 = -12.;

    double balanceP = 5. * TO_RADIAN;

    // double TorsoP = imuEuler(1)>1.?1.:(imuEuler(1)<-1.?-1.:imuEuler(1));
    // double TorsoPvel = imuGyroInW(1)>2.?2.:(imuGyroInW(1)<-2.?-2.:imuGyroInW(1));
    double TorsoP = imuEuler(1) - balanceP;
    double TorsoPvel = imuGyroInW(1);


    double thetaAcc = k1 * cp(0) + k2 * cpvel(0) + k3 * TorsoP + k4 * TorsoPvel;

    if ((theta > 0.3999 && thetaAcc > 0) || (theta < -0.3999 && thetaAcc < 0))
    {
    }
    else
    {
      thetaVel += thetaAcc * dt;
    }

    thetaVel = thetaVel > 1. ? 1. : (thetaVel < -1. ? -1. : thetaVel);
    theta += thetaVel * dt;
    theta = theta > 0.4 ? 0.4 : (theta < -0.4 ? -0.4 : theta);
    // std::cout<<theta*TO_DEGREE<<"\r";


    if(-0.01<comstate(0) && comstate(0)<-0.01){
    
    }
    else{
      thetafoot = thetafoot*0.99 + 0.015*comstate(0);
    }

    thetafoot = thetafoot > 0.2 ? 0.2 : (thetafoot < -0.2 ? -0.2 : thetafoot);

  }

  void Walking::run()
  {
    struct timeval t_start, t_end;
    gettimeofday(&t_start, NULL);

    stateUpdate();
    if (running == true)
    {
      if (lastRunning == false)
      {
        start();
        nextStepUpdate();
        std::cout << "stepCount: " << stepCount << "\n";
      }
      legPhaseUpdate();
      paramUpdate();
    }

    if (lastRunning == false && running == true)
    {
      enabletorsobalance == false;
    }
    if (stepState.timeCount > (stepParam.ds1TotalCnt + 2) && stepState.dsCount >= stepParam.ds2TotalCnt)
    {
      if (stepCount >= stepCountTarget)
      {
        enabletorsobalance == false;
      }
    }

    if (enabletorsobalance)
    {
      torsoBalance();
    }
    botParam.CoM.pos.rotation() = sva::RotY(theta);
    botParam.rFoot.pos.rotation() = sva::RotY(thetafoot);
    botParam.lFoot.pos.rotation() = sva::RotY(thetafoot);
    
    if (!comIK.computerJointValue(float_q_ref, botParam.CoM.pos, botParam.lFoot.pos, botParam.rFoot.pos))
    {
      exit_fun();
      return;
    }

    if (running == true)
    {
      hipOffset();
      timeUpdate();
      stepCountUpdate();
    }
    lastRunning = running;

    gettimeofday(&t_end, NULL);
    double time = (t_end.tv_sec + t_end.tv_usec * 1e-6) - (t_start.tv_sec + t_start.tv_usec * 1e-6);
    // std::cout << "time: " << time * 1000.0 << "\n";
  }
}; // namespace Walking