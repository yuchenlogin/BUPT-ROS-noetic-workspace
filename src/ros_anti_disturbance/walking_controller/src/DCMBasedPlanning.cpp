#include <fstream>
#include <sstream>
#include <string>
#include <cstdio>
#include <unistd.h>

#include <sys/time.h>
#include <iostream>
#include <iomanip>
#include <limits.h>

#include "util/Util.h"
#include "DCMBasedPlanning.h"
// #include "CoMIK.h"

using namespace DCM;
using namespace std;

DCMBasedPlanning *DCMBasedPlanning::m_UniqueInstance = new DCMBasedPlanning();

DCMBasedPlanning::DCMBasedPlanning()
{
  timeStep = 0.001; //s
  T_step = 0.6;
  T_dsBegin = 0.06;
  T_dsEnd = 0.06;

  stepLength = 0.0;
  stepWidth = 0.065; //m
  swingH = 0.03;     //m

  TorsoHeightWalk = 0.345; //torso height at walking

  ReplanStepNum = 3;
  lFootHoffset = 0.;

  CoMH = TorsoHeightWalk; //com height

  omega_x = 0.75 * std::sqrt(Gravity / CoMH); // sqrt(g/H)
  omega_y = 0.8 * std::sqrt(Gravity / CoMH);

  ss_cop_len = 0.;
}

DCMBasedPlanning::~DCMBasedPlanning()
{
}

void DCMBasedPlanning::setTimeStep(double_t s)
{
  timeStep = s;
}

void DCMBasedPlanning::clear()
{
  cprefvec.clear();
  coprefvec.clear();
  comRefTrajvec.clear();
  legStatevec.clear();
  WrightDistvec.clear();

  lFootRefTrajvec.clear();
  rFootRefTrajvec.clear();

  CoMavec.clear();
  CoMVvec.clear();
  CoMvec.clear();
}

void DCMBasedPlanning::plannerInit(std::vector<Vector2d> FootPrint)
{
  RobotState robot;
  robot.cp = Vector2d(0., 0.);
  robot.cpv = Vector2d(0., 0.);
  robot.com = Vector3d(0., 0., CoMH);
  robot.comv = Vector3d(0., 0., 0.);
  robot.coma = Vector3d(0., 0., 0.);
  robot.timecount = 0;
  robot.legs = rightStance;
  robot.legs_old = doublesupport;

  int fpsize = FootPrint.size();
  std::vector<Vector2d> footprint = FootPrint;
  footprint[0] = (footprint[0] + footprint[1]) / 2;
  footprint[fpsize - 1] = (footprint[fpsize - 1] + footprint[fpsize - 2]) / 2;

  ReplanStepNum = fpsize;
  computeCoPTraj(footprint, &robot);
  computelegSTraj(footprint, &robot);
  optcopref = coprefvec;
  OptimalTrajInit(&robot);
  computeOptimalTraj(&robot);
  clear();
}

void DCMBasedPlanning::plan(std::vector<Vector2d> FootPrint)
{
  clear();

  RobotState robot;
  robot.cp = Vector2d(0., 0.);
  robot.cpv = Vector2d(0., 0.);
  robot.com = Vector3d(0., 0., CoMH);
  robot.comv = Vector3d(0., 0., 0.);
  robot.coma = Vector3d(0., 0., 0.);
  robot.timecount = 0;
  robot.legs = rightStance;
  robot.legs_old = doublesupport;

  int fpsize = FootPrint.size();
  std::vector<Vector2d> footprint = FootPrint;
  footprint[0] = (footprint[0] + footprint[1]) / 2;
  footprint[fpsize - 1] = (footprint[fpsize - 1] + footprint[fpsize - 2]) / 2;

  ReplanStepNum = fpsize;
  computeCoPTraj(footprint, &robot);
  computelegSTraj(footprint, &robot);
  optcopref = coprefvec;
  // OptimalTrajInit(&robot);
  computeOptimalTraj(&robot);
}

void DCMBasedPlanning::computeOptimalFootprint(RobotState *robot)
{
  // Prediction horizon
  int N = ReplanStepNum; //compute N footprint, N+1 robot state

  //Discrete time model
  double Zcom = CoMH;
  double omega = omega_x; //std::sqrt(Gravity / Zcom);
  double stepF = stepLength;
  double stepD = stepWidth;

  double dt = T_step;
  double ewt = exp(omega * dt);
  double e_wt = exp(-omega * dt);
  MatrixXd I2x2 = MatrixXd::Identity(2, 2);

  MatrixXd Adt(2, 2);
  Adt << ewt + e_wt, (ewt - e_wt) / omega,
      omega * (ewt - e_wt), ewt + e_wt;
  Adt = 0.5 * Adt;
  MatrixXd Adtxy = kroneckerProduct(Adt, I2x2);

  MatrixXd bdt(2, 1);
  bdt << 1 - 0.5 * (ewt + e_wt), 0.5 * omega * (e_wt - ewt);
  MatrixXd bdtxy = kroneckerProduct(bdt, I2x2);

  if (robot->legs == doublesupport && robot->timecount < (T_step / timeStep) / 2)
  {
    dt = T_dsEnd - robot->timecount * timeStep;
  }
  else
  {
    dt = T_step - robot->timecount * timeStep;
  }
  ewt = exp(omega * dt);
  e_wt = exp(-omega * dt);

  MatrixXd Adtx0(2, 2);
  Adtx0 << ewt + e_wt, (ewt - e_wt) / omega,
      omega * (ewt - e_wt), ewt + e_wt;
  Adtx0 = 0.5 * Adtx0;
  MatrixXd Adtx0xy = kroneckerProduct(Adtx0, I2x2);

  MatrixXd bdtx0(2, 1);
  bdtx0 << 1 - 0.5 * (ewt + e_wt), 0.5 * omega * (e_wt - ewt);
  MatrixXd bdtx0xy = kroneckerProduct(bdtx0, I2x2);

  int nx = bdtxy.rows(), nu = bdtxy.cols();

  double v0xref = omega * stepF * (1 + cosh(omega * dt)) / sinh(omega * dt);
  double v0yref = -omega * stepD * (1 - cosh(omega * dt)) / sinh(omega * dt);

  double remaint = T_step - robot->timecount * timeStep;
  double comx0 = (robot->com(0) - robot->cop(0)) * cosh(omega * remaint) + robot->comv(0) / omega * sinh(omega * remaint);
  double comv0 = (robot->com(0) - robot->cop(0)) * omega * sinh(omega * remaint) + robot->comv(0) * cosh(omega * remaint);

  MatrixXd umax(2, 1);
  umax << 100, 100;
  MatrixXd umin(2, 1);
  umin << -100, -100;

  MatrixXd xmax(2, 1);
  xmax << 1000, 1000;
  MatrixXd xmin(2, 1);
  xmin << -1000, -1000;

  MatrixXd x0(4, 1);
  x0 << robot->com(0), robot->com(1), robot->comv(0), robot->comv(1);
  MatrixXd xrN(4, 1);
  xrN << robot->cop(0) + 6 * stepF, 0., 0., 0.; //TODO: 前进时候的xrN参考值需要计算

  //fp cost
  SparseMatrix<double> Afp(nu * N, nx * (N + 1) + nu * N);
  Afp.setZero();
  // Afp.insert(0, nx*(N+1)) = 1;
  // Afp.insert(1, nx*(N+1)+1) = 1;
  for (int k = 1; k < N; k++)
  {
    Afp.insert(nu * k, nx * (N + 1) + nu * (k - 1)) = -1;
    Afp.insert(nu * k, nx * (N + 1) + nu * k) = 1;
    Afp.insert(nu * k + 1, nx * (N + 1) + nu * (k - 1) + 1) = -1;
    Afp.insert(nu * k + 1, nx * (N + 1) + nu * k + 1) = 1;
  }

  MatrixXd bfp(nu * N, 1);
  bfp.setZero();

  //TODO: footprint differences dp 处理开始步和结束步
  // bfp(0) = robot->cop(0);
  // bfp(1) = robot->cop(1);
  if (robot->legs == doublesupport)
  {
    bfp(nu * N - 2) = stepF;
    bfp(nu * N - 1) = stepD * pow(-1, (robot->legs_old + 1) / 2 + N);
    for (int i = 1; i < N - 1; i++)
    {
      bfp(nu * i) = stepF;
      bfp(nu * i + 1) = 2 * stepD * pow(-1, i + 1 + (robot->legs_old + 1) / 2);
    }
  }
  else
  {
    bfp(nu * N - 2) = stepF;
    bfp(nu * N - 1) = stepD * pow(-1, (robot->legs + 1) / 2 + N);
    for (int i = 1; i < N - 1; i++)
    {
      bfp(nu * i) = stepF;
      bfp(nu * i + 1) = 2 * stepD * pow(-1, i + 1 + (robot->legs + 1) / 2);
    }
  }

  SparseMatrix<double> AxN(nx, nx * (N + 1) + nu * N);
  AxN.setZero();
  for (int i = 2; i < nx; i++)
  {
    AxN.insert(i, N * nx + i) = 1;
  }
  MatrixXd bxN = xrN;

  SparseMatrix<double> Au0x(1, nx * (N + 1) + nu * N);
  Au0x.setZero();
  Au0x.insert(0, nx * (N + 1)) = 1;
  MatrixXd bu0x = robot->cop.segment(0, 1);

  SparseMatrix<double> Au0y(1, nx * (N + 1) + nu * N);
  Au0y.setZero();
  Au0y.insert(0, nx * (N + 1) + 1) = 1;
  MatrixXd bu0y = robot->cop.segment(1, 1);

  double wfp = 10;
  double wxN = 5;
  double wu0x = 20;
  double wu0y = 10;

  int costrows = Afp.rows() + AxN.rows() + Au0x.rows() + Au0y.rows();
  int costcols = Afp.cols();

  SparseMatrix<double, RowMajor> AcostSparse(costrows, costcols);
  AcostSparse.topRows(Afp.rows()) = wfp * Afp;
  AcostSparse.middleRows(Afp.rows(), AxN.rows()) = wxN * AxN;
  AcostSparse.middleRows(Afp.rows() + AxN.rows(), Au0x.rows()) = wu0x * Au0x;
  AcostSparse.bottomRows(Au0y.rows()) = wu0y * Au0y;

  MatrixXd bcost(costrows, 1);
  bcost << wfp * bfp, wxN * bxN, wu0x * bu0x, wu0y * bu0y;

  SparseMatrix<double> Psparse = AcostSparse.transpose() * AcostSparse;
  MatrixXd q = -AcostSparse.transpose() * bcost;

  SparseMatrix<double> tu = Psparse.triangularView<Eigen::Upper>();

  //equality constraints 没有使用第一行约束x0
  SparseMatrix<double> Ax(nx * (N + 1), nx * (N + 1));
  std::vector<Triplet<double>> Axtriplet;
  for (int i = 0; i < nx; i++)
  {
    for (int j = 0; j < nx; j++)
    {
      Axtriplet.emplace_back(nx + i, j, Adtx0xy(i, j));
    }
  }
  for (int k = 1; k < N; k++)
  {
    for (int i = 0; i < nx; i++)
    {
      for (int j = 0; j < nx; j++)
      {
        Axtriplet.emplace_back(nx + k * nx + i, k * nx + j, Adtxy(i, j));
      }
    }
  }
  for (int k = nx * 1; k < nx * (N + 1); k++)
  {
    Axtriplet.emplace_back(k, k, -1);
  }
  Ax.setFromTriplets(Axtriplet.begin(), Axtriplet.end());

  SparseMatrix<double> Bu(nx * (N + 1), nu * N);
  std::vector<Triplet<double>> Butriplet;
  for (int i = 0; i < nx; i++)
  {
    for (int j = 0; j < nu; j++)
    {
      Butriplet.emplace_back(nx + i, j, bdtx0xy(i, j));
    }
  }
  for (int k = 1; k < N; k++)
  {
    for (int i = 0; i < nx; i++)
    {
      for (int j = 0; j < nu; j++)
      {
        Butriplet.emplace_back(nx + k * nx + i, k * nu + j, bdtxy(i, j));
      }
    }
  }
  Bu.setFromTriplets(Butriplet.begin(), Butriplet.end());

  SparseMatrix<double, ColMajor> Aeq(Ax.rows(), Ax.cols() + Bu.cols());
  Aeq.leftCols(Ax.cols()) = Ax;
  Aeq.rightCols(Bu.cols()) = Bu;

  // upper lower bound
  SparseMatrix<double> Asparse((N + 1) * nx + N * nu, (N + 1) * nx + N * nu);
  std::vector<Triplet<double>> Atriplet;
  for (int i = 0; i < (N + 1) * nx + N * nu; i++)
  {
    Atriplet.emplace_back(i, i, 1);
  }
  Asparse.setFromTriplets(Atriplet.begin(), Atriplet.end());

  //cop constraint
  SparseMatrix<double> Astancecop(nu * 2, nx * (N + 1) + nu * N);
  Astancecop.setZero();
  Astancecop.insert(0, nx * (N + 1)) = 1;
  Astancecop.insert(1, nx * (N + 1) + 1) = 1;
  Astancecop.insert(2, nx * (N + 1) + 2) = 1;
  Astancecop.insert(3, nx * (N + 1) + 3) = 1;

  SparseMatrix<double, RowMajor> AsparseAll(Aeq.rows() + Asparse.rows() + Astancecop.rows(), Aeq.cols());
  AsparseAll.topRows(Aeq.rows()) = Aeq;
  AsparseAll.middleRows(Aeq.rows(), Asparse.rows()) = Asparse;
  AsparseAll.bottomRows(Astancecop.rows()) = Astancecop;

  std::vector<c_float> qvec, lvec, uvec;
  for (int i = 0; i < q.size(); i++)
  {
    qvec.push_back(q(i));
  }

  uvec.insert(uvec.begin(), Aeq.rows(), 0);
  uvec.insert(uvec.end(), (N + 1) * nx, xmax(0));
  uvec.insert(uvec.end(), N * nu, umax(0));
  uvec[Aeq.rows() + 0] = x0(0); //init state constraint
  uvec[Aeq.rows() + 1] = x0(1);
  uvec[Aeq.rows() + 2] = x0(2);
  uvec[Aeq.rows() + 3] = x0(3);

  lvec.insert(lvec.begin(), Aeq.rows(), 0);
  lvec.insert(lvec.end(), (N + 1) * nx, xmin(0));
  lvec.insert(lvec.end(), N * nu, umin(0));
  lvec[Aeq.rows() + 0] = x0(0);
  lvec[Aeq.rows() + 1] = x0(1);
  lvec[Aeq.rows() + 2] = x0(2);
  lvec[Aeq.rows() + 3] = x0(3);

  //stance leg cop constraint
  if (robot->legs != doublesupport)
  {
    uvec.insert(uvec.end(), robot->cop(0) + 0.07);
    uvec.insert(uvec.end(), robot->cop(1) + 0.05);
    uvec.insert(uvec.end(), robot->cop(0) + 0.5);
    uvec.insert(uvec.end(), robot->cop(1) + 0.6);

    lvec.insert(lvec.end(), robot->cop(0) - 0.07);
    lvec.insert(lvec.end(), robot->cop(1) - 0.05);
    lvec.insert(lvec.end(), robot->cop(0) - 0.5);
    lvec.insert(lvec.end(), robot->cop(1) - 0.6);
  }
  else
  {
    if (robot->legs_old == leftStance)
    {
      uvec.insert(uvec.end(), robot->cop1(0) + 0.13 + stepF);
      uvec.insert(uvec.end(), robot->cop1(1) + 0.1);
      uvec.insert(uvec.end(), robot->cop2(0) + 0.13);
      uvec.insert(uvec.end(), robot->cop2(1) + 0.1 + 2 * stepD);

      lvec.insert(lvec.end(), robot->cop1(0) - 0.13);
      lvec.insert(lvec.end(), robot->cop1(1) - 0.1 - 2 * stepD);
      lvec.insert(lvec.end(), robot->cop2(0) - 0.13 - stepF);
      lvec.insert(lvec.end(), robot->cop2(1) - 0.1);
    }
    if (robot->legs_old == rightStance)
    {
      uvec.insert(uvec.end(), robot->cop1(0) + 0.13 + stepF);
      uvec.insert(uvec.end(), robot->cop1(1) + 0.05 + 2 * stepD);
      uvec.insert(uvec.end(), robot->cop2(0) + 0.13);
      uvec.insert(uvec.end(), robot->cop2(1) + 0.05);

      lvec.insert(lvec.end(), robot->cop1(0) - 0.13);
      lvec.insert(lvec.end(), robot->cop1(1) - 0.05);
      lvec.insert(lvec.end(), robot->cop2(0) - 0.13 - stepF);
      lvec.insert(lvec.end(), robot->cop2(1) - 0.05 - 2 * stepD);
    }
  }

  //solution
  OSQPTasks::TaskSolver FPSolver;
  FPSolver.update(tu, qvec, AsparseAll, lvec, uvec);

  VectorXd solution = FPSolver.solver();

  {
    cout << "timecount: " << robot->timecount << endl;
    cout << "legs: " << robot->legs << endl;
    cout << "comx0: " << robot->com.transpose() << endl;
    cout << "comv0: " << robot->comv.transpose() << endl;
    cout << "x0 v0: " << solution.segment(0, 4).transpose() << endl;
    cout << "cop: " << robot->cop.transpose() << endl;
    cout << bfp.transpose() << endl;
    cout << solution.segment(nx * (N + 1), nu * N).transpose() << endl;

    MatrixXd x0(4, 1);
    x0 << solution.segment(0, 4);
    MatrixXd u0(2, 1);
    u0 << solution.segment(nx * (N + 1), 2);

    MatrixXd x1;
    x1 = Adtx0xy * x0 + bdtx0xy * u0;
    // cout<<x1.transpose()<<endl;
  }

  for (int i = 0; i < N; i++)
  {
    Vector2d footprint(solution(i * nu + nx * (N + 1)), solution(i * nu + 1 + nx * (N + 1)));
    Footprint.push_back(footprint);
    if (footprint.norm() > 100)
    {
      cout << "footprint solver failed" << endl;
      exit(0);
    }
  }
}

void DCMBasedPlanning::computelegSTraj(std::vector<Vector2d> footprint, RobotState *robot)
{

  if (robot->legs == leftStance)
  {
    for (int i = 0; i < (T_step - T_dsBegin) / timeStep - robot->timecount; i++)
    {
      legStatevec.push_back(robot->legs);
    }
    for (int i = 0; i < (T_dsBegin + T_dsEnd) / timeStep; i++)
    {
      legStatevec.push_back(doublesupport);
    }

    for (int i = 0; i < footprint.size() - 2; i++)
    {
      if (i % 2 == 0)
      {
        for (int i = 0; i < (T_step - T_dsBegin - T_dsEnd) / timeStep; i++)
        {
          legStatevec.push_back(rightStance);
        }
      }
      else
      {
        for (int i = 0; i < (T_step - T_dsBegin - T_dsEnd) / timeStep; i++)
        {
          legStatevec.push_back(leftStance);
        }
      }
      for (int i = 0; i < (T_dsBegin + T_dsEnd) / timeStep; i++)
      {
        legStatevec.push_back(doublesupport);
      }
    }
  }
  else if (robot->legs == rightStance)
  {
    for (int i = 0; i < (T_step - T_dsBegin) / timeStep - robot->timecount; i++)
    {
      legStatevec.push_back(robot->legs);
    }
    for (int i = 0; i < (T_dsBegin + T_dsEnd) / timeStep; i++)
    {
      legStatevec.push_back(doublesupport);
    }

    for (int i = 0; i < footprint.size() - 2; i++)
    {
      if (i % 2 == 0)
      {
        for (int i = 0; i < (T_step - T_dsBegin - T_dsEnd) / timeStep; i++)
        {
          legStatevec.push_back(leftStance);
        }
      }
      else
      {
        for (int i = 0; i < (T_step - T_dsBegin - T_dsEnd) / timeStep; i++)
        {
          legStatevec.push_back(rightStance);
        }
      }
      for (int i = 0; i < (T_dsBegin + T_dsEnd) / timeStep; i++)
      {
        legStatevec.push_back(doublesupport);
      }
    }
  }
  else
  {
    int count;
    if (robot->timecount > T_step / timeStep / 2)
      count = (T_step + T_dsEnd) / timeStep - robot->timecount;
    else
      count = T_dsEnd / timeStep - robot->timecount;

    for (int i = 0; i < count; i++)
    {
      legStatevec.push_back(robot->legs);
    }
    if (robot->legs_old == leftStance)
    {
      for (int i = 0; i < footprint.size() - 2; i++)
      {
        if (i % 2 == 0)
        {
          for (int i = 0; i < (T_step - T_dsBegin - T_dsEnd) / timeStep; i++)
          {
            legStatevec.push_back(rightStance);
          }
        }
        else
        {
          for (int i = 0; i < (T_step - T_dsBegin - T_dsEnd) / timeStep; i++)
          {
            legStatevec.push_back(leftStance);
          }
        }
        for (int i = 0; i < (T_dsBegin + T_dsEnd) / timeStep; i++)
        {
          legStatevec.push_back(doublesupport);
        }
      }
    }

    if (robot->legs_old == rightStance)
    {
      for (int i = 0; i < footprint.size() - 2; i++)
      {
        if (i % 2 == 0)
        {
          for (int i = 0; i < (T_step - T_dsBegin - T_dsEnd) / timeStep; i++)
          {
            legStatevec.push_back(leftStance);
          }
        }
        else
        {
          for (int i = 0; i < (T_step - T_dsBegin - T_dsEnd) / timeStep; i++)
          {
            legStatevec.push_back(rightStance);
          }
        }
        for (int i = 0; i < (T_dsBegin + T_dsEnd) / timeStep; i++)
        {
          legStatevec.push_back(doublesupport);
        }
      }
    }
  }
  int competelegs = ReplanStepNum * T_step / timeStep - legStatevec.size();
  for (int i = 0; i < competelegs; i++)
  {
    legStatevec.push_back(doublesupport);
  }
}

void DCMBasedPlanning::computeCoPTraj(std::vector<Vector2d> footprint, RobotState *robot)
{

  coprefvec.clear();

  if (robot->legs != doublesupport)
  {
    if (robot->timecount < (int)(T_step / timeStep))
    {
      for (int i = 0; i < T_step / timeStep - robot->timecount; i++)
      {
        coprefvec.push_back(footprint[0]);
      }
      footprint.erase(footprint.begin());
    }
  }
  if (robot->legs == doublesupport)
  {
    if (robot->timecount > (int)(T_step / timeStep) / 2)
    {
      for (int i = 0; i < T_step / timeStep - robot->timecount; i++)
      {
        coprefvec.push_back(footprint[0]);
      }
      footprint.erase(footprint.begin());
    }
    else
    {
      for (int i = 0; i < T_dsEnd / timeStep - robot->timecount; i++)
      {
        coprefvec.push_back(footprint[0]);
      }
      footprint.erase(footprint.begin());

      for (int i = 0; i < (T_step - T_dsEnd) / timeStep; i++)
      {
        coprefvec.push_back(footprint[0]);
      }
      footprint.erase(footprint.begin());
    }
  }

  int stepnum = footprint.size();

  std::vector<Vector2d> copref;
  // copref.push_back((footprint[0]+footprint[1])/2);
  for (int i = 0; i < stepnum; i++)
  {
    copref.push_back(footprint[i]);
  }
  // copref.push_back((footprint[stepnum-1]+footprint[stepnum-2])/2);

  int timecount = 0;
  int stepcount = 0;

  Vector2d copref_iniDS = copref[0];
  Vector2d copref_endDS = copref[0];

  //first step  ss phase
  for (int i = 0; i < round((T_step - T_dsBegin) / timeStep); i++)
  {
    coprefvec.push_back(copref[stepcount]);
    timecount++;
  }

  //first step  ds phase
  copref_iniDS = copref[stepcount + 1];
  copref_endDS = copref[stepcount + 1] + Vector2d(ss_cop_len, 0.);
  for (int i = 0; i < round((T_dsBegin + T_dsEnd) / timeStep); i++)
  {
    double t = i * timeStep / (T_dsBegin + T_dsEnd);
    Vector2d cop = (1 - t) * copref[stepcount] + t * copref_iniDS;
    coprefvec.push_back(cop);
    timecount++;
  }
  stepcount++;

  //middle step
  while (stepcount < stepnum - 2)
  {
    //ss phase
    for (int i = 0; i < round((T_step - T_dsBegin - T_dsEnd) / timeStep); i++)
    {
      double t = i * timeStep / (T_step - T_dsBegin - T_dsEnd);
      Vector2d cop = (1 - t) * copref_iniDS + t * copref_endDS;
      coprefvec.push_back(cop);
      timecount++;
    }
    //ds phase
    copref_iniDS = copref[stepcount + 1] + Vector2d(-ss_cop_len, 0.);
    for (int i = 0; i < round((T_dsBegin + T_dsEnd) / timeStep); i++)
    {
      double t = i * timeStep / (T_dsBegin + T_dsEnd);
      Vector2d cop = (1 - t) * copref_endDS + t * copref_iniDS;
      coprefvec.push_back(cop);
      timecount++;
    }
    copref_endDS = copref[stepcount + 1] + Vector2d(ss_cop_len, 0.);
    stepcount++;
  }

  //second last step ss phase
  for (int i = 0; i < round((T_step - T_dsBegin - T_dsEnd) / timeStep); i++)
  {
    double t = i * timeStep / (T_step - T_dsBegin - T_dsEnd);
    Vector2d cop = (1 - t) * copref_iniDS + t * copref[stepcount];
    coprefvec.push_back(cop);
    timecount++;
  }
  //last step ds phase
  for (int i = 0; i < round((T_dsBegin + T_dsEnd) / timeStep); i++)
  {
    double t = i * timeStep / (T_dsBegin + T_dsEnd);
    Vector2d cop = (1 - t) * copref[stepcount] + t * copref.back();
    coprefvec.push_back(cop);
    timecount++;
  }
  stepcount++;

  //last step ss phase
  for (int i = 0; i < round((T_step - T_dsBegin) / timeStep); i++)
  {
    coprefvec.push_back(copref.back());
    timecount++;
  }
  /*
	if(robot->legs!=doublesupport){
		int remaincop = coprefvec.size()-(ReplanStepNum-1)*T_step/timeStep;
		int remainstancecop = remaincop-T_dsBegin/timeStep;
		Vector2d cop1 = coprefvec[0];
		Vector2d cop2 = coprefvec[remaincop];
		for(int i=0;i<(T_dsBegin+T_dsEnd)/timeStep;i++){
			double t=i/((T_dsBegin+T_dsEnd)/timeStep);
			Vector2d cop = cop1*(1-t)+cop2*t;
			coprefvec[i+remainstancecop] = cop;
		}
	}
	else{
		int remainDScop;
		int robottimecount = ReplanStepNum*T_step/timeStep-coprefvec.size();
		if(robottimecount > T_step/timeStep/2)
			remainDScop=(T_step+T_dsEnd)/timeStep - robottimecount;
		else
			remainDScop=T_dsEnd/timeStep - robottimecount;
		Vector2d cop1 = coprefvec[0];
		Vector2d cop2 = coprefvec[remainDScop+1];
		for(int i=0;i<remainDScop;i++){
			double t=(((T_dsBegin+T_dsEnd)/timeStep)-remainDScop+i)/((T_dsBegin+T_dsEnd)/timeStep);
			Vector2d cop = cop1+(cop2-cop1)*t;
			coprefvec[i] = cop;
		}
		cout<<"remainDScop: "<<remainDScop<<endl;
	}
*/
  //compete coprefvec point to ReplanStepNum steps
  int competecop = ReplanStepNum * T_step / timeStep - coprefvec.size();
  for (int i = 0; i < competecop; i++)
  {
    coprefvec.push_back(copref.back());
    timecount++;
  }
}

void DCMBasedPlanning::computeOptimalTraj(RobotState *robot)
{

  struct timeval start;
  struct timeval time1, time2, time3, time4, time5, time6, time7;
  struct timeval end;
  gettimeofday(&start, NULL);

  int N = ReplanStepNum * T_step / timeStep; //coprefvec.size();
  int nx = 6, nu = 2;

  MatrixXd x0(6, 1);
  x0 << robot->com(0), robot->com(1), robot->comv(0), robot->comv(1), 0, 0;
  MatrixXd xrN(6, 1);
  xrN << optcopref[N - 1](0), optcopref[N - 1](1), 0, 0, 0, 0;

  MatrixXd bzmp(nu * (N + 1), 1);
  for (int i = 0; i < N; i++)
  {
    bzmp.block(nu * i, 0, 2, 1) = optcopref[i];
  }
  bzmp.block(nu * N, 0, 2, 1) = optcopref[N - 1];
  MatrixXd bxN = xrN;

  MatrixXd bx0 = x0;

  MatrixXd beq(nx * (N + 1), 1);
  beq.setZero();

  double wzmp = 1;
  double wxN = 10;
  double wx0 = 10;
  double weq = 100;

  int costrows = bzmp.rows() + bxN.rows() + bx0.rows() + beq.rows();

  MatrixXd bcost(costrows, 1);
  bcost << wzmp * bzmp, wxN * bxN, wx0 * bx0, weq * beq;
  MatrixXd q = -Acostglobal.transpose() * bcost;

  std::vector<c_float> qvec;
  for (int i = 0; i < q.size(); i++)
  {
    qvec.push_back(q(i));
  }

  // DynSolver.updateq(qvec);
  // VectorXd solution = DynSolver.solver();

  OSQPTasks::TaskSolver dynsolver;
  dynsolver.update(Pglobal, qvec, Aglobal, lvecglobal, uvecglobal);
  VectorXd solution = dynsolver.solver();

  std::vector<Vector3d> velvec;
  std::vector<Vector3d> comvec;
  std::vector<Vector2d> cpvec;

  copvec.clear();

  Vector2d cop(0., 0.);
  Vector3d vel(x0(2), x0(3), 0.);
  Vector3d compos(x0(0), x0(1), CoMH);
  Vector2d cp(0., 0.);

  MatrixXd cdt(1, 3);
  cdt << 1., 0, -CoMH / Gravity;

  for (int i = 0; i < N; i++)
  {
    VectorXd cop0 = cdt * Vector3d(solution(i * 6), solution(i * 6 + 2), solution(i * 6 + 4));
    VectorXd cop1 = cdt * Vector3d(solution(i * 6 + 1), solution(i * 6 + 3), solution(i * 6 + 5));
    cop = Vector2d(cop0(0), cop1(0));
    copvec.push_back(cop);

    vel += Vector3d(solution(i * 6 + 4), solution(i * 6 + 5), 0.) * timeStep;
    velvec.push_back(vel);

    compos += Vector3d(solution(i * 6 + 2), solution(i * 6 + 3), 0.) * timeStep;
    comvec.push_back(compos);

    cp = compos.segment(0, 2) + vel.segment(0, 2) / omega_x;
    cpvec.push_back(cp);
  }
  cprefvec = cpvec;
  CoMVvec = velvec;
  CoMvec = comvec;

  cprefvec.push_back(cpvec.back());
  CoMVvec.push_back(velvec.back());
  CoMvec.push_back(comvec.back());


  cout << fixed << setprecision(5);
  std::ofstream copstream("src_data/copstream.txt");
  for (int i = 0; i < N; i++)
  {
    copstream
        << coprefvec[i](0) << " , " << coprefvec[i](1) << " , "
        << copvec[i](0) << " , " << copvec[i](1) << " , "
        << optcopref[i](0) << " , " << optcopref[i](1) << " , "

        // <<CoMvec[i](0)<<" , "<<CoMvec[i](1)<<" , "
        // <<CoMVvec[i](0)<<" , "<<CoMVvec[i](1)<<" , "
        // <<cprefvec[i](0)<<" , "<<cprefvec[i](1)<<" , "

        << solution(i * 6) << " , " << solution(i * 6 + 1) << " , "
        << solution(i * 6 + 2) << " , " << solution(i * 6 + 3) << " , "
        << solution(i * 6 + 4) << " , " << solution(i * 6 + 5) << " , "

        << comvec[i](0) << " , " << comvec[i](1) << " , "
        << velvec[i](0) << " , " << velvec[i](1) << " , "
        << cpvec[i](0) << " , " << cpvec[i](1) << " , "

        << 0.05 * legStatevec[i]
        << endl;
  }

  copstream.close();

  // cout<<"comsolved "<<solution.segment(0,2).transpose()<<endl;
  // cout<<"comadd "<<comvec[0].transpose()<<endl;

  gettimeofday(&end, NULL);
  double usedtimeall = (end.tv_sec - start.tv_sec) * 1000. + (end.tv_usec - start.tv_usec) / 1000.;
  // cout<<"usedtimeall(ms): "<<usedtimeall<<endl;
}

void DCMBasedPlanning::GetSolver(DCMBasedPlanning *dcmplanner)
{

  this->Acostglobal = dcmplanner->Acostglobal;
  this->Pglobal = dcmplanner->Pglobal;
  this->Aglobal = dcmplanner->Aglobal;
  this->lvecglobal = dcmplanner->lvecglobal;
  this->uvecglobal = dcmplanner->uvecglobal;
}

void DCMBasedPlanning::OptimalTrajInit(RobotState *robot)
{

  // Prediction horizon
  int N = ReplanStepNum * T_step / timeStep; //coprefvec.size();

  //Discrete time model
  double dt = timeStep;
  double Zcom = CoMH;
  double omega = omega_x;

  MatrixXd I2x2 = MatrixXd::Identity(2, 2);

  MatrixXd Adt(3, 3);
  Adt << 1., dt, pow(dt, 2) / 2,
      0., 1., dt,
      0., 0., 1.;
  MatrixXd Adtxy = kroneckerProduct(Adt, I2x2);

  MatrixXd bdt(3, 1);
  bdt << pow(dt, 3) / 6, pow(dt, 2) / 2, dt;
  // bdt<<0., 0., dt;
  MatrixXd bdtxy = kroneckerProduct(bdt, I2x2);

  // MatrixXd cdt(1, 3);
  // cdt << 1., 0, -1/(omega*omega);
  // MatrixXd cdtxy = kroneckerProduct(cdt, I2x2);

  // MatrixXd cdtx(1, 3);
  // cdtx << 1., 0, -1/(omega_x*omega_x);
  // MatrixXd cdty(1, 3);
  // cdty << 1., 0, -1/(omega_y*omega_y);
  MatrixXd cdtxy(2, 6);
  cdtxy << 1., 0., 0., 0., -1 / (omega_x * omega_x), 0.,
      0., 1., 0., 0., 0., -1 / (omega_y * omega_y);

  int nx = bdtxy.rows(), nu = bdtxy.cols();

  MatrixXd umax(2, 1);
  umax << 100, 100;
  MatrixXd umin(2, 1);
  umin << -100, -100;

  MatrixXd xmax(6, 1);
  xmax << 1000, 1000, 1000, 1000, 1000, 1000;
  MatrixXd xmin(6, 1);
  xmin << -1000, -1000, -1000, -1000, -1000, -1000;

  MatrixXd x0(6, 1);
  x0 << robot->com(0), robot->com(1), robot->comv(0), robot->comv(1), 0, 0;
  MatrixXd xrN(6, 1);
  xrN << optcopref[N - 1](0), optcopref[N - 1](1), 0, 0, 0, 0;

  MatrixXd bzmp(nu * (N + 1), 1);
  for (int i = 0; i < N; i++)
  {
    bzmp.block(nu * i, 0, 2, 1) = optcopref[i];
  }
  bzmp.block(nu * N, 0, 2, 1) = optcopref[N - 1];

  MatrixXd bxN = xrN;

  MatrixXd bx0 = x0;

  MatrixXd beq(nx * (N + 1), 1);
  beq.setZero();

  SparseMatrix<double> Azmp(nu * (N + 1), nx * (N + 1) + nu * N);
  Azmp.setZero();
  for (int k = 0; k < N + 1; k++)
  {
    for (int i = 0; i < nu; i++)
    {
      for (int j = 0; j < nx; j++)
      {
        Azmp.insert(k * nu + i, k * nx + j) = cdtxy(i, j);
      }
    }
  }

  //remove ddx as cost
  SparseMatrix<double> AxN(nx, nx * (N + 1) + nu * N);
  AxN.setZero();
  for (int i = 0; i < nx - 2; i++)
  {
    AxN.insert(i, N * nx + i) = 1;
  }

  SparseMatrix<double> Ax0(nx, nx * (N + 1) + nu * N);
  Ax0.setZero();
  for (int i = 0; i < nx - 2; i++)
  {
    Ax0.insert(i, i) = 1;
  }

  SparseMatrix<double> Ax(nx * (N + 1), nx * (N + 1));
  std::vector<Triplet<double>> Axtriplet;
  for (int k = 0; k < N; k++)
  {
    for (int i = 0; i < nx; i++)
    {
      for (int j = 0; j < nx; j++)
      {
        Axtriplet.emplace_back(nx + k * nx + i, k * nx + j, Adtxy(i, j));
      }
    }
  }
  for (int k = nx * 1; k < nx * (N + 1); k++)
  {
    Axtriplet.emplace_back(k, k, -1);
  }
  Ax.setFromTriplets(Axtriplet.begin(), Axtriplet.end());

  SparseMatrix<double> Bu(nx * (N + 1), nu * N);
  std::vector<Triplet<double>> Butriplet;
  for (int k = 0; k < N; k++)
  {
    for (int i = 0; i < nx; i++)
    {
      for (int j = 0; j < nu; j++)
      {
        Butriplet.emplace_back(nx + k * nx + i, k * nu + j, bdtxy(i, j));
      }
    }
  }
  Bu.setFromTriplets(Butriplet.begin(), Butriplet.end());

  SparseMatrix<double, ColMajor> Aeq(Ax.rows(), Ax.cols() + Bu.cols());
  Aeq.leftCols(Ax.cols()) = Ax;
  Aeq.rightCols(Bu.cols()) = Bu;

  double wzmp = 1;
  double wxN = 10;
  double wx0 = 10;
  double weq = 100;

  int costrows = Azmp.rows() + AxN.rows() + Ax0.rows() + Aeq.rows();
  int costcols = Azmp.cols();

  SparseMatrix<double, RowMajor> AcostSparse(costrows, costcols);
  AcostSparse.topRows(Azmp.rows()) = wzmp * Azmp;
  AcostSparse.middleRows(Azmp.rows(), AxN.rows()) = wxN * AxN;
  AcostSparse.middleRows(Azmp.rows() + AxN.rows(), Ax0.rows()) = wx0 * Ax0;
  AcostSparse.bottomRows(Aeq.rows()) = weq * Aeq;
  Acostglobal = AcostSparse;

  SparseMatrix<double> Psparse = AcostSparse.transpose() * AcostSparse;
  Pglobal = Psparse.triangularView<Eigen::Upper>();

  MatrixXd bcost(costrows, 1);
  bcost << wzmp * bzmp, wxN * bxN, wx0 * bx0, weq * beq;
  MatrixXd q = -AcostSparse.transpose() * bcost;

  SparseMatrix<double> Asparse((N + 1) * nx + N * nu, (N + 1) * nx + N * nu);
  std::vector<Triplet<double>> Atriplet;
  for (int i = 0; i < (N + 1) * nx + N * nu; i++)
  {
    Atriplet.emplace_back(i, i, 1);
  }
  Asparse.setFromTriplets(Atriplet.begin(), Atriplet.end());
  Aglobal = Asparse;

  std::vector<c_float> qvec, lvec, uvec;
  for (int i = 0; i < q.size(); i++)
  {
    qvec.push_back(q(i));
  }

  uvec.insert(uvec.begin(), (N + 1) * nx, xmax(0));
  uvec.insert(uvec.begin() + (N + 1) * nx, N * nu, umax(0));

  lvec.insert(lvec.begin(), (N + 1) * nx, xmin(0));
  lvec.insert(lvec.begin() + (N + 1) * nx, N * nu, umin(0));

  lvecglobal = lvec;
  uvecglobal = uvec;

  //solution

  // SparseMatrix<double> tu =  Psparse.triangularView<Eigen::Upper>();

  DynSolver.update(Pglobal, qvec, Asparse, lvec, uvec);
  VectorXd solution = DynSolver.solver();

  /*
	std::vector<Vector2d> copvec;
	std::vector<Vector3d> velvec;
	std::vector<Vector3d> comvec;
	std::vector<Vector2d> cpvec;

	Vector2d cop(0., 0.);
	Vector3d vel(x0(2), x0(3), 0.);
	Vector3d compos(x0(0), x0(1), Zcom);
	Vector2d cp(0., 0.);

	for (int i = 0; i < N; i++)
	{
		VectorXd cop0 = cdt * Vector3d(solution(i * 6), solution(i * 6 + 2), solution(i * 6 + 4));
		VectorXd cop1 = cdt * Vector3d(solution(i * 6 + 1), solution(i * 6 + 3), solution(i * 6 + 5));
		cop = Vector2d(cop0(0), cop1(0));
		copvec.push_back(cop);

		vel += Vector3d(solution(i * 6 + 4), solution(i * 6 + 5), 0.) * timeStep;
		velvec.push_back(vel);

		compos += Vector3d(solution(i * 6 + 2), solution(i * 6 + 3), 0.) * timeStep;
		comvec.push_back(compos);

		cp = compos.segment(0, 2) + vel.segment(0, 2) / omega_x;
		cpvec.push_back(cp);
	}
	cprefvec = cpvec;
	CoMVvec = velvec;
	CoMvec = comvec;

	cout << fixed << setprecision(5);
	std::ofstream copstream("/home/fsr/Desktop/matalb/copstream.txt");
	for (int i = 0; i < N; i++)
	{
		copstream
				<< coprefvec[i](0) << " , " << coprefvec[i](1) << " , "
				<< copvec[i](0) << " , " << copvec[i](1) << " , "
				<< optcopref[i](0) << " , " << optcopref[i](1) << " , "

				<< CoMvec[i](0) << " , " << CoMvec[i](1) << " , "
				<< CoMVvec[i](0) << " , " << CoMVvec[i](1) << " , "
				<< cprefvec[i](0) << " , " << cprefvec[i](1) << " , "

				<< solution(i * 6) << " , " << solution(i * 6 + 1) << " , "
				<< solution(i * 6 + 2) << " , " << solution(i * 6 + 3) << " , "
				<< solution(i * 6 + 4) << " , " << solution(i * 6 + 5) << " , "

				<< comvec[i](0) << " , " << comvec[i](1) << " , "
				<< velvec[i](0) << " , " << velvec[i](1) << " , "
				<< cpvec[i](0) << " , " << cpvec[i](1) << " , "

				// <<0.05*legStatevec[i]
				<< endl;
	}

	copstream.close();
	*/
}
