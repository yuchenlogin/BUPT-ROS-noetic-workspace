#include <iostream>
#include <fstream>
#include <cmath>
#include <time.h>

#include "DCMBasedPlanning.h"
#include "LIPMWalk.h"

using namespace Human;
using namespace std;

DCMBasedPlanning *DCMBasedPlanning::m_UniqueInstance = new DCMBasedPlanning();

DCMBasedPlanning::DCMBasedPlanning()
{
	timeStep = 0.01; //s
	Tstep = 0.32;
	Tdsini = 0.1;
	Tdsend = 0.1;

	stepLength = 0.;
	stepWidth = 0.06; //m
	swingH = 0.02;		//m

	lFootHoffset = 0.;

	TorsoHeightWalk = 0.345; //torso height at walking

	gra_g = 9.81; //m/s2

	double CoMH = TorsoHeightWalk; //com height

	TimeCon_x = std::sqrt(gra_g / CoMH); //sqrt(g/H)
	TimeCon_y = std::sqrt(gra_g / CoMH);
}

DCMBasedPlanning::~DCMBasedPlanning()
{
}

Eigen::Vector6d DCMBasedPlanning::fifthPolyInterpInit(Eigen::Vector6d bound, double t) //bound--(start pos,end pos,start vel,end vel,start acc,end add)
{
	Eigen::Matrix<double, 6, 6> m6X6;
	Eigen::Vector6d coef; //coef--(a0,a1,a2,a3,a4,a5)

	m6X6 << 1, 0, 0, 0, 0, 0,
			1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5),
			0, 1, 0, 0, 0, 0,
			0, 1, 2 * t, 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4),
			0, 0, 2, 0, 0, 0,
			0, 0, 2, 6 * t, 12 * pow(t, 2), 20 * pow(t, 3);

	coef = m6X6.inverse() * bound;

	return coef;
}

double DCMBasedPlanning::fifthPolyInterp(Eigen::Vector6d coef, double t)
{
	Eigen::Vector6d tVect;
	tVect << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);
	double s = tVect.dot(coef);
	return s;
}

void DCMBasedPlanning::clear()
{
	cprefvec.clear();
	coprefvec.clear();
	comRefTrajvec.clear();
	legStatevec.clear();
	WrightDistvec.clear();
}

Vector2d getSScpref(Vector2d cpend, Vector2d cop, double time, double Tstep)
{
}

Vector2d getDScpref(MatrixXd P, double time)
{
}

Eigen::Vector3d getCycloidPoint(double t, double swingH, Vector3d beginp, Vector3d endp)
{

	t = 2 * M_PI * t; //0-1 ->  0-2*pi
	double r = sqrt(sqr(endp(0) - beginp(0)) + sqr(endp(1) - beginp(1))) / 2;
	double l = r * (t - sin(t));
	double h = r * (1 - cos(t));

	//illegal input
	if (r == 0)
		return beginp;

	Vector3d point;
	point(0) = beginp(0) + (endp(0) - beginp(0)) * l / (2 * M_PI * r);
	point(1) = beginp(1) + (endp(1) - beginp(1)) * l / (2 * M_PI * r);
	point(2) = swingH * h / (2 * r);
	return point;
}

void DCMBasedPlanning::computeCpTraj(std::vector<Vector2d> footprint, RobotState *robot)
{

	int stepnum = footprint.size();

	std::vector<Vector2d> copref;
	copref.push_back((footprint[0] + footprint[1]) / 2);
	for (int i = 1; i < stepnum - 1; i++)
	{
		copref.push_back(footprint[i]);
	}
	copref.push_back((footprint[stepnum - 1] + footprint[stepnum - 2]) / 2);

	std::vector<Vector2d> cpref_end;
	cpref_end.push_back(copref.back());
	for (int i = stepnum - 1; i > 0; i--)
	{
		Vector2d cprefend;
		cprefend = copref[i] + std::exp(-TimeCon_x * Tstep) * (cpref_end.front() - copref[i]);

		cpref_end.insert(cpref_end.begin(), cprefend);
	}

	// for(int i=0;i<stepnum;i++)
	// 	cout<<"cpend: "<<cpref_end[i].transpose()<<endl;

	std::vector<Vector2d> cpref_iniDS, cpref_endDS;
	for (int i = 1; i < stepnum; i++)
	{
		Vector2d cprefiniDS = copref[i - 1] + std::exp(-TimeCon_x * Tdsini) * (cpref_end[i - 1] - copref[i - 1]);
		cpref_iniDS.push_back(cprefiniDS);

		Vector2d cprefendDS = copref[i] + std::exp(TimeCon_x * Tdsend) * (cpref_end[i - 1] - copref[i]);
		cpref_endDS.push_back(cprefendDS);
	}

	std::vector<Vector2d> cpvref_iniDS, cpvref_endDS;
	for (int i = 1; i < stepnum; i++)
	{
		Vector2d cpvrefiniDS = TimeCon_x * std::exp(-TimeCon_x * Tdsini) * (cpref_end[i - 1] - copref[i - 1]);
		cpvref_iniDS.push_back(cpvrefiniDS);

		Vector2d cpvrefendDS = TimeCon_x * std::exp(TimeCon_x * Tdsend) * (cpref_end[i - 1] - copref[i]);
		cpvref_endDS.push_back(cpvrefendDS);
	}

	if (robot->legs == doublesupport)
	{
		cpref_iniDS[0] = robot->cp;
		cpvref_iniDS[0] = robot->cpv;
	}
	else if (robot->legs == leftStance || robot->legs == rightStance)
	{

		double swingt = Tstep - Tdsini;
		double timeused = swingt * robot->LSRSrate;
		Vector2d cp_iniDS = copref[1] + std::exp(TimeCon_x * (swingt - timeused)) * (robot->cp - copref[1]);
		Vector2d cpv_iniDS = TimeCon_x * std::exp(TimeCon_x * (swingt - timeused)) * (robot->cp - copref[1]);

		cpref_iniDS[1] = cp_iniDS;
		cpvref_iniDS[1] = cpv_iniDS;

		Vector2d cp_endDS = copref[1] + std::exp(TimeCon_x * (-(timeused - Tdsend))) * (robot->cp - copref[1]);
		Vector2d cpv_endvDS = TimeCon_x * std::exp(TimeCon_x * (-(timeused - Tdsend))) * (robot->cp - copref[1]);

		cpref_endDS[0] = cp_endDS;
		cpvref_endDS[0] = cpv_endvDS;
	}
	else
	{
		//first setp, do nothing
	}

	MatrixXd Pcoeff(4, 4);
	double Tds = Tdsini + Tdsend;
	Pcoeff << 2 / pow(Tds, 3), 1 / pow(Tds, 2), -2 / pow(Tds, 3), 1 / pow(Tds, 2),
			-3 / pow(Tds, 2), -2 / Tds, 3 / pow(Tds, 2), -1 / Tds,
			0, 1, 0, 0,
			1, 0, 0, 0;

	int timecount = 0;
	int stepcount = 0;

	//first step  ss phase
	for (int i = 0; i < round((Tstep - Tdsini) / timeStep); i++)
	{
		double time = i * timeStep;
		Vector2d cp = copref[stepcount] +
									std::exp(TimeCon_x * (time - Tstep)) * (cpref_end[stepcount] - copref[stepcount]);
		cprefvec.push_back(cp);
		coprefvec.push_back(copref[stepcount]);
		WrightDistvec.push_back(0.5);
		legStatevec.push_back(doublesupport);

		timecount++;
	}

	//first step  ds phase
	MatrixXd bound(4, 2);
	bound << cpref_iniDS[0].transpose(),
			cpvref_iniDS[0].transpose(),
			cpref_endDS[0].transpose(),
			cpvref_endDS[0].transpose();
	MatrixXd P;
	P = Pcoeff * bound;

	for (int i = 0; i < round((Tdsini + Tdsend) / timeStep); i++)
	{
		MatrixXd timecoeff(2, 4);
		double time = i * timeStep;
		timecoeff << pow(time, 3), pow(time, 2), time, 1,
				3 * pow(time, 2), 2 * time, 1, 0;

		MatrixXd cp_cpv_t;
		cp_cpv_t = timecoeff * P;
		Vector2d cp = cp_cpv_t.block(0, 0, 1, 2).transpose();
		Vector2d cpv = cp_cpv_t.block(1, 0, 1, 2).transpose();
		Vector2d cop = cp - cpv / TimeCon_x;
		double weightdist = 0.5 + 0.5 * (cop - copref[stepcount]).norm() / (copref[stepcount + 1] - copref[stepcount]).norm();

		cprefvec.push_back(cp);
		coprefvec.push_back(cop);
		WrightDistvec.push_back(weightdist);
		legStatevec.push_back(doublesupport);

		timecount++;
	}
	stepcount++;

	//middle step
	while (stepcount < stepnum - 1)
	{
		//ss phase
		for (int i = 0; i < round((Tstep - Tdsini - Tdsend) / timeStep); i++)
		{
			double time = i * timeStep;
			Vector2d cp = copref[stepcount] +
										std::exp(TimeCon_x * (time)) * (cpref_endDS[stepcount - 1] - copref[stepcount]);

			cprefvec.push_back(cp);
			coprefvec.push_back(copref[stepcount]);
			WrightDistvec.push_back(1);

			//TODO: add stancefoot tag in footprint
			int stancefoot = footprint[stepcount](1) / fabs(footprint[stepcount](1));
			if (stancefoot == 1)
			{
				legStatevec.push_back(leftStance);
			}
			else
			{
				legStatevec.push_back(rightStance);
			}

			timecount++;
		}

		//ds phase
		// MatrixXd bound(4,2);
		bound.setZero();
		bound << cpref_iniDS[stepcount].transpose(),
				cpvref_iniDS[stepcount].transpose(),
				cpref_endDS[stepcount].transpose(),
				cpvref_endDS[stepcount].transpose();
		// MatrixXd P;
		P.setZero();
		P = Pcoeff * bound;

		for (int i = 0; i < round((Tdsini + Tdsend) / timeStep); i++)
		{
			MatrixXd timecoeff(2, 4);
			double time = i * timeStep;
			timecoeff << pow(time, 3), pow(time, 2), time, 1,
					3 * pow(time, 2), 2 * time, 1, 0;

			MatrixXd cp_cpv_t;
			cp_cpv_t = timecoeff * P;
			Vector2d cp = cp_cpv_t.block(0, 0, 1, 2).transpose();
			Vector2d cpv = cp_cpv_t.block(1, 0, 1, 2).transpose();
			Vector2d cop = cp - cpv / TimeCon_x;
			double weightdist = (cop - copref[stepcount]).norm() / (copref[stepcount + 1] - copref[stepcount]).norm();

			cprefvec.push_back(cp);
			coprefvec.push_back(cop);
			WrightDistvec.push_back(weightdist);
			legStatevec.push_back(doublesupport);

			timecount++;
		}
		stepcount++;
	}

	//last step
	for (int i = Tdsend / timeStep; i <= round(Tstep / timeStep); i++)
	{
		double time = i * timeStep;
		Vector2d cp = copref[stepcount] +
									std::exp(TimeCon_x * (time - Tstep)) * (cpref_end[stepcount] - copref[stepcount]);
		cprefvec.push_back(cp);
		coprefvec.push_back(copref[stepcount]);
		WrightDistvec.push_back(0.5);
		legStatevec.push_back(doublesupport);

		timecount++;
	}

	Vector3d CoM = robot->com;
	Vector3d CoMV = robot->comv;
	Vector3d CoMa = robot->coma;

	int startcount = 0;
	if (robot->legs == doublesupport)
	{
		startcount = robot->timecount - Tstep / timeStep;
	}
	else if (robot->legs == leftStance || robot->legs == rightStance)
	{
		startcount = robot->timecount;
	}
	else
	{
	}

	for (int i = 0; i < startcount; i++)
	{
		CoMavec.push_back(CoMa);
		CoMVvec.push_back(CoMV);
		CoMvec.push_back(CoM);
	}

	for (int i = startcount; i < cprefvec.size(); i++)
	{
		Vector2d cpC = CoM.segment(0, 2) + CoMV.segment(0, 2) / TimeCon_x;
		Vector2d copD = coprefvec[i] + 10 * (cpC - cprefvec[i]);
		CoMa.segment(0, 2) = pow(TimeCon_x, 2) * (CoM.segment(0, 2) - copD);
		CoMV += CoMa * timeStep;
		CoM += CoMV * timeStep;
		CoMavec.push_back(CoMa);
		CoMVvec.push_back(CoMV);
		CoMvec.push_back(CoM);
	}

	coprefvec.push_back(copref[stepcount + 1]);
}
