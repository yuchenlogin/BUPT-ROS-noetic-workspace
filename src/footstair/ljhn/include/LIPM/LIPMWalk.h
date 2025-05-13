/*
LIPMWalk.cpp
*/

#ifndef WALK_HPP
#define WALK_HPP


#include <Eigen/Dense>  
#include <Eigen/Geometry> 
#include <type_traits>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>
#include <vector>

#define Pi 3.14159265358979323846

using namespace Eigen;  

template<class V>
constexpr V sqr(const V& a) { return a * a; }

typedef Eigen::Matrix<float, 3, 2> Matrix3x2f;
typedef Eigen::Matrix<float, 2, 3> Matrix2x3f;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Translation<float,3>  Translation3f;
typedef Eigen::Transform<float,3,Affine> Transform3f;

namespace GaitManager {
class LIPMWalk{
	public:
		LIPMWalk();
		~LIPMWalk();

		struct WholeBodyMotion
		{
			sva::PTransformd leftleg, rightleg, torso;
			sva::MotionVecd leftlegVelo, rightlegVelo, torsoVelo;
			int stepphase, steprhythm, rhythmcount, stepcount;
		};




		void computeWholeBodyTrajectory(std::vector<WholeBodyMotion>& Traj);

		int run();
		void start(double x, int targetstepcount, double angle);
		void start(double *waypoint_X0, int targetstepcount, double angle);
		void start(double x, int targetstepcount, double *waypoint_Yaw);
		void start(double *waypoint_X0, int targetstepcount, double *waypoint_Yaw);	
		void stop();
		struct ParameterOfPosture{
			//定义右手系，机器人前方为x轴正方形，左方为y轴正方向，上方为z轴正方向
			double Lfoot_x, Lfoot_y, Lfoot_z, Lfoot_R, Lfoot_P, Lfoot_Y;
			double Rfoot_x, Rfoot_y, Rfoot_z, Rfoot_R, Rfoot_P, Rfoot_Y;
			double Torso_x, Torso_y, Torso_z, Torso_R, Torso_P, Torso_Y, TorsoCoM_z, TorsoCoM_x;
			double LArm_R, LArm_P, LArm_elbow, RArm_R, RArm_P, RArm_elbow;
		} PosPara, LastPosPara, CurrentPos, TouchPos;

		double ArmSwing;
		Vector3f com;
		Vector3f stepmark;
		Vector3f LlegPath;
		Vector3f RlegPath;

		static const double Rad2Deg;
		static const double TorsoHeight;

		int FB_GYRO, RL_GYRO, FB_ACCEL, RL_ACCEL;
		int FSR_L[6],  FSR_R[6];
		double CoM_Vx, CoM_Vy;
		double CoM_x_RelaToW[10], CoM_y_RelaToW[10];
		double CoM_x_ideal[10], CoM_y_ideal[10];
		double CoM_x_measured, CoM_y_measured, CoM_H_measured;
		double error_com_x, error_com_y, error_com_x_last, error_com_y_last;

		double WayPoint_Yaw[1000];
		double WayPoint_Y0[1000];
		double WayPoint_X0[1000];
		double WayPoint_T[1000];

		double TimeStep;		
		int StepCount;
		int StepCountTarget;
		double Swing_H;
		double JointValue[21];//二十个舵机  0号不用
		double CurrentJointValue[21];
		
		enum StateOfRobot
		{
			Walk_side,
			Walk_start_side,
			Walk_stop_side,

			Walk_forward,
			Walk_start,
			Walk_stop,
			Walk_stand,
			Walk_standed,

			Pause,
		} RobotState;

	// private:
		void firstStep();
		void lastStep();

		constexpr const static double Gravity = 9.8;
		double TimeCon_x;
		double TimeCon_y;
		// double T_circle, T_dsp;
		double DissipationRatio;
		double DSPRatio;
		double DSP_x;
		double DSP_y;
		void PatternInit();

		enum PhaseOfStep
		{
			LeftStance = -1,
			RightStance = 1
		} StepPhase, LastStepPhase;
		enum ChangeOfPhase
		{ 
			NoChange = 0,
			LtoR = 1,
			RtoL = -1
		} PhaseChange;
		int StepRhythm;
		int RhythmCount;
		int extraDSP;
		void updateStepPhase();

		Transform3f TransformUpdateS;
		Transform3f TransformPToS;
		Transform3f TransformLastPToP;
		Transform3f TransformSToW;
		Transform3f TransformNextSToS;
		Transform3f TransformNextPToNextS;
		Transform3f TransformLastSToS;
		void updateTransform();
		/*P坐标系下*/
		// double V0, Angle_VxVy;//重心位于中间时的速度方向角，右腿支撑时，质心速度朝向左前方，规定此时Angle_V0为负
		struct footprint{
			constexpr static const double torso_c_stand = 0.002;
			double torso_c;
			double D_sta, D_end;
			double F_sta, F_end;
			double VeloX_sta, VeloX_end;
			double VeloY_sta, VeloY_end;
		}FootPrintCurrent,FootPrintOld,FootPrintNew;
		void computePendulumVelo(struct footprint* LastFootPrint,struct footprint* FootPrint);

		double sinHx;
		double sinHy;
		double cosHx;
		double cosHy;
		void updateStepParameter();

		int PendulumCount, PendulumCountEnd;
		double CoM_x_RelaToP[2000];
		double CoM_y_RelaToP[2000];
		double V0_x_RelaToP[2000];
		double V0_y_RelaToP[2000];
		void updatePendulum();

		/*S坐标系下*/
		double RobotYaw_Torso,	RobotYaw_Pendulum,	RobotYawCurrent,		RobotYawNew,	RobotYawOld;
		double DSvelo_x, DSvelo_y;
		Vector6d cubicsplinePx, cubicsplineAx, cubicsplinePy, cubicsplineAy;
		void CubicSplineInit(Vector6d *In, Vector6d *Out, double t);
		double CubicSpline(Vector6d *In,  double t);
		void TransformPtoS(double Xin, double Yin, double *Xout, double *Yout);
		void TransformTtoP(double Xin, double Yin, double *Xout, double *Yout);
		void SolePitchCal(int FrontOrBack, double *pitchOut, double *xOut, double *yOut, double *zOut);

		Vector3f vectorSwingBegin,vectorSwingEnd;
		struct ParameterOfSwingPhase{
			double swing_x_begin, swing_x_end, swing_y_begin,swing_z_begin;
			double swing_y_end, swing_Y_begin, swing_Y_end, swing_z_end; 
		}SwingParameter;
		void footTrajectory(double rhythm,struct ParameterOfSwingPhase  SwingPara);
		Eigen::Matrix3d Euler_2_rotation(Eigen::Matrix<double,3,1> euler);

		Vector3f Interpolation[4];
		Eigen::Vector3f getBezierPoint(double t);

		Eigen::Vector3f getCycloidPoint(double t,Vector3f beginp,Vector3f endp);

		void generatePosture();
		void computerJointValue();

		void HipOffset();
		void GyroBalance();
};

}
#endif

