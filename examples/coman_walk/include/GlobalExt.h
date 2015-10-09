#ifndef _GLOBALEXT_H// header guards
#define _GLOBALEXT_H

#include <vector>
using namespace std;

#define RADTODEG(x)  x*180.0/pi
#define DEGTORAD(x)  x*pi/180.0

//--------below are the global varaibles declearation---------------
extern const double g;// gravity constant
extern const double pi;

//-------- begin of physical parameters of cCub robot
extern double step_width;
extern const double hip_Comy; // hip displacement to the pelvis center
extern const double lengthupperleg;
extern const double lengthunderleg;
extern const double footankletotip;
extern const double footankletoside;
extern const double footankletoback;
extern const double fullleg;
extern const double ankle_height;  //ankle height is 6.03cm, not that small

extern const double extraMass;
extern const double MassUpperbody;
extern const double MassUpperleg;
extern const double MassUnderleg;
extern const double MassFoot;

extern const double MassTot;
extern const double Mass[];

// xyz position of COM in local frame
extern const double COG_UB[3];
extern const double COG_UpperLeg[3];
extern const double COG_UnderLeg[3];
extern const double COG_foot[3];

extern const double IG_body_loc[3][3];

extern const double IG_upperleg_loc[3][3];

extern const double IG_underleg_loc[3][3];

extern const double IG_foot_loc[3][3];

extern double P[3][1];// max hip height
//------------ end of physical parameters


//--- begin of control parameters
extern const double deltay_plus;
extern const double deltay_minus;
extern const double deltax_plus;
extern const double deltax_minus;
extern double xzmp_push;  // offset from zmpx to comx

extern const double Bunny; // sagittal virtial bunny and damper to pull COM

extern double ref_point;
extern double pitch;  // 15 or 20 degree is good
extern double Body_R[3][3];

extern double delta_Hipx;//to control the hip x offset to stabilize
extern double K_sway;//to control the sway distance
//--- end of control parameters


//---- begin of gains in control
extern const double Kp;
extern const double Cvv;	// lateral virtual spring damper to rest COM

extern const double Kv;
extern const double Cv;	// lateral virtial bunny and damper to pull COM to limit cyle

extern const double Kpx;	// P D gain of sagittal virtual spring damper
extern const double Kvx;
extern const double Kpx2;// sagittal virtual spring gain to stop
extern const double Kvx2;// sagittal virtual damper gain to stop

extern double vheight;// how much percent
extern double vheight_des;// how much percent

//---- end of gains in control


//--- begin of gait parameters
extern const double Lift;
//extern double Lift_amp;
extern double entry_point;
extern double entry_time;
extern double p_ds;
extern double t_ds;

extern double zc;
extern double delta; // step width 5 cm
extern double ds;// max sway distance
extern double ds_des;// desired max sway distance from user command

extern double get_comp[3];
extern double rest_p; // the position where the com finally rests
extern double Vx_des;	// without setting any value
extern double SL;	// Step Length, which is continous after filtering
extern double SL_des;	// desired Step Length from user command, which is discotinous
extern double Ox;
//--- end of gait parameters

//--------begin of time parameters in simulation-----------
//double time=0;
extern long int dtime;	// discrete time
extern long int dloop;	// discrete points to save data
extern const double Tstep;
extern long int endTimeSamples;
extern long int stopTimeSamples;

extern double Tcycle;

extern double Duration;  // for foot x generation
extern double ref_time; //for foot x generation
extern double ref_x; //for foot x generation
extern double ref_v; //for foot x generation
extern double ref_a; //for foot x generation
extern double X_ref;

//------- begin of initial condition for the robot-------
extern double comx; // initial com position
extern double comy;
extern double comz;
extern double COM_realpos[3];
extern vector<double> COM_new;// store the COM data for solving inverse kinematics

extern double Vx;
extern double Vy;
extern double Vz;
extern double Ax;
extern double Ay;
extern double Az;

extern double ZMPx;
extern double ZMPy;
extern double ZMPx_fc;
extern double ZMPy_fc;
extern double Kxfc;
extern double Kyfc;
extern double desV;
extern double pseudoZMPx;
extern double pseudoZMPy;

extern vector<int> walking_phase;
extern vector<int> phase_no;

extern const double offset;  // the offset between foot plate center and ankle position
extern double comx_offset; //comx offset (initial position) respect to ankle

extern int numberofsteps;
extern double FootCenter[2];// 1st element is the initial value, last is the latest
extern double Footx[2];//
extern double SuppotCenter;// for sagittal comx motion
//------- end of initial condition for the robot-------



//-------- begin of flags, use in logic switcher
typedef struct
{
	int enable;
	int done;
}_flags;
extern _flags checkMoveToInitial;
extern _flags checkEntry;
extern _flags checkQ1;
extern _flags checkQ4;
extern _flags checkQ3;
extern _flags checkQ2;
extern _flags checkStop;
extern _flags checkPreStop;//slow down in one step
extern _flags commandStop;// keyboard to stop
extern _flags turnLeft;// flags for keyboard to turn
extern _flags turnRight;//flags for  keyboard to turn
extern _flags startTurnLeft;//flags for generating yaw angles
extern _flags startTurnRight;//flags for generating yaw angles

extern int mark_velocity; // record the velocity direction for stopping robot
//-------- end of flags, use in logic switcher
//
//-------- begin of foot vector
typedef struct
{
	vector<double> x;// previous and current discret point
	vector<double> y;
	vector<double> z;
}_3rowVector;

extern _3rowVector HipP;
extern _3rowVector LeftFootP;
extern _3rowVector RightFootP;
extern _3rowVector LeftFootStancePos;
extern _3rowVector RightFootStancePos;
//-------- end of foot vector

////-------- begin of data storage vectors
typedef struct
{
	vector<double> comx;
	vector<double> comy;	// vector has double value, defualt is zero value
	vector<double> comz;

	vector<double> realcomx;
	vector<double> realcomy;	// vector has double value, defualt is zero value
	vector<double> realcomz;

	vector<double> Hipx;
	vector<double> Hipy;
	vector<double> Hipz;

	vector<double> phase_no;
	vector<double> Vy;
	vector<double> desV;
	vector<double> zmpx;
	vector<double> zmpy;
	vector<double> walking_phase;
	vector<double> Vx;

	vector<double> RightFootx;
	vector<double> RightFooty;
	vector<double> RightFootz;

	vector<double> LeftFootx;
	vector<double> LeftFooty;
	vector<double> LeftFootz;

}_data;

extern _data data;
//-------- end of data storage vectors
extern vector<double> dtraj;// reference joint angular trajectory
extern vector<double> traj;// joint angular trajectory after compensation
extern vector<double> traj_old;// joint angular trajectory
//--------above are the global varaibles declearation---------------
extern double HipO[4];
extern double LFtO[4];
extern double RFtO[4];
extern double WaistO[3];
extern double WaistO_des[3];

extern double temp1;
extern double temp2;
extern double temp3;

extern double q_compst[15][2];
extern double q_err[15][2];
extern double q_link[15][2];

extern double deltaHip[5];//first 3 are the modification of hip position, last two are hip pitch and roll

extern int takestepenable;//enable robot takes a step to balance
extern double takesteptime;//the time when robot takes a step to balance

// temp i put home position here
extern vector<float> homePos;// degree

#endif
