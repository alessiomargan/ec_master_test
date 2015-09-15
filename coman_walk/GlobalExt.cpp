//#include "stdafx.h"
#include "GlobalExt.h"

//--------below are the global varaibles declearation---------------
const double g=9.81;// gravity constant
const double pi=3.14159265358979323846264388;

//-------- begin of physical parameters of cCub robot
double step_width=1.0;
const double hip_Comy=0.0726; // hip displacement to the pelvis center
const double lengthupperleg = 0.2258;
const double lengthunderleg= 0.2010;
const double footankletotip = .12;
const double footankletoside = 0.09/2.0;
const double footankletoback = 0.06;
const double ankle_height = .0793;
const double fullleg = lengthupperleg+lengthunderleg+ankle_height;

const double extraMass = 12.0;// extra weight
const double electronic=0.08;
const double MassUpperbody = 2.997+extraMass; //+1 is the mass of the metal stuff at the back
const double MassUpperleg = 3.89;
const double MassUnderleg =2.268;  //
const double MassFoot = 0.742;  //

const double MassTot = MassUpperbody+2.0*MassUpperleg+2.0*MassUnderleg+2.0*MassFoot;
const double Mass[] = {MassFoot, MassUnderleg,MassUpperleg, MassFoot, MassUnderleg,MassUpperleg ,MassUpperbody};

// xyz position of COM in local frame
const double COG_UB[3]={0,0,0.2563};
const double COG_UpperLeg[3]={0,0,-0.1007};
const double COG_UnderLeg[3]={0,0,-0.1267};
const double COG_foot[3]={ 0.018,0,-0.0559};

const double IG_body_loc[3][3] ={{0.0253,0,0},
						 {0,0.0085,0},
						 {0,0,0.0216}};

const double IG_upperleg_loc[3][3] ={{0.0144,0,0},
							 {0,0.0144,0},
							 {0,0,0.0007}};

const double IG_underleg_loc[3][3] ={{0.0154,0,0},
							 {0,0.0148,0},
							 {0,0,0.0012}};

const double IG_foot_loc[3][3] ={{0.0005,0,0},
						 {0,0.0012,0},
						 {0,0,0.0009}};

//------------ end of physical parameters


//--- begin of control parameters
const double deltay_plus = 0.02;
const double deltay_minus = -0.005;
const double deltax_plus = 0.07;
const double deltax_minus = -0.07;
double xzmp_push=0;  // offset from zmpx to comx

const double Bunny=8; // sagittal virtial bunny and damper to pull COM

double ref_point = 0;

double delta_Hipx=0.00;//the comx extra offset added to the desired comx, to control the hip x offset to stabilize
double K_sway=1.0;//to control the sway distance
//--- end of control parameters


//---- begin of gains in control
const double Kp = 30;
const double Cvv = 8;	// lateral virtual spring damper to rest COM

const double Kv = 120;
const double Cv =1.0;	// lateral virtial bunny and damper to pull COM to limit cyle

const double Kpx = 5;	// P D gain of sagittal virtual spring damper
const double Kvx = 2;
const double Kpx2=25;// sagittal virtual spring gain to stop
const double Kvx2=12.5;// sagittal virtual damper gain to stop

double vheight_des=0.35;// how much percent
double vheight=vheight_des;// how much percent
//---- end of gains in control

//--- begin of gait parameters
const double Lift = 0.03;
//double Lift_amp = 0;
double entry_point = 0;
double entry_time=0;
double p_ds=0.2;// 20% of DS
double t_ds;


double zc = 0.4;  //0.4 is for half body COMAN, full body COMAN has 0.53
double delta = 0.05; // step width 5 cm
double ds=0.04;// max sway distance delta for half-body coman is 5cm
double ds_des=ds;// desired max sway distance from user command

double get_comp[3]={0,0,zc};
double rest_p = 0; // the position where the com finally rests
double Vx_des;	// without setting any value
double SL = 0.0;	// Step Length
double SL_des = SL;
double Ox = SL/2;
//--- end of gait parameters

//--------begin of time parameters in simulation-----------
//double time=0;
long int dtime=0;	// discrete time
long int dloop=0;	// discrete points to save data
const double Tstep = 0.001;

double Tcycle;

double Duration;  // for foot x generation
double ref_time; //for foot x generation
double ref_x; //for foot x generation
double ref_v; //for foot x generation
double ref_a; //for foot x generation
double X_ref;

int mark_velocity; // for stopping robot

//------- begin of initial condition for the robot-------
const double offset = 0.0;  // the offset between the main support center (COP) and ankle position

double comx_offset = 0.00; // this COM offset gives zero Hip position at the beginning

int numberofsteps=0;
double Footx[2]={0,0};
double FootCenter[2]={Footx[2]+offset,Footx[2]+offset};// 2-element array, 1st is initial value, 2nd is latest value

double SuppotCenter=FootCenter[1];// it should be vector

double comx = comx_offset; // initial com position
double comy = 0;
double comz = zc;
double COM_realpos[3]={0,0,0};
vector<double> COM_new(3,0);// store the COM data for solving inverse kinematics

double Vx = 0;
double Vy = 0;
double Vz = 0;
double Ax = 0;
double Ay = 0;
double Az = 0;

double ZMPx=0;
double ZMPy=0;
double ZMPx_fc=0;
double ZMPy_fc=0;
double Kxfc=2*pi*5;
double Kyfc=2*pi*40;

double desV;
double pseudoZMPx;
double pseudoZMPy;

vector<int> walking_phase(2,0);//only store current and 1 past state
vector<int> phase_no(2,0);


//------- end of initial condition for the robot-------

//-------- begin of foot vector

_3rowVector HipP;
_3rowVector LeftFootP;
_3rowVector RightFootP;
_3rowVector LeftFootStancePos;
_3rowVector RightFootStancePos;

//-------- end of foot vector

//-------- begin of flags, use in logic switcher
_flags checkMoveToInitial;
_flags checkEntry;
_flags checkQ1;
_flags checkQ4;
_flags checkQ3;
_flags checkQ2;
_flags checkStop;
_flags checkPreStop;
_flags checkStopTrigger;
_flags commandStop;
_flags turnLeft;// flags for  keyboard to turn
_flags turnRight;// flags for  keyboard to turn
_flags startTurnLeft;//flags for generating yaw angles
_flags startTurnRight;//flags for generating yaw angles

//-------- end of flags, use in logic switcher

//-------- begin of data storage vectors
_data data;
//-------- end of data storage vectors
//double traj[15]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};// joint trajectory
vector<double> dtraj(30,0);// reference joint angular trajectory
vector<double> traj(30,0);// joint trajectory
vector<double> traj_old(30,0);//to save previous data to get velocity

/*WARNING!: HipO LFtO RFtO are in radian!!!*/
double HipO[4]={0,DEGTORAD(0),0,0};  // roll pitch yaw operation to global coordinate, last is local yaw. DEGTORAD(1.5) gives real zero orientation, due to joint calibration offset
double LFtO[4]={0,0,0,0};
double RFtO[4]={0,0,0,0};
/*roll pitch yaw ([0][1][2]) for global, last yaw ([3]) for local*/
/*the global yaw is used to change the foot yaw with respect to waist
the local yaw is to change the yaw with respect to terrain surface
the local yaw Z axis is always perpendicular to terrain surface*/

/*WARNING!: WaistO is in degree!!!*/
double WaistO[3]={0,0,0};
//	   	waist yaw pitch roll rotation operation respect to local fram, operation order is pith roll yaw
double WaistO_des[3]={0,0,0};

/*temprary data storage*/
double temp1=0;
double temp2=0;
double temp3=0;


/*here the variables for stabilizer*/
double deltaHip[5]={0};//the modification of hip position


/* for balancing: to take a step*/
int takestepenable=0;//enable robot takes a step to balance
double takesteptime=0;//the time when robot takes a step to balance

// home position in degree for old COMAN
vector<float> homePos = {
    // lower body #15
    0,  0,  0,  0,  0, -2,  0, -3,  0,  -2, -3.5, 0,-3,  0,  0,
//  1,  2,  3,  4,  5,  6,  7,  8,  9, 10,  11, 12, 13, 14, 15
    // upper body #10
    -28, 70,  0, 90,  -28, 70,  0, 90,  0,  0};
// 16, 17, 18, 19, 20, 21, 22, 23, 24, 25
//--------above are the global varaibles declearation---------------
