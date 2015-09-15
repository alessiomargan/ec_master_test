#include <math.h>
#include <iostream>
#include <vector>
#define DEGTomRAD(X) (X*M_PI*1e5)/180.0
using namespace std;
double discreteTime=0;// discrete time in sec

bool test1_enable=false;
bool test2_enable=false;
bool stop_enable=false;



// maximum joint angle positive
vector<float> joint_max = {
    // lower body #15
    0,  45,  25,  35,  35, 2, 45,  105,  65,30,  55, 45, 105,70, 30,
//  1,  2,   3,   4,   5,  6,  7,    8,   9, 10, 11, 12, 13, 14, 15
    // upper body #10 right arm to left arm, last 2 are right and left neck
    90, 100,  70, -2,  90, 25, 70, -2, 0,  0};
//  16, 17,  18,  19, 20, 21, 22, 23, 24, 25
// maximum joint angle negative
vector<float> joint_min = {
    // lower body #15
    0, -15,-25, -90,-90,-55, -45,  0, -45, -30, -2,-45,  0,-45, -30,
//  1,  2,  3,   4,  5,  6,   7,   8,  9,  10,  11, 12, 13, 14, 15
    // upper body #10 right arm to left arm, last 2 are right and left neck
   -180,-25,-70,-120,-180,-100,-70,-120,  0,  0};
//  16, 17,  18, 19, 20,  21,   22, 23,   24, 25

//declare subfunctions
void joint_speed(vector<float> homePos, int size, double freq, int r_pos[]);
void joint_home(vector<float> homePos, int size, int r_pos[]);
void joint_range(vector<float> homePos, int size, double freq, int r_pos[]);
void JointTestState(char cmd);

//------------------------------------------------------------
void test_joint(vector<float> homePos, int size, double freq, int r_pos[])
{
    if(test1_enable==true&&test2_enable==false) //01
    {
        joint_range(homePos, size, freq, r_pos);
    }
    else if(test2_enable==true&&test1_enable==false) //10
    {
        joint_speed(homePos, size, 0.25, r_pos);//speed test has 5times higher frequency
    }
    else if(test2_enable==false&&test1_enable==false) // 00
    {
        joint_home(homePos, size, r_pos);
    }

}


void joint_speed(vector<float> homePos, int size, double freq, int r_pos[])
{
    double sinwave=sin(2*M_PI*freq*discreteTime);
    double coswave=0.5*(1-cos(2*M_PI*freq*discreteTime));

    double hipangle=40;
    // lower body boards
    r_pos [ 0] = DEGTomRAD(homePos [0]);// waist yaw
    r_pos [ 1] = DEGTomRAD(homePos [1]) + DEGTomRAD(0)  * sinwave;// waist pitch
    r_pos [ 2] = DEGTomRAD(homePos [2]) + DEGTomRAD(0)  * sinwave;// waist roll
    r_pos [ 3] = DEGTomRAD(homePos [3]) + DEGTomRAD(-hipangle) * coswave;//right hip pitch
    r_pos [ 4] = DEGTomRAD(homePos [4]) + DEGTomRAD(-hipangle) * coswave;//left hip pitch
    r_pos [ 5] = DEGTomRAD(homePos [5]) + DEGTomRAD(0)  * coswave;//right hip roll
    r_pos [ 6] = DEGTomRAD(homePos [6]) + DEGTomRAD(0)  * coswave;//right hip yaw
    r_pos [ 7] = DEGTomRAD(homePos [7]) + DEGTomRAD(2*hipangle)* coswave;//right knee
    r_pos [ 8] = DEGTomRAD(homePos [8]) + DEGTomRAD(-hipangle) * coswave;//right ankle pitch
    r_pos [ 9] = DEGTomRAD(homePos [9]) + DEGTomRAD(0)  * coswave;//right ankle roll
    r_pos [10] = DEGTomRAD(homePos[10]) + DEGTomRAD(0)  * coswave;//left hip roll
    r_pos [11] = DEGTomRAD(homePos[11]) + DEGTomRAD(0)  * coswave;//left hip yaw
    r_pos [12] = DEGTomRAD(homePos[12]) + DEGTomRAD(2*hipangle)* coswave;//left knee
    r_pos [13] = DEGTomRAD(homePos[13]) + DEGTomRAD(-hipangle) * coswave;//left ankle pitch
    r_pos [14] = DEGTomRAD(homePos[14]) + DEGTomRAD(0)  * coswave;//left ankle roll
    // upper body boards
    r_pos [15] = DEGTomRAD(homePos[15]) + DEGTomRAD(30) * coswave;// right shoulder pitch
    r_pos [16] = DEGTomRAD(homePos[16]) - DEGTomRAD(5) * coswave;// right shoulder roll
    r_pos [17] = DEGTomRAD(homePos[17]) - DEGTomRAD(10) * coswave;// right shoulder yaw
    r_pos [18] = DEGTomRAD(homePos[18]) + DEGTomRAD(20)  * coswave;// right elbow

    r_pos [19] = DEGTomRAD(homePos[19]) + DEGTomRAD(30) * coswave;// left shoulder pitch
    r_pos [20] = DEGTomRAD(homePos[20]) + DEGTomRAD(5) * coswave;// left shoulder roll
    r_pos [21] = DEGTomRAD(homePos[21]) + DEGTomRAD(10) * coswave;// left shoulder yaw
    r_pos [22] = DEGTomRAD(homePos[22]) + DEGTomRAD(20)  *coswave;// left elbow

    discreteTime +=0.001; // loop runs at 1ms
    if(discreteTime>(1.0/freq))
    {
        discreteTime=0; // reset time to zero after a cycle
        if (stop_enable==true)
        {
            test2_enable=false;//disable test2
            //test2_done=true;// test2 completed
            cout << "Speed test completed:)" << endl;
        }
    }
}

void joint_home(vector<float> homePos, int size, int r_pos[])
{
    // lower body boards
    r_pos [ 0] = DEGTomRAD(homePos [0]);// waist yaw
    r_pos [ 1] = DEGTomRAD(homePos [1]);// waist pitch
    r_pos [ 2] = DEGTomRAD(homePos [2]);// waist roll
    r_pos [ 3] = DEGTomRAD(homePos [3]);//right hip pitch
    r_pos [ 4] = DEGTomRAD(homePos [4]);//left hip pitch
    r_pos [ 5] = DEGTomRAD(homePos [5]);//right hip roll
    r_pos [ 6] = DEGTomRAD(homePos [6]);//right hip yaw
    r_pos [ 7] = DEGTomRAD(homePos [7]);//right knee
    r_pos [ 8] = DEGTomRAD(homePos [8]);//right ankle pitch
    r_pos [ 9] = DEGTomRAD(homePos [9]);//right ankle roll
    r_pos [10] = DEGTomRAD(homePos[10]);//left hip roll
    r_pos [11] = DEGTomRAD(homePos[11]);//left hip yaw
    r_pos [12] = DEGTomRAD(homePos[12]);//left knee
    r_pos [13] = DEGTomRAD(homePos[13]);//left ankle pitch
    r_pos [14] = DEGTomRAD(homePos[14]);//left ankle roll
    // upper body boards
    r_pos [15] = DEGTomRAD(homePos[15]);// right shoulder pitch
    r_pos [16] = DEGTomRAD(homePos[16]);// right shoulder roll
    r_pos [17] = DEGTomRAD(homePos[17]);// right shoulder yaw
    r_pos [18] = DEGTomRAD(homePos[18]);// right elbow

    r_pos [19] = DEGTomRAD(homePos[19]);// left shoulder pitch
    r_pos [20] = DEGTomRAD(homePos[20]);// left shoulder roll
    r_pos [21] = DEGTomRAD(homePos[21]);// left shoulder yaw
    r_pos [22] = DEGTomRAD(homePos[22]);// left elbow
}


void joint_range(vector<float> homePos, int size, double freq, int r_pos[])
{
    // unit wave
    double Tcycle = 1.0/freq;
    double tau = 0.5*Tcycle;
    double w = 2*M_PI/tau;
    double coswave=0.5*(1-cos(w*discreteTime));

    vector<float> A(size,0);// magnitude of positive max range
    vector<float> B(size,0);// magnitude of negative max range

    for (int i=0;i<size;i++)
    {
        A[i]=joint_max[i]-homePos[i];
        B[i]=joint_min[i]-homePos[i];
    }

    if(discreteTime<tau)  // first half cycle
    {
            //lower body boards
            r_pos [ 0] = DEGTomRAD(A[0])*coswave;  // waist yaw
            r_pos [ 1] = DEGTomRAD(A[1])*coswave;  // waist pitch
            r_pos [ 2] = 0*DEGTomRAD(A[2])*coswave;  // waist roll
            r_pos [ 3] = DEGTomRAD(A[3])*coswave;  //right hip pitch
            r_pos [ 4] = DEGTomRAD(A[4])*coswave; //left hip pitch
            r_pos [ 5] = DEGTomRAD(B[5])*coswave;  //right hip roll move outward
            r_pos [ 6] = DEGTomRAD(B[6])*coswave;  //right hip yaw move outward
            r_pos [ 7] = DEGTomRAD(A[7])*coswave; //right knee
            r_pos [ 8] = DEGTomRAD(A[8])*coswave;  //right ankle pitch
            r_pos [ 9] = DEGTomRAD(A[9])*coswave;  //right ankle roll
            r_pos [10] = DEGTomRAD(A[10])*coswave;  //left hip roll move outward
            r_pos [11] = DEGTomRAD(A[11])*coswave;  //left hip yaw move outward
            r_pos [12] = DEGTomRAD(A[12])*coswave; //left knee
            r_pos [13] = DEGTomRAD(A[13])*coswave;  //left ankle pitch
            r_pos [14] = DEGTomRAD(B[14])*coswave;  //left ankle roll
            // upper body boards
            r_pos [15] = DEGTomRAD(homePos[15])+DEGTomRAD(B[15])*coswave;// right shoulder pitch
            r_pos [16] = DEGTomRAD(homePos[16])+DEGTomRAD(A[16])*coswave;// right shoulder roll
            r_pos [17] = DEGTomRAD(homePos[17])+DEGTomRAD(B[17])*coswave;// right shoulder yaw
            r_pos [18] = DEGTomRAD(homePos[18])+DEGTomRAD(B[18])*coswave;// right elbow

            r_pos [19] = DEGTomRAD(homePos[19])+DEGTomRAD(B[19])*coswave;// left shoulder pitch
            r_pos [20] = DEGTomRAD(homePos[20])+DEGTomRAD(B[20])*coswave;// left shoulder roll
            r_pos [21] = DEGTomRAD(homePos[21])+DEGTomRAD(A[21])*coswave;// left shoulder yaw
            r_pos [22] = DEGTomRAD(homePos[22])+DEGTomRAD(B[22])*coswave;// left elbow
    }
    else    // 2nd half cycle
    {
            //lower body boards
            r_pos [ 0] = DEGTomRAD(B[0])*coswave;  // waist yaw
            r_pos [ 1] = DEGTomRAD(B[1])*coswave;  // waist pitch
            r_pos [ 2] = 0*DEGTomRAD(B[2])*coswave;  // waist roll
            r_pos [ 3] = DEGTomRAD(B[3])*coswave;  //right hip pitch
            r_pos [ 4] = DEGTomRAD(B[4])*coswave; //left hip pitch
            r_pos [ 5] = DEGTomRAD(A[5])*coswave;  //right hip roll move inward
            r_pos [ 6] = DEGTomRAD(A[6])*coswave;  //right hip yaw move inward
            r_pos [ 7] = DEGTomRAD(A[7])*coswave; //right knee
            r_pos [ 8] = DEGTomRAD(B[8])*coswave;  //right ankle pitch
            r_pos [ 9] = DEGTomRAD(B[9])*coswave;  //right ankle roll
            r_pos [10] = DEGTomRAD(B[10])*coswave;  //left hip roll move inward
            r_pos [11] = DEGTomRAD(B[11])*coswave;  //left hip yaw move inward
            r_pos [12] = DEGTomRAD(A[12])*coswave; //left knee
            r_pos [13] = DEGTomRAD(B[13])*coswave;  //left ankle pitch
            r_pos [14] = DEGTomRAD(A[14])*coswave;  //left ankle roll
            // upper body boards
            r_pos [15] = DEGTomRAD(homePos[15])+DEGTomRAD(A[15])*coswave;// right shoulder pitch
            r_pos [16] = DEGTomRAD(homePos[16])+DEGTomRAD(B[16])*coswave;// right shoulder roll
            r_pos [17] = DEGTomRAD(homePos[17])+DEGTomRAD(A[17])*coswave;// right shoulder yaw
            r_pos [18] = DEGTomRAD(homePos[18])+DEGTomRAD(A[18])*coswave;// right elbow

            r_pos [19] = DEGTomRAD(homePos[19])+DEGTomRAD(A[19])*coswave;// left shoulder pitch
            r_pos [20] = DEGTomRAD(homePos[20])+DEGTomRAD(A[20])*coswave;// left shoulder roll
            r_pos [21] = DEGTomRAD(homePos[21])+DEGTomRAD(B[21])*coswave;// left shoulder yaw
            r_pos [22] = DEGTomRAD(homePos[22])+DEGTomRAD(A[22])*coswave;// left elbow
    }

    discreteTime +=0.001; // loop runs at 1ms
    if(discreteTime>Tcycle)
    {
        discreteTime=0; //after repeating 1 cycle, reset time to zero
        if (stop_enable==true)
        {
            test1_enable=false;//disable test1
            cout << "Joint range test completed:)" << endl;
        }
    }
}


// this is the funtion that i need to put in the _loop in order to take the kayboard command
void JointTestState(char cmd)
{
	switch (cmd)
	{
		case '1':
			{
				if(test1_enable==true)
				{
				    cout << "Joint range test is already running:)" << endl;
				}
				else if(test2_enable==true)
				{
				    cout << "Speed test is running. Stop it by pressing 's':)" << endl;
				}
                else if(test1_enable==false&&test2_enable==false)
				{
				    test1_enable=true;//enable test1
				    stop_enable=false;
				    cout << "Start test of joint range:)" << endl;
				}
			}
		break;

        case '2':
			{
				if(test2_enable==true)
				{
				    cout << "Speed test is already running:)" << endl;
				}
                else if(test1_enable==true)
				{
				    cout << "Joint range test is running. Stop it by pressing 's':)" << endl;
				}
                else if(test2_enable==false&&test1_enable==false)
				{
				    test2_enable=true;//enable test2
				    stop_enable=false;
				    cout << "Start speed test:)" << endl;
				}
			}
		break;

		case 's':
            {
			    if(test1_enable==true||test2_enable==true)
			    {
			        stop_enable=true;
			        cout << "Current test will stop after one cycle" << endl;
			    }
			}
		break;

		default:
		break;
	}
}
