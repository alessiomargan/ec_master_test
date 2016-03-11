#include <math.h>
#include <iostream>
#include <vector>
#define DEGTomRAD(X) (X*M_PI*1e5)/180.0
#define DEGToRAD(X) (X*M_PI)/180.0
using namespace std;
double discreteTime=0;// discrete time in sec

bool test1_enable=false;
bool test2_enable=false;
bool stop_enable=false;



// maximum joint angle positive
//vector<float> joint_max = {
//    // lower body #15
//    0,  45,  25,  35,  35, 2, 45,  105,  65,30,  55, 45, 105,70, 30,
////  1,  2,   3,   4,   5,  6,  7,    8,   9, 10, 11, 12, 13, 14, 15
//    // upper body #10 right arm to left arm, last 2 are right and left neck
//    90, 100,  70, -2,  90, 25, 70, -2, 0,  0};
////  16, 17,  18,  19, 20, 21, 22, 23, 24, 25
//// maximum joint angle negative
//vector<float> joint_min = {
//    // lower body #15
//    0, -15,-25, -90,-90,-55, -45,  0, -45, -30, -2,-45,  0,-45, -30,
////  1,  2,  3,   4,  5,  6,   7,   8,  9,  10,  11, 12, 13, 14, 15
//    // upper body #10 right arm to left arm, last 2 are right and left neck
//   -180,-25,-70,-120,-180,-100,-70,-120,  0,  0};
////  16, 17,  18, 19, 20,  21,   22, 23,   24, 25
///
vector<float> joint_max = {
    // lower body #15
    0,  45,  25,  40,  40, 5, 45,  105,  65, 30, 55, 45, 105, 65, 30,
//  1,  2,   3,   4,   5,   6,  7,    8,   9, 10, 11, 12, 13, 14, 15
    // upper body #10 right arm to left arm, last 2 are right and left neck
    90, 100,  70, -2,  90, 25, 70, -2, 0,  0, 90-5, 30-5, 45-5, 90-5, 30-5, 80-5};
//  16, 17,  18,  19, 20, 21, 22, 23, 24, 25
// maximum joint angle negative
vector<float> joint_min = {
    // lower body #15
    0, -15,-25, -100,-100,-55, -45,  0, -45, -30, -5,-45,  0,-45, -30,
//  1,  2,  3,   4,    5,  6,   7,   8,  9,  10,   11, 12, 13, 14, 15
    // upper body #10 right arm to left arm, last 2 are right and left neck
   -180,-25,-70,-120,-180,-100,-70,-120,  0,  0, -90+5, -30+5, -80+5, -90+5, -30+5, -45+5};
//  16, 17,  18, 19, 20,  21,   22, 23,   24, 25

//declare subfunctions
void joint_speed ( vector<float> homePos, int size, double freq, float r_pos[] );
void joint_home ( vector<float> homePos, int size, float r_pos[] );
void joint_range ( vector<float> homePos, int size, double freq, float r_pos[] );
void JointTestState ( char cmd );

//------------------------------------------------------------
void test_joint ( vector<float> homePos, int size, double freq, float r_pos[] ) {
    if ( test1_enable==true&&test2_enable==false ) { //01
        joint_range ( homePos, size, freq, r_pos );
    } else if ( test2_enable==true&&test1_enable==false ) { //10
        joint_speed ( homePos, size, 0.25, r_pos ); //speed test has 5times higher frequency
    } else if ( test2_enable==false&&test1_enable==false ) { // 00
        joint_home ( homePos, size, r_pos );
    }

}


void joint_speed ( vector<float> homePos, int size, double freq, float r_pos[] ) {
    double sinwave=sin ( 2*M_PI*freq*discreteTime );
    double coswave=0.5* ( 1-cos ( 2*M_PI*freq*discreteTime ) );

    double hipangle=40;
    // lower body boards
    r_pos [ 0] = DEGToRAD ( homePos [0] ); // waist yaw
    r_pos [ 1] = DEGToRAD ( homePos [1] ) + DEGToRAD ( 0 )  * sinwave; // waist pitch
    r_pos [ 2] = DEGToRAD ( homePos [2] ) + DEGToRAD ( 0 )  * sinwave; // waist roll
    r_pos [ 3] = DEGToRAD ( homePos [3] ) + DEGToRAD ( -hipangle ) * coswave; //right hip pitch
    r_pos [ 4] = DEGToRAD ( homePos [4] ) + DEGToRAD ( -hipangle ) * coswave; //left hip pitch
    r_pos [ 5] = DEGToRAD ( homePos [5] ) + DEGToRAD ( 0 )  * coswave; //right hip roll
    r_pos [ 6] = DEGToRAD ( homePos [6] ) + DEGToRAD ( 0 )  * coswave; //right hip yaw
    r_pos [ 7] = DEGToRAD ( homePos [7] ) + DEGToRAD ( 2*hipangle ) * coswave; //right knee
    r_pos [ 8] = DEGToRAD ( homePos [8] ) + DEGToRAD ( -hipangle ) * coswave; //right ankle pitch
    r_pos [ 9] = DEGToRAD ( homePos [9] ) + DEGToRAD ( 0 )  * coswave; //right ankle roll
    r_pos [10] = DEGToRAD ( homePos[10] ) + DEGToRAD ( 0 )  * coswave; //left hip roll
    r_pos [11] = DEGToRAD ( homePos[11] ) + DEGToRAD ( 0 )  * coswave; //left hip yaw
    r_pos [12] = DEGToRAD ( homePos[12] ) + DEGToRAD ( 2*hipangle ) * coswave; //left knee
    r_pos [13] = DEGToRAD ( homePos[13] ) + DEGToRAD ( -hipangle ) * coswave; //left ankle pitch
    r_pos [14] = DEGToRAD ( homePos[14] ) + DEGToRAD ( 0 )  * coswave; //left ankle roll
    // upper body boards
    r_pos [15] = DEGToRAD ( homePos[15] ) + DEGToRAD ( 30 ) * coswave; // right shoulder pitch
    r_pos [16] = DEGToRAD ( homePos[16] ) - DEGToRAD ( 5 ) * coswave; // right shoulder roll
    r_pos [17] = DEGToRAD ( homePos[17] ) - DEGToRAD ( 10 ) * coswave; // right shoulder yaw
    r_pos [18] = DEGToRAD ( homePos[18] ) + DEGToRAD ( 20 )  * coswave; // right elbow

    r_pos [19] = DEGToRAD ( homePos[19] ) + DEGToRAD ( 30 ) * coswave; // left shoulder pitch
    r_pos [20] = DEGToRAD ( homePos[20] ) + DEGToRAD ( 5 ) * coswave; // left shoulder roll
    r_pos [21] = DEGToRAD ( homePos[21] ) + DEGToRAD ( 10 ) * coswave; // left shoulder yaw
    r_pos [22] = DEGToRAD ( homePos[22] ) + DEGToRAD ( 20 )  *coswave; // left elbow

    discreteTime +=0.001; // loop runs at 1ms
    if ( discreteTime> ( 1.0/freq ) ) {
        discreteTime=0; // reset time to zero after a cycle
        if ( stop_enable==true ) {
            test2_enable=false;//disable test2
            //test2_done=true;// test2 completed
            cout << "Speed test completed:)" << endl;
        }
    }
}

void joint_home ( vector<float> homePos, int size, float r_pos[] ) {
    // lower body boards
    r_pos [ 0] = DEGToRAD ( homePos [0] ); // waist yaw
    r_pos [ 1] = DEGToRAD ( homePos [1] ); // waist pitch
    r_pos [ 2] = DEGToRAD ( homePos [2] ); // waist roll
    r_pos [ 3] = DEGToRAD ( homePos [3] ); //right hip pitch
    r_pos [ 4] = DEGToRAD ( homePos [4] ); //left hip pitch
    r_pos [ 5] = DEGToRAD ( homePos [5] ); //right hip roll
    r_pos [ 6] = DEGToRAD ( homePos [6] ); //right hip yaw
    r_pos [ 7] = DEGToRAD ( homePos [7] ); //right knee
    r_pos [ 8] = DEGToRAD ( homePos [8] ); //right ankle pitch
    r_pos [ 9] = DEGToRAD ( homePos [9] ); //right ankle roll
    r_pos [10] = DEGToRAD ( homePos[10] ); //left hip roll
    r_pos [11] = DEGToRAD ( homePos[11] ); //left hip yaw
    r_pos [12] = DEGToRAD ( homePos[12] ); //left knee
    r_pos [13] = DEGToRAD ( homePos[13] ); //left ankle pitch
    r_pos [14] = DEGToRAD ( homePos[14] ); //left ankle roll
    // upper body boards
    r_pos [15] = DEGToRAD ( homePos[15] ); // right shoulder pitch
    r_pos [16] = DEGToRAD ( homePos[16] ); // right shoulder roll
    r_pos [17] = DEGToRAD ( homePos[17] ); // right shoulder yaw
    r_pos [18] = DEGToRAD ( homePos[18] ); // right elbow

    r_pos [19] = DEGToRAD(homePos[19]);// left shoulder pitch
    r_pos [20] = DEGToRAD(homePos[20]);// left shoulder roll
    r_pos [21] = DEGToRAD(homePos[21]);// left shoulder yaw
    r_pos [22] = DEGToRAD(homePos[22]);// left elbow


    r_pos [23] = DEGToRAD(homePos[23]);// 
    r_pos [24] = DEGToRAD(homePos[24]);// 


    r_pos [25] = DEGToRAD(homePos[25]);// 
    r_pos [26] = DEGToRAD(homePos[26]);// 
    r_pos [27] = DEGToRAD(homePos[27]);// 
    r_pos [28] = DEGToRAD(homePos[28]);// 
    r_pos [29] = DEGToRAD(homePos[29]);// 
    r_pos [30] = DEGToRAD(homePos[30]);// 
}


void joint_range ( vector<float> homePos, int size, double freq, float r_pos[] ) {
    // unit wave
    double Tcycle = 1.0/freq;
    double tau = 0.5*Tcycle;
    double w = 2*M_PI/tau;
    double coswave=0.5* ( 1-cos ( w*discreteTime ) );

    vector<float> A ( size,0 ); // magnitude of positive max range
    vector<float> B ( size,0 ); // magnitude of negative max range

    for (int i=0;i<size;i++)
    {
        A[i]=0.9*(joint_max[i]-homePos[i]);
        B[i]=0.9*(joint_min[i]-homePos[i]);
    }

    if ( discreteTime<tau ) { // first half cycle
        //lower body boards
        r_pos [ 0] = DEGToRAD ( homePos[ 3] ) +0*DEGToRAD ( A[0] ) *coswave; // waist yaw
        r_pos [ 1] = DEGToRAD ( homePos[ 4] ) +0*DEGToRAD ( A[1] ) *coswave; // waist pitch
        r_pos [ 2] = DEGToRAD ( homePos[ 5] ) +0*DEGToRAD ( A[2] ) *coswave; // waist roll
        r_pos [ 3] = DEGToRAD ( homePos[ 3] ) +DEGToRAD ( A[3] ) *coswave; //right hip pitch
        r_pos [ 4] = DEGToRAD ( homePos[ 4] ) +DEGToRAD ( A[4] ) *coswave; //left hip pitch
        r_pos [ 5] = DEGToRAD ( homePos[ 5] ) +DEGToRAD ( B[5] ) *coswave; //right hip roll move outward
        r_pos [ 6] = DEGToRAD ( homePos[ 6] ) +DEGToRAD ( B[6] ) *coswave; //right hip yaw move outward
        r_pos [ 7] = DEGToRAD ( homePos[ 7] ) +DEGToRAD ( A[7] ) *coswave; //right knee
        r_pos [ 8] = DEGToRAD ( homePos[ 8] ) +DEGToRAD ( A[8] ) *coswave; //right ankle pitch
        r_pos [ 9] = DEGToRAD ( homePos[ 9] ) +DEGToRAD ( A[9] ) *coswave; //right ankle roll
        r_pos [10] = DEGToRAD ( homePos[10] ) +DEGToRAD ( A[10] ) *coswave; //left hip roll move outward
        r_pos [11] = DEGToRAD ( homePos[11] ) +DEGToRAD ( A[11] ) *coswave; //left hip yaw move outward
        r_pos [12] = DEGToRAD ( homePos[12] ) +DEGToRAD ( A[12] ) *coswave; //left knee
        r_pos [13] = DEGToRAD ( homePos[13] ) +DEGToRAD ( A[13] ) *coswave; //left ankle pitch
        r_pos [14] = DEGToRAD ( homePos[14] ) +DEGToRAD ( B[14] ) *coswave; //left ankle roll
        // upper body boards
        r_pos [15] = DEGToRAD ( homePos[15] ) +DEGToRAD ( B[15] ) *coswave; // right shoulder pitch
        r_pos [16] = DEGToRAD ( homePos[16] ) +DEGToRAD ( A[16] ) *coswave; // right shoulder roll
        r_pos [17] = DEGToRAD ( homePos[17] ) +DEGToRAD ( B[17] ) *coswave; // right shoulder yaw
        r_pos [18] = DEGToRAD ( homePos[18] ) +DEGToRAD ( B[18] ) *coswave; // right elbow

           r_pos [19] = DEGToRAD(homePos[19])+DEGToRAD(B[19])*coswave;// left shoulder pitch
           r_pos [20] = DEGToRAD(homePos[20])+DEGToRAD(B[20])*coswave;// left shoulder roll
           r_pos [21] = DEGToRAD(homePos[21])+DEGToRAD(A[21])*coswave;// left shoulder yaw
           r_pos [22] = DEGToRAD(homePos[22])+DEGToRAD(B[22])*coswave;// left elbow

            r_pos [25] = DEGToRAD(homePos[25])+DEGToRAD(B[25])*coswave;;// 
		    r_pos [26] = DEGToRAD(homePos[26])+DEGToRAD(B[26])*coswave;;// 
		    r_pos [27] = DEGToRAD(homePos[27])+DEGToRAD(B[27])*coswave;;// 
		    r_pos [28] = DEGToRAD(homePos[28])+DEGToRAD(A[28])*coswave;;// 
		    r_pos [29] = DEGToRAD(homePos[29])+DEGToRAD(B[29])*coswave;;// 
		    r_pos [30] = DEGToRAD(homePos[30])+DEGToRAD(A[30])*coswave;;// 

    }
    else    // 2nd half cycle
    {
         
            //lower body boards
            r_pos [ 0] = DEGToRAD(homePos[ 3])+0*DEGToRAD(B[0])*coswave;  // waist yaw
            r_pos [ 1] = DEGToRAD(homePos[ 4])+0*DEGToRAD(B[1])*coswave;  // waist pitch
            r_pos [ 2] = DEGToRAD(homePos[ 5])+0*DEGToRAD(B[2])*coswave;  // waist roll
            r_pos [ 3] = DEGToRAD(homePos[ 3])+DEGToRAD(B[3])*coswave;  //right hip pitch
            r_pos [ 4] = DEGToRAD(homePos[ 4])+DEGToRAD(B[4])*coswave; //left hip pitch
            r_pos [ 5] = DEGToRAD(homePos[ 5])+DEGToRAD(A[5])*coswave;  //right hip roll move inward
            r_pos [ 6] = DEGToRAD(homePos[ 6])+DEGToRAD(A[6])*coswave;  //right hip yaw move inward
            r_pos [ 7] = DEGToRAD(homePos[ 7])+DEGToRAD(A[7])*coswave; //right knee
            r_pos [ 8] = DEGToRAD(homePos[ 8])+DEGToRAD(B[8])*coswave;  //right ankle pitch
            r_pos [ 9] = DEGToRAD(homePos[ 9])+DEGToRAD(B[9])*coswave;  //right ankle roll
            r_pos [10] = DEGToRAD(homePos[10])+DEGToRAD(B[10])*coswave;  //left hip roll move inward
            r_pos [11] = DEGToRAD(homePos[11])+DEGToRAD(B[11])*coswave;  //left hip yaw move inward
            r_pos [12] = DEGToRAD(homePos[12])+DEGToRAD(A[12])*coswave; //left knee
            r_pos [13] = DEGToRAD(homePos[13])+DEGToRAD(B[13])*coswave;  //left ankle pitch
            r_pos [14] = DEGToRAD(homePos[14])+DEGToRAD(A[14])*coswave;  //left ankle roll
            // upper body boards
            r_pos [15] = DEGToRAD(homePos[15])+DEGToRAD(A[15])*coswave;// right shoulder pitch
            r_pos [16] = DEGToRAD(homePos[16])+DEGToRAD(B[16])*coswave;// right shoulder roll
            r_pos [17] = DEGToRAD(homePos[17])+DEGToRAD(A[17])*coswave;// right shoulder yaw
            r_pos [18] = DEGToRAD(homePos[18])+DEGToRAD(A[18])*coswave;// right elbow

            r_pos [19] = DEGToRAD(homePos[19])+DEGToRAD(A[19])*coswave;// left shoulder pitch
            r_pos [20] = DEGToRAD(homePos[20])+DEGToRAD(A[20])*coswave;// left shoulder roll
            r_pos [21] = DEGToRAD(homePos[21])+DEGToRAD(B[21])*coswave;// left shoulder yaw
            r_pos [22] = DEGToRAD(homePos[22])+DEGToRAD(A[22])*coswave;// left elbow

            r_pos [25] = DEGToRAD(homePos[25])+DEGToRAD(A[25])*coswave;;// 
		    r_pos [26] = DEGToRAD(homePos[26])+DEGToRAD(A[26])*coswave;;// 
		    r_pos [27] = DEGToRAD(homePos[27])+DEGToRAD(A[27])*coswave;;// 
		    r_pos [28] = DEGToRAD(homePos[28])+DEGToRAD(B[28])*coswave;;// 
		    r_pos [29] = DEGToRAD(homePos[29])+DEGToRAD(A[29])*coswave;;// 
		    r_pos [30] = DEGToRAD(homePos[30])+DEGToRAD(B[30])*coswave;;// 
    }

    discreteTime +=0.001; // loop runs at 1ms
    if ( discreteTime>Tcycle ) {
        discreteTime=0; //after repeating 1 cycle, reset time to zero
        if ( stop_enable==true ) {
            test1_enable=false;//disable test1
            cout << "Joint range test completed:)" << endl;
        }
    }
}


// this is the funtion that i need to put in the _loop in order to take the kayboard command
void JointTestState ( char cmd ) {
    switch ( cmd ) {
    case '1': {
        if ( test1_enable==true ) {
            cout << "Joint range test is already running:)" << endl;
        } else if ( test2_enable==true ) {
            cout << "Speed test is running. Stop it by pressing 's':)" << endl;
        } else if ( test1_enable==false&&test2_enable==false ) {
            test1_enable=true;//enable test1
            stop_enable=false;
            cout << "Start test of joint range:)" << endl;
        }
    }
    break;

//         case '2':
// 			{
// 				if(test2_enable==true)
// 				{
// 				    cout << "Speed test is already running:)" << endl;
// 				}
//                 else if(test1_enable==true)
// 				{
// 				    cout << "Joint range test is running. Stop it by pressing 's':)" << endl;
// 				}
//                 else if(test2_enable==false&&test1_enable==false)
// 				{
// 				    test2_enable=true;//enable test2
// 				    stop_enable=false;
// 				    cout << "Start speed test:)" << endl;
// 				}
// 			}
// 		break;

    case 's': {
        if ( test1_enable==true||test2_enable==true ) {
            stop_enable=true;
            cout << "Current test will stop after one cycle" << endl;
        }
    }
    break;

    default:
        break;
    }
}
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
