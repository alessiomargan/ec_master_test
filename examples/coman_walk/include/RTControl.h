#ifndef _RTCONTROL_H
#define _RTCONTROL_H

#include "Matrix.h"
#include <vector>
#include <iomanip>
using namespace std;
void RTControl ( double RTtime, float FTSensor[], std::vector<float> homingPos, int size, float *pos );
void WaistControl();
void ArmControl();
void moveToInitialPosition ( double RTtime );
void GaitPattern();
int Entry();
int InitializeWalkState();
void SetInitialFlag();
int InitializeDataLog();
int ControlQ1();
int ControlQ4();
int ControlQ3();
int ControlQ2();
int Stop();

void GetPhase();
int Sign ( double );

double Poly5 ( double,double,double,double, double,double,double,double,double );

double LateralGaitF ( double,double,double,double );

void InverseKinematics ( MatrixClass,MatrixClass,MatrixClass,MatrixClass,double, double * );

void IKTrajectory();

void InvKHIP ( double * );

void UpdateFootTraj();

void KeyBoardControl ( char );

void TurnRobot();

void UpdateGaitParameter();

double Poly6 ( double, double, double, double, double, double );

void JointCompensator ( float link_encoder[15] );

void JointLimit();


#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
