#include <cmath>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include "GlobalExt.h"
#include "RTControl.h"
using namespace std;

extern void Stabilizer(float FTdata[12], double dT, double *deltaHip);

//RT control runs all controllers
void RTControl(double RTtime, float FTSensor[12], vector<float> homingPos, int size, int *pos)
{
    for ( int i=0;i<size;i++ ) {
        homePos[i]=homingPos[i];
    }
    if ( checkMoveToInitial.enable==1&&checkMoveToInitial.done==0 ) {
        moveToInitialPosition(RTtime);// only run once at the beginning
    }
    UpdateGaitParameter();
    GaitPattern();
    TurnRobot();
    if ( checkMoveToInitial.enable==0&&(dtime*Tstep>0.5*Tcycle) ) {//stabilization starts only when moveToInitial finishes;if walking, apply stabilization after Q1
        Stabilizer(FTSensor, Tstep, deltaHip);// pass out waist angles
    }
    InvKHIP(deltaHip); // it only generates the trajectories for leg 12 DOF
    //cout<<deltaHip[0]<<"\t"<<deltaHip[1]<<"\t"<<deltaHip[2]<<endl;
#ifdef EPFL
    JointLimit();
#endif
    WaistControl();
    ArmControl();
    for ( int i=0;i<size;i++ ) {
        pos[i]=int(1e5*traj[i]);// assign the leg trajectory to pos in 10mRad unit
    }
    dloop++;
    dtime++;
}

void WaistControl()
{
    traj[0]=DEGTORAD(homePos[0]);// yaw
    traj[1]=DEGTORAD(homePos[1]);// pitch
    traj[2]=DEGTORAD(homePos[2]);//roll
}

void ArmControl()
{
    traj[15]=DEGTORAD(homePos[15]);// right arm pitch
    traj[16]=DEGTORAD(homePos[16]);// right arm roll
    traj[17]=DEGTORAD(homePos[17]);
    traj[18]=DEGTORAD(homePos[18]);

    traj[19]=DEGTORAD(homePos[19]);// left arm pitch
    traj[20]=DEGTORAD(homePos[20]);// left arm roll
    traj[21]=DEGTORAD(homePos[21]);
    traj[22]=DEGTORAD(homePos[22]);
}

void moveToInitialPosition(double RTtime)
{
    HipP.x[2]=0;
    HipP.y[2]=0;
    /*left foot position*/
    LeftFootP.x[2]=0;
    LeftFootP.y[2]=hip_Comy;
    LeftFootP.z[2]=ankle_height;
    /*right foot position*/
    RightFootP.x[2]=0;
    RightFootP.y[2]=-hip_Comy;
    RightFootP.z[2]=ankle_height;

    double timestartwalk=1.0;
    double A=fullleg-HipP.z[0];

    HipP.z[2]=fullleg+A*(cos(pi/timestartwalk*RTtime)-1.0)/2.0;
    InvKHIP(deltaHip);

    if ( RTtime>timestartwalk ) {
        checkMoveToInitial.enable=0;
        checkMoveToInitial.done=1;
        checkStop.enable=1;
        if ( checkStop.enable==0 ) {
            dtime=0;
            walking_phase[0]=0;//stop.enable=0->start walking, so reset walking phase to zero
            walking_phase[1]=0;//reset walking phase to zero
        }
    }
}

void GaitPattern()
{
    if ( checkEntry.enable && !checkEntry.done && !checkStop.enable ) {// Entry of limit cycle
        checkEntry.enable=Entry();
    } else if ( checkEntry.done && !checkEntry.enable && !checkStop.enable ) {// loop of limit cycle
        GetPhase();
        /*this is the slow down the robot before final DS*/
        if ( commandStop.enable ) {
            Vx_des = (1.0*2*SL/Tcycle);//6*SL is the scaling coefficient, 0.6 corresponds to 0.1 step length
        }
        /**/

        if ( (phase_no[0]==4&&phase_no[1]==3)||(phase_no[0]==2&&phase_no[1]==1) ) {
            if ( checkPreStop.enable ) {
                SL=0;
            }
            numberofsteps++;
            Footx[1]=Footx[1]+SL;
            FootCenter[1]=Footx[1]+offset;

            /*this is for stopping the robot*/
            if ( commandStop.enable&&checkPreStop.enable==0&&checkStop.enable==0 ) {
                checkPreStop.enable=1;
                mark_velocity=Sign(Vy);
                X_ref=FootCenter[1];//last one step uses virtual spring damper to stop COM X motion
            } else if ( checkPreStop.enable==1&&Sign(Vy)==-mark_velocity ) {
                checkPreStop.enable=0;
                checkStop.enable=1;
            }
            /*this is for stopping the robot*/
        }
        if ( !checkStop.enable ) {
            SuppotCenter=FootCenter[1];
            //intermediate variables
            if ( (phase_no[0]==1&&phase_no[1]==4)||(phase_no[0]==4&&phase_no[1]==3) ) {
                Duration = dtime*Tstep - ref_time;
                ref_time = (dtime-1)*Tstep;//update new ref_time
                ref_x=RightFootP.x[2];
                ref_v=(RightFootP.x[2]-RightFootP.x[1])/Tstep;
                ref_a=(RightFootP.x[2]+RightFootP.x[0]-2*RightFootP.x[1])/(Tstep*Tstep);
            } else if ( (phase_no[0]==3&&phase_no[1]==2)||(phase_no[0]==2&&phase_no[1]==1) ) {
                Duration = dtime*Tstep - ref_time;
                ref_time = (dtime-1)*Tstep;//update new ref_time
                ref_x=LeftFootP.x[2];
                ref_v=(LeftFootP.x[2]-LeftFootP.x[1])/Tstep;
                ref_a=(LeftFootP.x[2]+LeftFootP.x[0]-2*LeftFootP.x[1])/(Tstep*Tstep);
            } else {
                //cout << "Entry.m starts enter limit cycle"<<endl;
            }
            COM_new[0]=comx;
            COM_new[1]=comy;
            COM_new[2]=comz+vheight*abs(comy);

            //------------- start to calculate by one controller
            if ( phase_no[1]==1 ) {
                ControlQ1();
            } else if ( phase_no[1]==4 ) {
                ControlQ4();
            } else if ( phase_no[1]==3 ) {
                ControlQ3();
            } else if ( phase_no[1]==2 ) {
                ControlQ2();
            }
        }
        if ( checkStop.enable ) {
            checkStop.done=Stop();
        }
        //--------- end of calculation by 4 phase controllers
    } else if ( checkStop.done==0&&checkStop.enable==1 ) {
        checkStop.done=Stop();
    }
    // numerical integration
    comx = comx + Vx*Tstep + 0.5*Ax*Tstep*Tstep;
    Vx = Vx + Ax*Tstep;
    comy = comy + Vy*Tstep + 0.5*Ay*Tstep*Tstep;
    Vy = Vy + Ay*Tstep;
}

int InitializeWalkState()
{
    Tcycle =LateralGaitF(delta, ds, g, zc);
    SL=0.00;
    SL_des = SL;
    ds=0.03;
    K_sway=1.48;// hip to COM ratio without body
    dtime=0;
    t_ds=0;
    entry_point=0;
    ref_time=0;
    entry_time=0;
    ZMPx_fc=0;
    ZMPy_fc=0;

    Vx_des = 1.2*2*SL/Tcycle;
    xzmp_push=0.05*Vx_des;// now is zero at the entry phase

    walking_phase[0]=0;//only store current and 1 past state
    walking_phase[1]=0;//only store current and 1 past state
    phase_no[0]=0;
    phase_no[1]=0;

    numberofsteps=0;
    Footx[0]=0;
    Footx[1]=0;
    FootCenter[0]=Footx[0]+offset;// 2-element array, 1st is initial value, 2nd is latest value
    FootCenter[1]=Footx[0]+offset;
    SuppotCenter=FootCenter[1];// it should be vector

    /*temprary data storage*/
    temp1=0;
    temp2=0;
    temp3=0;

    //offset = 0.01;  // the offset between foot plate center and ankle position
    comx = comx_offset; // initial com position
    comy = 0;
    comz = zc;

    Vx = 0;
    Vy = 0;
    Vz = 0;
    Ax = 0;
    Ay = 0;
    Az = 0;

    COM_new[0]=comx;
    COM_new[1]=comy;
    COM_new[2]=comz;
    //
    /*------------- useful flags to present the state ---------------*/
    // initialization of state parameters
    checkMoveToInitial.enable=0;// enable move to initial position
    checkMoveToInitial.done=0;

    checkEntry.enable=1;
    checkQ1.enable=0;
    checkQ4.enable=0;
    checkQ3.enable=0;
    checkQ2.enable=0;
    checkStop.enable=0;
    checkPreStop.enable=0;

    checkEntry.done=0;
    checkQ1.done=0;
    checkQ4.done=0;
    checkQ3.done=0;
    checkQ2.done=0;
    checkStop.done=0;
    checkPreStop.done=0;

    commandStop.enable=0;
    commandStop.done=0;

    turnLeft.enable=0;// keyboard to turn
    turnLeft.done=0;// keyboard to turn
    turnRight.enable=0;// keyboard to turn
    turnRight.done=0;// keyboard to turn

    startTurnLeft.enable=0;// keyboard to turn
    startTurnLeft.done=0;// keyboard to turn
    startTurnRight.enable=0;// keyboard to turn
    startTurnRight.done=0;// keyboard to turn
    /*------------- useful flags to present the state ---------------*/

    /*assign the initial value to the position vectors*/
    HipP.x.assign (3,0);
    HipP.y.assign (3,0);
    HipP.z.assign (3,ankle_height+lengthunderleg+lengthupperleg-0.012);//initial hip height
    LeftFootP.x.assign (3,Footx[0]);
    LeftFootP.y.assign (3,step_width*hip_Comy);
    LeftFootP.z.assign (3,ankle_height);
    RightFootP.x.assign (3,Footx[0]);
    RightFootP.y.assign (3,-step_width*hip_Comy);
    RightFootP.z.assign (3,ankle_height);

    LeftFootStancePos.x.assign (3,Footx[0]);
    LeftFootStancePos.y.assign (3,step_width*hip_Comy);
    LeftFootStancePos.z.assign (3,ankle_height);
    RightFootStancePos.x.assign (3,Footx[0]);
    RightFootStancePos.y.assign (3,-step_width*hip_Comy);
    RightFootStancePos.z.assign (3,ankle_height);
    //   /************* above is for the data storage ********************************/
    return 1;
}

void SetInitialFlag()
{
    checkMoveToInitial.enable=1;// enable move to inital position
    checkStop.enable=1;// set the initial state as stop
}


int ControlQ1()
{
    if ( comy<=ds ) {
        desV=Sign(Vy)*sqrt(abs(2*g/zc*delta*(ds-comy)-g/zc*(ds*ds-comy*comy)));
    } else if ( comy>ds ) {
        desV=-Sign(Vy)*sqrt(abs(2*g/zc*delta*(ds-comy)-g/zc*(ds*ds-comy*comy)));
    }

    Ay = Kv*(desV-Vy) - Cv*Vy;//SS
    pseudoZMPy = comy - Ay/g*zc;
    ZMPy_fc+= Kyfc*(pseudoZMPy-ZMPy_fc)*Tstep;

    walking_phase[0]=walking_phase[1];
    walking_phase[1]=1; // left support, arriving3
    //-----  make sure the zmp inside support polygon
    if ( walking_phase[1]==1 ) {
        if ( ZMPy_fc< delta + deltay_minus ) {
            ZMPy = delta + deltay_minus;
        } else if ( ZMPy_fc > delta + deltay_plus ) {
            ZMPy = delta + deltay_plus;
        } else {
            ZMPy = ZMPy_fc;
        }
    } else {
        cout << "unkown in ZMP shaper" << endl;
    }
    if ( numberofsteps!=0&&(comy<=entry_point) ) {// the first entry phase is skipped
        Ay=0;//15 Oct 2012
        ZMPy_fc = comy - Ay/g*zc;
        ZMPy = ZMPy_fc;
    }

    if ( checkPreStop.enable ) {
        Ax = Kpx2*(X_ref-comx)-Kvx2*Vx;
    } else {
        Ax = Bunny*(Vx_des-Vx) +Kpx*(abs(comy)/ds*Ox+(FootCenter[1]-Ox) - comx )- Kvx*Vx;
    }

    if ( numberofsteps==0 ) { // first step is different
        pseudoZMPx = comx-xzmp_push;
    } else {
        pseudoZMPx = comx - Ax/g*zc;
    }
    ZMPx_fc+= Kxfc*(pseudoZMPx-ZMPx_fc)*Tstep;


    if ( ZMPx_fc > (FootCenter[1]+ deltax_plus) ) {
        ZMPx = (FootCenter[1]+ deltax_plus);
    } else if ( ZMPx_fc < (FootCenter[1]+ deltax_minus) ) {
        ZMPx = (FootCenter[1]+ deltax_minus);
    } else {
        ZMPx = ZMPx_fc;
    }
    // recompute Ax Ay after reshaping zmp
    Ax = (comx - ZMPx)/zc*g;
    Ay = (comy - ZMPy)/zc*g;

    //--------------------  Foot trajectory
    if ( walking_phase[0]==0&&walking_phase[1]==1 ) { // now left support
        get_comp[0]=COM_new[0];
        get_comp[1]=COM_new[1];
        get_comp[2]=COM_new[2];
    }

    UpdateFootTraj();

    RightFootP.y[2]=-step_width*hip_Comy;

    if ( numberofsteps==0 ) { // this is the first step
        if ( dtime*Tstep>=t_ds ) {
            RightFootP.x[2]=Poly5(0,0,Footx[0],t_ds,0,0,Footx[0]+SL,0.5*Tcycle-t_ds,dtime*Tstep);
            //RightFootP.z[2]= Poly6(t_ds, 0.5*Tcycle-t_ds+0.03*Tcycle, ankle_height, ankle_height+Lift, ankle_height, dtime*Tstep);
            //RightFootP.x[2] = RightFootP.x[1];
            RightFootP.z[2]= ankle_height;
        } else {    // pseudo DS
            RightFootP.z[2] = ankle_height;
            RightFootP.x[2] = RightFootP.x[1];
        }
    } else { // numberofsteps>0  already in limit cycle
        if ( dtime*Tstep-ref_time>=t_ds ) {
            RightFootP.x[2]=Poly5(0,0,RightFootStancePos.x[2],t_ds+Tstep,0,0,Footx[1]+SL,0.5*Tcycle-t_ds,dtime*Tstep-ref_time);
            RightFootP.z[2]= Poly6(t_ds, 0.5*Tcycle-t_ds-0.02*Tcycle, ankle_height, ankle_height+Lift, ankle_height, dtime*Tstep-ref_time);

            if ( checkPreStop.enable==1 ) {
                RightFootP.x[2]=Poly5(0,0,RightFootStancePos.x[2],t_ds+Tstep,0,0,Footx[1],0.5*Tcycle-t_ds,dtime*Tstep-ref_time);
            }
        } else {    // pseudo DS
            RightFootP.z[2] = ankle_height;
            RightFootP.x[2] = RightFootP.x[1];
        }

    }

    /*stance leg*/
    if ( dtime==0 ) {
        LeftFootP.x[2]=Footx[0];
    } else {
        LeftFootP.x[2]=LeftFootP.x[1];
        LeftFootStancePos.x[2]=LeftFootP.x[2];
    }
    LeftFootP.y[2]=step_width*hip_Comy;
    LeftFootP.z[2]=ankle_height;

    return 1;
}


int ControlQ2()
{
    if ( comy>=-ds ) {
        desV = Sign(Vy)*sqrt(abs(2*delta*g/zc*(ds+comy)-g/zc*(ds*ds-comy*comy)));
    } else if ( comy<-ds ) {
        desV =-Sign(Vy)*sqrt(abs(2*delta*g/zc*(ds+comy)-g/zc*(ds*ds-comy*comy)));
    }

    Ay = Kv*(desV-Vy) - Cv*Vy;//SS
    pseudoZMPy = comy - Ay/g*zc;
    ZMPy_fc+= Kyfc*(pseudoZMPy-ZMPy_fc)*Tstep;

    walking_phase[0]=walking_phase[1];
    walking_phase[1]= -1;   // right single support, departuring
    //estimation of acceleration and ZMP
    //-----  make sure the zmp inside support polygon
    if ( walking_phase[1]==-1 ) {
        //SS
        if ( ZMPy_fc< -(delta+deltay_plus) )
            ZMPy = -(delta+deltay_plus);
        else if ( ZMPy_fc > -(delta+deltay_minus) )
            ZMPy = -(delta+deltay_minus);
        else
            ZMPy = ZMPy_fc;
    } else {
        cout << "unexpected walking phase" << endl;
    }
    if ( comy>-entry_point ) {//DS
        Ay=0;//15 Oct 2012
        ZMPy_fc = comy - Ay/g*zc;
        ZMPy = ZMPy_fc;
    }

    if ( checkPreStop.enable ) {
        Ax = Kpx2*(X_ref-comx)-Kvx2*Vx;
    } else {
        Ax = Bunny*(Vx_des-Vx)+Kpx*(FootCenter[1]+Ox-abs(comy)/ds*Ox - comx )- Kvx*Vx;
    }
    pseudoZMPx = comx - Ax/g*zc;
    ZMPx_fc+= Kxfc*(pseudoZMPx-ZMPx_fc)*Tstep;
    if ( ZMPx_fc > (FootCenter[1]+ deltax_plus) ) {
        ZMPx = (FootCenter[1]+ deltax_plus);
    } else if ( ZMPx_fc < (FootCenter[1]+ deltax_minus) ) {
        ZMPx = (FootCenter[1]+ deltax_minus);
    } else {
        ZMPx = ZMPx_fc;
    }
    // recompute Ax Ay after reshaping zmp
    Ax = (comx - ZMPx)/zc*g;
    Ay = (comy - ZMPy)/zc*g;

    //-------------------- below are Foot trajectory
    UpdateFootTraj();

    LeftFootP.y[2] = step_width*hip_Comy;
    //------------- foot lift height
    if ( dtime*Tstep-ref_time<=Tcycle/4-t_ds ) {
        LeftFootP.z[2] = Poly6(-(Tcycle/4-t_ds), Tcycle/4-t_ds, ankle_height, ankle_height+Lift, ankle_height, dtime*Tstep-ref_time);
        if ( checkPreStop.enable==1 ) {
            LeftFootP.x[2] = Poly5(ref_a,ref_v,ref_x, 0, 0,0,Footx[1],Tcycle/4-t_ds,dtime*Tstep-ref_time);
        } else {
            LeftFootP.x[2] = Poly5(ref_a,ref_v,ref_x,0, 0,0,Footx[1]+SL,Tcycle/4-t_ds,dtime*Tstep-ref_time);
            if ( (LeftFootP.x[2]>Footx[1]+SL)&&(SL>=0) ) {
                LeftFootP.x[2]=Footx[1]+SL;
            } else if ( (LeftFootP.x[2]<Footx[1]+SL)&&(SL<0) ) {
                LeftFootP.x[2]=Footx[1]+SL;
            }
        }
    } else {
        LeftFootP.z[2]=ankle_height;
    }

    //------------- foot lift height
    //right stance leg
    RightFootP.x[2] = RightFootP.x[1];
    RightFootStancePos.x[2]=RightFootP.x[2];
    RightFootP.y[2] = -step_width*hip_Comy;
    RightFootP.z[2] = ankle_height;

    return 1;
}


int ControlQ3()
{
    if ( comy>=-ds ) {
        desV = Sign(Vy)*sqrt(abs(2*delta*g/zc*(ds+comy)-g/zc*(ds*ds-comy*comy)));
    } else if ( comy<-ds ) {
        desV =-Sign(Vy)*sqrt(abs(2*delta*g/zc*(ds+comy)-g/zc*(ds*ds-comy*comy)));
    }
    // estimation of acceleration and ZMP

    Ay = Kv*(desV-Vy) - Cv*Vy;//SS

    pseudoZMPy = comy - Ay/g*zc;
    ZMPy_fc+= Kyfc*(pseudoZMPy-ZMPy_fc)*Tstep;

    walking_phase[0]=walking_phase[1];
    walking_phase[1]=-1;//right single support, arriving
    //---------  make sure the zmp inside support polygon
    if ( walking_phase[1]==-1 ) {//  for double support phase and initial standing phase
        if ( ZMPy_fc< -(delta+deltay_plus) )
            ZMPy = -(delta+deltay_plus);
        else if ( ZMPy_fc > -(delta+deltay_minus) )
            ZMPy = -(delta+deltay_minus);
        else
            ZMPy = ZMPy_fc;
    } else {
        cout << "unexpected walking phase" << endl;
    }
    if ( comy>-entry_point ) {
        Ay=0;//15 Oct 2012
        ZMPy_fc = comy - Ay/g*zc;
        ZMPy = ZMPy_fc;
    }

    //---------------
    if ( checkPreStop.enable ) {
        Ax = Kpx2*(X_ref-comx)-Kvx2*Vx;
    } else {
        Ax = Bunny*(Vx_des-Vx)+Kpx*(abs(comy)/ds*Ox+(FootCenter[1]-Ox) - comx )- Kvx*Vx;
    }
    pseudoZMPx = comx - Ax/g*zc;
    ZMPx_fc+= Kxfc*(pseudoZMPx-ZMPx_fc)*Tstep;

    if ( ZMPx_fc > (FootCenter[1]+ deltax_plus) ) {
        ZMPx = (FootCenter[1]+ deltax_plus);
    } else if ( ZMPx_fc < (FootCenter[1]+ deltax_minus) ) {
        ZMPx = (FootCenter[1]+ deltax_minus);
    } else {
        ZMPx = ZMPx_fc;
    }

    // recompute Ax Ay after reshaping zmp
    Ax = (comx - ZMPx)/zc*g;
    Ay = (comy - ZMPy)/zc*g;
    //-------------------- below are Foot trajectory
    //ref_point = 0;  // set this ref point to 0, dont use it any longer
    UpdateFootTraj();

    LeftFootP.y[2]= step_width*hip_Comy;
    //------------- foot lift height
    if ( dtime*Tstep-ref_time>=t_ds ) {
        LeftFootP.z[2] = Poly6(t_ds, 0.5*Tcycle-t_ds-0.02*Tcycle, ankle_height, ankle_height+Lift, ankle_height, dtime*Tstep-ref_time);
        if ( checkPreStop.enable==1 ) {
            LeftFootP.x[2]=Poly5(0,0,LeftFootStancePos.x[2],t_ds,0,0,Footx[1],Tcycle/2-t_ds,dtime*Tstep-ref_time);
        } else {
            LeftFootP.x[2] = Poly5(0,0,LeftFootStancePos.x[2],t_ds,0,0,Footx[1]+SL,Tcycle/2-t_ds,dtime*Tstep-ref_time);
        }
    } else {
        LeftFootP.z[2]=ankle_height; // x position remains where it is
    }

    //------------- foot lift height
    //right stance leg
    RightFootP.x[2] = RightFootP.x[1];
    RightFootStancePos.x[2]=RightFootP.x[2];
    RightFootP.y[2] = -step_width*hip_Comy;
    RightFootP.z[2] = ankle_height;

    return 1;
}


int ControlQ4()
{
    if ( comy<=ds ) {
        desV=Sign(Vy)*sqrt(abs(2*g/zc*delta*(ds-comy)-g/zc*(ds*ds-comy*comy)));
    } else if ( comy>ds ) {
        desV=-Sign(Vy)*sqrt(abs(2*g/zc*delta*(ds-comy)-g/zc*(ds*ds-comy*comy)));
    }

    // estimation of acceleration and ZMP
    Ay = Kv*(desV-Vy) - Cv*Vy;//SS

    pseudoZMPy = comy - Ay/g*zc;
    ZMPy_fc+= Kyfc*(pseudoZMPy-ZMPy_fc)*Tstep;
    walking_phase[0]=walking_phase[1];
    walking_phase[1] = 1;   // left support
    //--------- make sure the zmp inside support polygon
    if ( walking_phase[1]==1 ) {
        if ( ZMPy_fc< delta+deltay_minus ) {
            ZMPy = delta+deltay_minus;
        } else if ( ZMPy_fc > delta+deltay_plus ) {
            ZMPy = delta+deltay_plus;
        } else {
            ZMPy = ZMPy_fc;
        }
    } else {
        cout << "unexpected walking phase" << endl;
    }
    if ( comy<=entry_point ) {// the first entry phase is skipped
        Ay=0;//15 Oct 2012
        ZMPy_fc = comy - Ay/g*zc;
        ZMPy = ZMPy_fc;
    }

    if ( checkPreStop.enable ) {
        Ax = Kpx2*(X_ref-comx)-Kvx2*Vx;
    } else {
        Ax = Bunny*(Vx_des-Vx)+ Kpx*(FootCenter[1]+Ox-abs(comy)/ds*Ox - comx )- Kvx*Vx;
    }
    pseudoZMPx = comx - Ax/g*zc;
    ZMPx_fc+= Kxfc*(pseudoZMPx-ZMPx_fc)*Tstep;
    if ( ZMPx_fc > (FootCenter[1]+ deltax_plus) ) {
        ZMPx = (FootCenter[1]+ deltax_plus);
    } else if ( ZMPx_fc < (FootCenter[1]+ deltax_minus) ) {
        ZMPx = (FootCenter[1]+ deltax_minus);
    } else {
        ZMPx = ZMPx_fc;
    }
    // recompute Ax Ay after reshaping zmp
    Ax = (comx - ZMPx)/zc*g;
    Ay = (comy - ZMPy)/zc*g;


    // ******** foot trajectory planning **********//
    UpdateFootTraj();

    RightFootP.y[2] = -step_width*hip_Comy;
    //ref_point = 0;  // set this ref point to 0, dont use it any longer
    //------------- foot lift height
    if ( numberofsteps==0 ) {
        if ( dtime*Tstep-ref_time<=Tcycle/4-t_ds ) {
            //RightFootP.z[2]= Poly6(-(ref_time-t_ds), Tcycle/4-t_ds, ankle_height, ankle_height+Lift, ankle_height, dtime*Tstep-ref_time);
            RightFootP.z[2] = ankle_height;
            if ( checkPreStop.enable==1 ) {
                RightFootP.x[2] = Poly5(ref_a,ref_v,ref_x,ref_time, 0,0,Footx[1],ref_time+Tcycle/2-t_ds,dtime*Tstep);
            } else {
                RightFootP.x[2] = Poly5(ref_a,ref_v,ref_x,ref_time, 0,0,Footx[1]+SL,ref_time+Tcycle/4-t_ds,dtime*Tstep);
                if ( (RightFootP.x[2]>Footx[1]+SL)&&(SL>=0) ) {
                    RightFootP.x[2]=Footx[1]+SL;
                } else if ( (RightFootP.x[2]<Footx[1]+SL)&&(SL<0) ) {
                    RightFootP.x[2]=Footx[1]+SL;
                }
            }
        } else {//pseudo DS
            RightFootP.z[2] = ankle_height;
        }
    } else {
        if ( dtime*Tstep-ref_time<=Tcycle/4-t_ds ) {
            RightFootP.z[2]= Poly6(-(Tcycle/4-t_ds), Tcycle/4-t_ds, ankle_height, ankle_height+Lift, ankle_height, dtime*Tstep-ref_time);

            if ( checkPreStop.enable==1 ) {//Q4 is the phase that go to stop phase
                RightFootP.x[2] = Poly5(ref_a,ref_v,ref_x,ref_time, 0,0,Footx[1],ref_time+Tcycle/4-t_ds,dtime*Tstep);
            } else {
                RightFootP.x[2] = Poly5(ref_a,ref_v,ref_x,ref_time, 0,0,Footx[1]+SL,ref_time+Tcycle/4-t_ds,dtime*Tstep);
                if ( (RightFootP.x[2]>Footx[1]+SL)&&(SL>=0) ) {
                    RightFootP.x[2]=Footx[1]+SL;
                } else if ( (RightFootP.x[2]<Footx[1]+SL)&&(SL<0) ) {
                    RightFootP.x[2]=Footx[1]+SL;
                }
            }
        } else {//pseudo DS
            RightFootP.z[2] = ankle_height;
        }
    }
    //------------- foot lift height
    //stance leg
    LeftFootP.x[2] = LeftFootP.x[1];
    LeftFootStancePos.x[2]=LeftFootP.x[2];
    LeftFootP.y[2] = step_width*hip_Comy;
    LeftFootP.z[2] = ankle_height;

    return 1;
}


int Entry()
{
    GetPhase();
    if ( comy<=ds ) {
        desV=Sign(Vy)*sqrt(abs(2*g*delta/zc*(ds-comy)-g/zc*(ds*ds-comy*comy)));
    } else if ( comy>ds ) {
        desV=-Sign(Vy)*sqrt(abs(2*g*delta/zc*(ds-comy)-g/zc*(ds*ds-comy*comy)));
    } else cout << "fail to calculate desV" << endl;

    /*foot trajectory both are stance legs,
    the position doesnt change, so not necessary to update*/
    LeftFootP.x[2]=LeftFootP.x[1];
    LeftFootP.y[2]=LeftFootP.y[1];
    LeftFootP.z[2]=LeftFootP.z[1];
    RightFootP.x[2]=RightFootP.x[1];
    RightFootP.y[2]=RightFootP.y[1];
    RightFootP.z[2]=LeftFootP.z[1];

    Ay = Kv*(desV-Vy) - 0*Vy;   //when start oscillation, use different gains
    pseudoZMPy = comy - Ay/g*zc;

    if ( walking_phase[1]==0 ) {
        //  for double support phase and initial standing phase
        if ( pseudoZMPy<-delta ) {
            ZMPy = -delta;
        } else if ( pseudoZMPy > delta ) {
            ZMPy = delta;
        } else {
            ZMPy = pseudoZMPy;
        }
    } else {
        commandStop.done=1;
        cout << "unkown in ZMP shaper, quit" << endl;
    }
    SuppotCenter = FootCenter[1];
    COM_new[0]=comx;
    COM_new[1]=comy;
    COM_new[2]=comz+vheight*abs(comy);

    pseudoZMPx = comx-xzmp_push;

    ZMPx = pseudoZMPx;

    // recompute Ax Ay after reshaping zmp
    Ax = (comx - ZMPx)/zc*g;
    Ay = (comy - ZMPy)/zc*g;

    if ( Vy<=desV ) {
        // redefine the walking phase after the updating
        //walking_phase[1]=0;	//initial standing
        // this is after calculation, the Vy is updated, should recheck the state
        return 1;
    } else {
        // redefine the walking phase after the updating
        walking_phase[0]=walking_phase[1];
        walking_phase[1]=1;
        //ref_point = comy;
        entry_point = comy;
        entry_time = dtime*Tstep; // this entrytime is recorded only once
        /*Lift_amp = ds-abs(entry_point);*/
        ref_time = dtime*Tstep; // this ref time will be updated later
        checkEntry.done=1;
        if ( p_ds*(Tcycle/4)<entry_time ) { // update t_ds everytime after updating the gait parameters
            t_ds = entry_time;
        } else {
            t_ds = p_ds*(Tcycle/4); // t_ds is the duration in the 1/4 of the cycle
        }
        return 0;
    }
}


int Stop()
{
    double ep;
    double vf;
    double af;

    SuppotCenter=FootCenter[1];
    X_ref=FootCenter[1]-offset+comx_offset;
    walking_phase[0]=walking_phase[1];
    walking_phase[1] =2;

    UpdateFootTraj();

    ep = ankle_height  - RightFootP.z[1];
    vf = ( RightFootP.z[1]- RightFootP.z[0] )/Tstep;
    af = 1200*ep - 300*vf;
    RightFootP.z[2] = RightFootP.z[1] + vf*Tstep + 0.5*af*Tstep*Tstep;

    ep = ankle_height  - LeftFootP.z[1];
    vf = ( LeftFootP.z[1]- LeftFootP.z[0] )/Tstep;
    af = 1200*ep - 300*vf;
    LeftFootP.z[2] =LeftFootP.z[1] + vf*Tstep + 0.5*af*Tstep*Tstep;

    LeftFootP.x[2] = LeftFootP.x[1];
    LeftFootP.y[2] = LeftFootP.y[1];

    RightFootP.x[2] = RightFootP.x[1];
    RightFootP.y[2] = RightFootP.y[1];

    desV = 0;
    Ay = Kp*(rest_p - comy) - Cvv*Vy;

    // estimation of  ZMP
    pseudoZMPy = comy - Ay/g*zc;
    ZMPy_fc+= Kyfc*(pseudoZMPy-ZMPy_fc)*Tstep;
    //  for double support phase and initial standing phase
    if ( ZMPy_fc< -(step_width*hip_Comy+deltay_plus) ) {
        ZMPy = -(step_width*hip_Comy+deltay_plus);
    } else if ( ZMPy_fc > (step_width*hip_Comy+deltay_plus) ) {
        ZMPy = (step_width*hip_Comy+deltay_plus);
    } else {
        ZMPy = ZMPy_fc;
    }

    Ax = Kpx2*(X_ref-comx)-Kvx2*Vx;
    pseudoZMPx = comx - Ax/g*zc;
    ZMPx_fc+= Kxfc*(pseudoZMPx-ZMPx_fc)*Tstep;
    if ( ZMPx_fc > (FootCenter[1]+ 1.0*deltax_plus) ) {
        ZMPx = (FootCenter[1]+ 1.0*deltax_plus);
    } else if ( ZMPx_fc < (FootCenter[1]+ 1.0*deltax_minus) ) {
        ZMPx = (FootCenter[1]+ 1.0*deltax_minus);
    } else {
        ZMPx = ZMPx_fc;
    }

    // recompute Ax Ay after reshaping zmp
    Ax = (comx - ZMPx)/zc*g;
    Ay = (comy - ZMPy)/zc*g;

    return 0;// return to checkStop.done=Stop();
}


void TurnRobot()
{
    /*detect the state*/
    if ( phase_no[0]==4&&phase_no[1]==3 ) {// left swing, to turn left
        if ( turnLeft.done ) {//clear flag done after 1 cycle
            turnLeft.done=0;
        }
        /*check the walking phase, start to turn at the right timing*/
        if ( turnLeft.enable&&!turnLeft.done ) {//1 0
            turnLeft.enable=0;
            turnLeft.done=1;
        }

    } else if ( phase_no[0]==2&&phase_no[1]==1 ) {// right swing, to turn right
        /*check the walking phase, start to turn at the right timing*/
        if ( turnRight.done ) {
            turnRight.done=0;
        }

        if ( turnRight.enable&&!turnRight.done ) {
            turnRight.enable=0;
            turnRight.done=1;
        }
    }

    /*calculate the turning left angles*/
    if ( turnLeft.done ) {   /*at this moment comy<0*/
        int Gain=15.0*(0.02/ds);
        RFtO[2]+=Gain*comy*Tstep;
        LFtO[2]=-RFtO[2];
        /*below is the angle limit*/
        if ( LFtO[2]<0 ) {
            LFtO[2]=0;
        } else if ( LFtO[2]>DEGTORAD(15) ) {
            LFtO[2]=DEGTORAD(15);
        }

        if ( RFtO[2]>0 ) {
            RFtO[2]=0;
        } else if ( RFtO[2]<DEGTORAD(-15) ) {
            RFtO[2]=DEGTORAD(-15);
        }
    } else if ( turnRight.done ) {// turn right
        /*at this moment comy>0*/
        int Gain=15.0*(0.02/ds);
        LFtO[2]+=Gain*comy*Tstep;
        RFtO[2]=-LFtO[2];

        /*below is the angle limit*/
        if ( LFtO[2]<0 ) {
            LFtO[2]=0;
        } else if ( LFtO[2]>DEGTORAD(15) ) {
            LFtO[2]=DEGTORAD(15);
        }

        if ( RFtO[2]>0 ) {
            RFtO[2]=0;
        } else if ( RFtO[2]<DEGTORAD(-15) ) {
            RFtO[2]=DEGTORAD(-15);
        }
    } else { //comments, both methods work fine, but seems has some bugs(unstable WHEN i press key fast)
        //below is method 2 the first order system, simpler amd works
        double xdot;
        double K=15.0/Tcycle;
        xdot=K*(0-LFtO[2]);
        LFtO[2]+=xdot*Tstep;
        RFtO[2]=-LFtO[2];

    }
}


void GetPhase()
{
    if ( Vy>=0&&comy>=0 ) {
        phase_no[0]=phase_no[1];
        phase_no[1]=1;
    } else if ( Vy<0&&comy>=0 ) {
        phase_no[0]=phase_no[1];
        phase_no[1]=4;
    } else if ( Vy<0&&comy<0 ) {
        phase_no[0]=phase_no[1];
        phase_no[1]=3;
    } else if ( Vy>=0&&comy<0 ) {
        phase_no[0]=phase_no[1];
        phase_no[1]=2;
    }
}



void UpdateFootTraj()
{
    LeftFootP.x[0]=LeftFootP.x[1];
    LeftFootP.x[1]=LeftFootP.x[2];

    LeftFootP.y[0]=LeftFootP.y[1];
    LeftFootP.y[1]=LeftFootP.y[2];

    LeftFootP.z[0]=LeftFootP.z[1];
    LeftFootP.z[1]=LeftFootP.z[2];

    RightFootP.x[0]=RightFootP.x[1];
    RightFootP.x[1]=RightFootP.x[2];

    RightFootP.y[0]=RightFootP.y[1];
    RightFootP.y[1]=RightFootP.y[2];

    RightFootP.z[0]=RightFootP.z[1];
    RightFootP.z[1]=RightFootP.z[2];
}


void UpdateGaitParameter()
{
    double ds_v=10*(ds_des-ds);
    ds+=ds_v*Tstep;
    Tcycle =LateralGaitF(delta, ds, g, zc);

    double SL_des_v=2*(SL_des-SL);
    SL+=SL_des_v*Tstep;
    Ox=SL/2;//Ox is half of the step length, needs to be updated as well

    Vx_des = 1.2*2*SL/Tcycle;

    WaistO[1]+=2*(WaistO_des[1]-WaistO[1])/Tcycle*Tstep;
    WaistO[2]+=2*(WaistO_des[2]-WaistO[2])/Tcycle*Tstep;

    vheight+=1/Tcycle*(vheight_des-vheight)*Tstep;
}


void JointLimit()
{
    // Note that 24 25 boards are not used
    // maximum joint angle positive
    vector<float> joint_max = {
        // lower body #15
        20,  45,  25,  35, 35, 2, 45,  105,  65,30,  55, 45, 105,65, 30,
        //  1,  2   ,3,   4,   5,  6,  7,    8,   9, 10, 11, 12, 13, 14, 15
        // upper body #10 right arm to left arm, last 2 are right and left neck
        90, 100,  80, -2,  90, 25,80, -2, 0,  0};
    //  16, 17,  18,  19, 20, 21, 22, 23,24, 25
    // maximum joint angle negative
    vector<float> joint_min = {
        // lower body #15
        -20, -15,-25, -90,-90,-55, -45,  0, -45, -30, -2,-45,  0,-45, -30,
        //  1,  2,  3,   4,  5,  6,   7,   8,  9,  10,  11, 12, 13, 14, 15
        // upper body #10 right arm to left arm, last 2 are right and left neck
        -180,-25,-80,-120,-180,-100,-80,-120,  0,  0};
    //  16, 17,  18, 19, 20,  21,   22, 23,  24, 25

    for ( int i=0;i<25;i++ ) {
        if ( traj[i]> DEGTORAD(joint_max[i]) ) {
            traj[i]=DEGTORAD(joint_max[i]);
        } else if ( traj[i]< DEGTORAD(joint_min[i]) ) {
            traj[i]=DEGTORAD(joint_min[i]);
        }
    }
}



int InitializeDataLog()
{
/******************* the vector for data storage *****************************/
    /*---- bear in mind that the actual time you require is endtime*Tstep,
    but we start from initial condition at 0, that is why we have one more point for
    data storage and Similarly for other variables, be awared! */
    data.comx.assign (1,comx);
    data.comy.assign (1,comy);
    data.comz.assign (1,comz);
    data.Hipx.assign (1,0);
    data.Hipy.assign (1,0);
    data.Hipz.assign (1,0);

    data.phase_no.assign (1,0);
    data.desV.assign (1,0);
    data.Vy.assign (1,0);
    data.Vx.assign (1,0);
    data.zmpx.assign (1,0);
    data.zmpy.assign (1,0);

    /*save the foot trajectory data*/
    data.RightFootx.assign (1,0);
    data.RightFooty.assign (1,0);
    data.RightFootz.assign (1,0);

    data.LeftFootx.assign (1,0);
    data.LeftFooty.assign (1,0);
    data.LeftFootz.assign (1,0);
    /*LeftFootP and RightFootP store the history data of two feet to
    calculate the polynomial. Initialization the array length*/

    /* initial value */
    data.comx[0]=comx;
    data.comy[0]=comy;
    data.comz[0]=comz;
    data.phase_no[0]=phase_no[0];
    data.desV[0]=desV;
    data.Vy[0]=Vy;
    /************* above is for the data storage ********************************/
    return 1;
}

double LateralGaitF(double delta, double ds, double g, double zc)
{
    double x0 = delta - ds;
    double v0 = 0;
    double x1 = delta;
    double v1 = sqrt(2*g/zc*delta*ds - g/zc*ds*ds);
    double Tc = sqrt(zc/g); // w = sqrt(g/zc)
    double tao = Tc*log((x1+Tc*v1)/(x0+Tc*v0));
    double Tcycle = 4*tao;
    return Tcycle;
}


void InverseKinematics(MatrixClass Waist_P,MatrixClass Waist_R,MatrixClass Foot_P,MatrixClass Foot_R,  double hipoffset, double *jointangles)
{
    const double A=lengthupperleg;
    const double B=lengthunderleg;

    MatrixClass Dt, R06T, r;

    Dt(1,1)=0.0;
    Dt(2,1)=hipoffset;
    Dt(3,1)=0;

    R06T=Foot_R.transpose();
    r=R06T*(Waist_P+(Waist_R*Dt)-Foot_P);//the vector points from ankle to hip with respect to local foot orientation frame
    double C;//the distance between hip and ankle joint
    double C_reset;//reset C if C is out of range
    C = sqrt(r(1,1)*r(1,1)+r(2,1)*r(2,1)+r(3,1)*r(3,1));

    double kneeMin=DEGTORAD(1.5);
    double kneeMax=DEGTORAD(120);
    double kneeExtentionMax=sqrt(A*A+B*B-2*A*B*cos(pi-kneeMin));
    double kneeExtentionMin=sqrt(A*A+B*B-2*A*B*cos(pi-kneeMax));

    if ( C>=kneeExtentionMax ) {
        C_reset=kneeExtentionMax;
        jointangles[3]=kneeMin;
    } else if ( C<=kneeExtentionMin ) {
        C_reset=kneeExtentionMin;
        jointangles[3]=kneeMax;
    } else {
        C_reset=C;
        jointangles[3]= pi-acos((A*A+B*B-C_reset*C_reset)/(2.0*A*B));
    }

    double alpha=asin(A*sin(pi-jointangles[3])/C_reset);
    double rxz=sqrt(r(1,1)*r(1,1)+r(3,1)*r(3,1));
    double costheta = C_reset/sqrt(r(1,1)*r(1,1)+r(3,1)*r(3,1))*cos(pi/2-alpha);
    double x=rxz*sqrt(1-costheta*costheta);
    jointangles[4] =atan2(r(2,1),x);

    double y=C_reset*cos(alpha)-x;
    double gama = asin(y*sin(alpha)/rxz);
    jointangles[5] = -(atan2(r(1,1),r(3,1))+gama+alpha);

    /*-- Rfoot=Rbody*Ry(q1)*Rx(q2)*Rz(q3)*Ry(q4)*Ry(q5)*Rx(q6) ----*/
    /*-- Rfoot=Rbody*Ry(q1)*Rx(q2)*Rz(q3)*Ry(q4+q5)*Rx(q6) ----*/
    /*-- => Ry(q1)*Rx(q2)*Rz(q3)= Rbody^T * Rfoot * Rx(q6)^T * Ry(q5)^T *Ry(q4)^T --*/
    /*--or => Ry(q1)*Rx(q2)*Rz(q3)= Rbody^T * Rfoot * Rx(q6)^T * Ry(q5+q4)^T --*/
    /*Note: Rx(q)^T = Rx(-q), so we dont need to transpose, make code easier*/
    MatrixClass R34, R45, R56, Rfoot_hip, R;
    R34(1,1)=cos(jointangles[3]);
    R34(1,2)=0;
    R34(1,3)=sin(jointangles[3]);
    R34(2,1)=0;
    R34(2,2)=1;
    R34(2,3)=0;
    R34(3,1)=-sin(jointangles[3]);
    R34(3,2)=0;
    R34(3,3)=cos(jointangles[3]);
    //R34 = [cos(q4) 0 sin(q4);
    //	0 1 0;
    //	-sin(q4) 0 cos(q4)];% knee
    R45(1,1)=1;
    R45(1,2)=0;
    R45(1,3)=0;
    R45(2,1)=0;
    R45(2,2)=cos(jointangles[4]);
    R45(2,3)=-sin(jointangles[4]);
    R45(3,1)=0;
    R45(3,2)=sin(jointangles[4]);
    R45(3,3)=cos(jointangles[4]);
    //R45 = [1 0 0;
    //	0 cos(q5) -sin(q5);
    //	0 sin(q5) cos(q5)];% ankle roll

    R56(1,1)=cos(jointangles[5]);
    R56(1,2)=0;
    R56(1,3)=sin(jointangles[5]);
    R56(2,1)=0;
    R56(2,2)=1;
    R56(2,3)=0;
    R56(3,1)=-sin(jointangles[5]);
    R56(3,2)=0;
    R56(3,3)=cos(jointangles[5]);
    //R56 = [cos(q6) 0 sin(q6);
    //	0 1 0;
    //	-sin(q6) 0 cos(q6)];% ankle pitch

    Rfoot_hip=Waist_R.transpose()*Foot_R;
    R=Rfoot_hip*R56.transpose()*R45.transpose()*R34.transpose();

    jointangles[0]=atan2(R(1,3),R(3,3));
    jointangles[2]=atan2(R(2,1),R(2,2));
    jointangles[1]=asin(-R(2,3));
}


void InvKHIP(double *deltaHip)
{
    MatrixClass Hip_P, Hip_R, LeftFoot_Pos, LeftFoot_R, RightFoot_Pos, RightFoot_R;
    double jointanglesL[6]={0,0,0,0,0,0};
    double jointanglesR[6]={0,0,0,0,0,0};

    /*-ORIENTATION is typical Roll-Pitch-Yaw configuration (Z-Y-X Euler angles), with respect to global frame, not locale----*/
    /*body orientation, instantanous coordinate, no yaw orientation*/
    Hip_R(1,1)=cos(HipO[2])*cos(HipO[1]);
    Hip_R(1,2)=-sin(HipO[2])*cos(HipO[0])+cos(HipO[2])*sin(HipO[1])*sin(HipO[0]);
    Hip_R(1,3)=sin(HipO[2])*sin(HipO[0])+cos(HipO[2])*sin(HipO[1])*cos(HipO[0]);
    Hip_R(2,1)=sin(HipO[2])*cos(HipO[1]);
    Hip_R(2,2)=cos(HipO[2])*cos(HipO[0])+sin(HipO[2])*sin(HipO[1])*sin(HipO[0]);
    Hip_R(2,3)=-cos(HipO[2])*sin(HipO[0])+sin(HipO[2])*sin(HipO[1])*cos(HipO[0]);
    Hip_R(3,1)=-sin(HipO[1]);
    Hip_R(3,2)=cos(HipO[1])*sin(HipO[0]);
    Hip_R(3,3)=cos(HipO[1])*cos(HipO[0]);

    /*left Foot orientation*/
    LeftFoot_R(1,1)=cos(LFtO[2])*cos(LFtO[1])*cos(LFtO[3])+(-sin(LFtO[2])*cos(LFtO[0])+cos(LFtO[2])*sin(LFtO[1])*sin(LFtO[0]))*sin(LFtO[3]);
    LeftFoot_R(1,2)=-cos(LFtO[2])*cos(LFtO[1])*sin(LFtO[3])+(-sin(LFtO[2])*cos(LFtO[0])+cos(LFtO[2])*sin(LFtO[1])*sin(LFtO[0]))*cos(LFtO[3]);
    LeftFoot_R(1,3)=sin(LFtO[2])*sin(LFtO[0])+cos(LFtO[2])*sin(LFtO[1])*cos(LFtO[0]);
    LeftFoot_R(2,1)=sin(LFtO[2])*cos(LFtO[1])*cos(LFtO[3])+(cos(LFtO[2])*cos(LFtO[0])+sin(LFtO[2])*sin(LFtO[1])*sin(LFtO[0]))*sin(LFtO[3]);
    LeftFoot_R(2,2)=-sin(LFtO[2])*cos(LFtO[1])*sin(LFtO[3])+(cos(LFtO[2])*cos(LFtO[0])+sin(LFtO[2])*sin(LFtO[1])*sin(LFtO[0]))*cos(LFtO[3]);
    LeftFoot_R(2,3)=-cos(LFtO[2])*sin(LFtO[0])+sin(LFtO[2])*sin(LFtO[1])*cos(LFtO[0]);
    LeftFoot_R(3,1)=-sin(LFtO[1])*cos(LFtO[3])+cos(LFtO[1])*sin(LFtO[0])*sin(LFtO[3]);
    LeftFoot_R(3,2)=sin(LFtO[1])*sin(LFtO[3])+cos(LFtO[1])*sin(LFtO[0])*cos(LFtO[3]);
    LeftFoot_R(3,3)=cos(LFtO[1])*cos(LFtO[0]);


    /*right Foot orientation */
    RightFoot_R(1,1)=cos(RFtO[2])*cos(RFtO[1])*cos(RFtO[3])+(-sin(RFtO[2])*cos(RFtO[0])+cos(RFtO[2])*sin(RFtO[1])*sin(RFtO[0]))*sin(RFtO[3]);
    RightFoot_R(1,2)=-cos(RFtO[2])*cos(RFtO[1])*sin(RFtO[3])+(-sin(RFtO[2])*cos(RFtO[0])+cos(RFtO[2])*sin(RFtO[1])*sin(RFtO[0]))*cos(RFtO[3]);
    RightFoot_R(1,3)=sin(RFtO[2])*sin(RFtO[0])+cos(RFtO[2])*sin(RFtO[1])*cos(RFtO[0]);
    RightFoot_R(2,1)=sin(RFtO[2])*cos(RFtO[1])*cos(RFtO[3])+(cos(RFtO[2])*cos(RFtO[0])+sin(RFtO[2])*sin(RFtO[1])*sin(RFtO[0]))*sin(RFtO[3]);
    RightFoot_R(2,2)=-sin(RFtO[2])*cos(RFtO[1])*sin(RFtO[3])+(cos(RFtO[2])*cos(RFtO[0])+sin(RFtO[2])*sin(RFtO[1])*sin(RFtO[0]))*cos(RFtO[3]);
    RightFoot_R(2,3)=-cos(RFtO[2])*sin(RFtO[0])+sin(RFtO[2])*sin(RFtO[1])*cos(RFtO[0]);
    RightFoot_R(3,1)=-sin(RFtO[1])*cos(RFtO[3])+cos(RFtO[1])*sin(RFtO[0])*sin(RFtO[3]);
    RightFoot_R(3,2)=sin(RFtO[1])*sin(RFtO[3])+cos(RFtO[1])*sin(RFtO[0])*cos(RFtO[3]);
    RightFoot_R(3,3)=cos(RFtO[1])*cos(RFtO[0]);

    /*----------Position -----------*/
    /*body position*/
    //each vector of HipP has 3 elements, but only the last one is updated, first 2 are not used here
    Hip_P(1,1)=comx+delta_Hipx+deltaHip[0];
    Hip_P(2,1)=K_sway*comy+deltaHip[1];
    Hip_P(3,1)=HipP.z[2]+deltaHip[2];
    /*left foot position*/
    LeftFoot_Pos(1,1)=LeftFootP.x[2];
    LeftFoot_Pos(2,1)=LeftFootP.y[2];
    LeftFoot_Pos(3,1)=LeftFootP.z[2];
    /*right foot position*/
    RightFoot_Pos(1,1)=RightFootP.x[2];
    RightFoot_Pos(2,1)=RightFootP.y[2];
    RightFoot_Pos(3,1)=RightFootP.z[2];

    InverseKinematics(Hip_P,Hip_R,LeftFoot_Pos,LeftFoot_R,hip_Comy, jointanglesL);
    InverseKinematics(Hip_P,Hip_R,RightFoot_Pos,RightFoot_R,-hip_Comy, jointanglesR);

#ifdef EPFL
//    /*Joint angles in rad for EPFL COMAN*/
    traj[3]=jointanglesR[0];
    traj[4]=jointanglesL[0];
    traj[5]=jointanglesR[1]; //right hip roll compensation
    traj[6]=jointanglesR[2];
    traj[7]=jointanglesR[3];
    traj[8]=jointanglesR[5];//right ankle pitch;
    traj[9]=jointanglesR[4];//right ankle roll;
    traj[10]=jointanglesL[1];// left hip roll
    traj[11]=jointanglesL[2];
    traj[12]=jointanglesL[3];// left knee
    traj[13]=jointanglesL[5];// left ankle pitch
    traj[14]=jointanglesL[4];//left ankle roll;
#else
    // for old COMAN
    traj[3]=(-jointanglesR[0]);
    traj[4]=(-jointanglesL[0])+DEGTORAD(homePos[4]);
    traj[5]=(jointanglesR[1])+DEGTORAD(homePos[5]); //right hip roll compensation
    traj[6]=(jointanglesR[2]);
    traj[7]=(-jointanglesR[3]);
    traj[8]=(-jointanglesR[5]);//right ankle pitch;
    traj[9]=(jointanglesR[4])+DEGTORAD(homePos[9]);//right ankle roll;
    traj[10]=(-jointanglesL[1])+DEGTORAD(homePos[10]);//left hip roll
    traj[11]=(-jointanglesL[2]);
    traj[12]=(-jointanglesL[3]);// left knee
    traj[13]=(-jointanglesL[5])+DEGTORAD(homePos[13]);//left ankle pitch
    traj[14]=(-jointanglesL[4])+0*DEGTORAD(homePos[14]);//left ankle roll;
#endif
}


double Poly5(double a0,double v0,double s0,double t0, double af,double vf,double sf,double tf,double time)
{
    //time is the current time
    double dS;
    double dT;
    double t;
    double k0;
    double k1;
    double k2;
    double k3;
    double k4;
    double k5;
    double s;

    dS = sf - s0;
    dT = tf - t0;
    t = time - t0;
    k0= s0;
    k1= v0;
    k2 = a0/2;
    k3 = ( 20*dS-(8*vf+12*v0)*dT-(3*a0-af)*dT*dT )/(2*dT*dT*dT);
    k4 = (-30*dS+(14*vf+16*v0)*dT+(3*a0-2*af)*dT*dT )/(2*dT*dT*dT*dT);
    k5 = (12*dS-6*(vf+v0)*dT-(a0-af)*dT*dT)/(2*dT*dT*dT*dT*dT);
    s = k0 + k1*t + k2*t*t + k3*t*t*t + k4*t*t*t*t+ k5*t*t*t*t*t;
    return s;
}

double Poly6(double t0, double tf, double z0, double zm, double zf, double time)
{
    // zm is the passby position at the mid of time interval
    double t;
    double dT3;
    double dT4;
    double dT5;
    double dT6;

    double k0;
    double k3;
    double k4;
    double k5;
    double k6;
    double s;

    t = time - t0;
    dT3 = (tf-t0)*(tf-t0)*(tf-t0);
    dT4 = (tf-t0)*dT3;
    dT5 = (tf-t0)*dT4;
    dT6 = (tf-t0)*dT5;

    k0= z0;
//	k1= 0;%v0 is zero here
//	k2 = 0;%a0 is zero here
    k3 = (-42*z0+64*zm-22*zf)/dT3;
    k4 = 3*(37*z0-64*zm+27*zf)/dT4;
    k5 = -6*(17*z0-32*zm+15*zf)/dT5;
    k6=32*(z0-2*zm+zf)/dT6;
    s = k0 + k3*t*t*t + k4*t*t*t*t+ k5*t*t*t*t*t+ k6*t*t*t*t*t*t;
    return s;
    //this is not a general 6 order polynomial, start and end has zero a and v,and pass the middle point
}

int Sign(double x)
{
    if ( x>=0 )
        return 1;
    return -1;
}

void KeyBoardControl(char cmd)
{
    switch ( cmd ) {
        case 'a':
            {
                turnLeft.enable=1;
                turnRight.enable=0;
                cout << "Turn Left" << endl;
            }
            break;

        case 'd':
            {
                turnRight.enable=1;
                turnLeft.enable=0;
                cout << "Turn Right" << endl;
            }
            break;

        case 'q':
            commandStop.done=1;
            cout << "Quit program " << endl;
            break;

        case 's':
            commandStop.enable=1;
            cout << "Stop the robot" << endl;
            break;


        case 'c':
            if ( checkStop.enable ) {
                InitializeWalkState();
                cout << "Start new walking again=)" << endl;
            } else {
                cout << "Warning: robot doesnt stop yet!" << endl;
            }
            break;


        case 'o':
            K_sway-=0.01;
            if ( K_sway<0.5 ) {
                K_sway=0.05;
            }
            cout << "new K_sway is " << K_sway << endl;
            break;

        case 'p':
            K_sway+=0.01;
            if ( K_sway>5 ) {
                K_sway=5;
            }
            cout << "new K_sway is " << K_sway << endl;
            break;

        case 'k':
            delta_Hipx-=0.0005;
            if ( delta_Hipx<-0.10 ) {
                delta_Hipx=-0.10;
            }
            cout << "new delta_Hipx is " << delta_Hipx << endl;
            break;

        case 'l':
            delta_Hipx+=0.0005;
            if ( delta_Hipx>0.08 ) {
                delta_Hipx=0.08;
            }
            cout << "new delta_Hipx is " << delta_Hipx << endl;
            break;


        case 'm':
            ds_des-=0.001;
            if ( ds_des<0.01 ) {
                ds_des=0.01;
            }
            Tcycle =LateralGaitF(delta, ds, g, zc);
            cout << "new ds_des is " << ds_des << endl;
            cout << "new Tcycle is " << Tcycle << endl;
            break;

        case ',':
            ds_des+=0.001;
            if ( ds_des>delta+deltay_minus-0.005 ) {
                ds_des=delta+deltay_minus-0.005;
            }
            Tcycle =LateralGaitF(delta, ds, g, zc);
            cout << "new ds_des is " << ds_des << endl;
            cout << "new Tcycle is " << Tcycle << endl;
            break;

            /*change the step length*/
        case 'u':
            SL_des-=0.005;
            if ( SL_des<-0.04 ) {
                SL_des=-0.04;
            }
            cout << "new SL_des is " << SL_des << endl;
            cout << "real SL is " << SL << endl;
            break;

        case 'i':
            SL_des+=0.005;
            if ( SL_des>0.08 ) {
                SL_des=0.08;
            }
            cout << "new SL_des is " << SL_des << endl;
            cout << "real SL is " << SL << endl;
            break;
            /*change the step length*/


//		case 't':
//			WaistO_des[1]-=0.5;
//			if (WaistO_des[1]<-5)
//			{
//				WaistO_des[1]=-5;
//			}
//			cout << "new Waist Pitch is " << WaistO_des[1] << endl;
//		break;
//
//		case 'y':
//			WaistO_des[1]+=0.5;
//			if (WaistO_des[1]>30)
//			{
//				WaistO_des[1]=30;
//			}
//			cout << "new Waist Pitch is " << WaistO_des[1] << endl;
//		break;

        default:
            break;
    }
}

