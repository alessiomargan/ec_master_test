#ifndef _STABILIZER_H// header guards
#define _STABILIZER_H

/*------------ for stabilizer --------------*/
const int samples1=60;  //filter size
const int samples2=200;
const int samples3=300;
double FT_buffer[12][samples2]={{0},{0}};       //Force circular buffer
double FT_filter[12]={0};       //filtered Force data
double ZMP_buffer[2][samples2]={{0},{0}};       //raw ZMP circular buffer
double ZMP[2]={0};// filtered ZMP
double dZMP_buffer[2][samples2]={{0},{0}};//derevative of filtered ZMP circular buffer
double dZMP[2]={0};// zmp velocity

double deltaZMPx=0;// zmp error
double deltaZMPy=0;

double sum0=0;
double sum[2]={0};
double FTsum[12]={0};

/*	for zmp calculation */
double pxr=0;
double pyr=0;
double pxl=0;
double pyl=0;
double px=0;
double py=0;
double pz=0.0015;

double px_low=0;// low frequency component of filtered ZMP
double py_low=0;
double px_compst=0;
double py_compst=0;
double zmpx_old=0;
double zmpy_old=0;

double Fzmin=10; //Fz threshold
double Fz[2]={0}; //Fz[0] left foot vertical force, Fz[1] right foot vertical force

double anti_G=0;
/*	robot parameter */
double G=15.5*9.81;// 31.3 kg for full body COMAN
double scale=0;
#endif

#include "GlobalExt.h"

void Stabilizer(float FTdata[12], double dT, double *deltaHip) //pass deltahip out
{
    // load/update the Force Measurement circular buffer
    for ( int j=0;j<12;j++ ) {
        for ( int i=0;i<samples2-1;i++ ) {
            FT_buffer[j][i] = FT_buffer[j][i+1];
        }
        FT_buffer[j][samples2-1] = FTdata[j];  //then push back the latest value into array
    }

    for ( int j=0;j<12;j++ ) { // reset zero
        FTsum[j] = 0;//I do it in such a way so that it is easy to plug this code somewhere-else, you just need to pass in 1D array
    }
    // filtered the FT data
    for ( int j=0;j<12;j++ ) {
        for ( int i=0;i<samples1;i++ ) { // samples1<samples2
            FTsum[j] += FT_buffer[j][samples2-1-i];
        }
        FT_filter[j] = FTsum[j]/samples1;
        FTsum[j] = 0;
    }

    //begin computation of ZMP based on force without filtering
    // right foot cop
    if ( FT_buffer[2][samples2-1]>Fzmin ) {
        pxr=(-FT_buffer[4][samples2-1]-pz*FT_buffer[0][samples2-1])/FT_buffer[2][samples2-1];
        pyr=(FT_buffer[3][samples2-1]-pz*FT_buffer[1][samples2-1])/FT_buffer[2][samples2-1];
        if ( pxr>0.14 ) {
            pxr=0.14;
        } else if ( pxr< -0.06 ) {
            pxr= -0.06;
        }
        if ( pyr>0.05 ) {
            pyr=0.05;
        } else if ( pyr<-0.05 ) {
            pyr=-0.05;
        }
    } else {
        pxr=0;
        pyr=0;
    }
    // left foot cop
    if ( FT_buffer[8][samples2-1]>Fzmin ) {
        pxl=(-FT_buffer[10][samples2-1]-pz*FT_buffer[6][samples2-1])/FT_buffer[8][samples2-1];
        pyl=(FT_buffer[9][samples2-1]-pz*FT_buffer[7][samples2-1])/FT_buffer[8][samples2-1];

        if ( pxl>0.14 ) {
            pxl=0.14;
        } else if ( pxl< -0.06 ) {
            pxl= -0.06;
        }
        if ( pyl>0.05 ) {
            pyl=0.05;
        } else if ( pyl<-0.05 ) {
            pyl=-0.05;
        }
    } else {
        pxl=0;
        pyl=0;
    }
    // overall COP
    if ( FT_buffer[2][samples2-1]>Fzmin && FT_buffer[8][samples2-1]>Fzmin ) {//double support
        px=(FT_buffer[2][samples2-1]*pxr+FT_buffer[8][samples2-1]*pxl)/(FT_buffer[2][samples2-1]+FT_buffer[8][samples2-1]);
        py=(FT_buffer[2][samples2-1]*(pyr-0.05)+FT_buffer[8][samples2-1]*(pyl+0.05))/(FT_buffer[2][samples2-1]+FT_buffer[8][samples2-1]);

        /*get filtered vertical force*/
        Fz[0]+=2.5*(FT_buffer[8][samples2-1]-Fzmin -Fz[0])*dT;//left
        Fz[1]+=2.5*(FT_buffer[2][samples2-1]-Fzmin -Fz[1])*dT;//right
    } else if ( FT_buffer[2][samples2-1]>Fzmin&&FT_buffer[8][samples2-1]<=Fzmin ) {//right single support
        px=pxr;
        py=pyr-0.05;

        /*get filtered vertical force*/
        Fz[0]+=2.5*(0 -Fz[0])*dT;//left
        Fz[1]+=2.5*(FT_buffer[2][samples2-1]-Fzmin -Fz[1])*dT;//right

    } else if ( FT_buffer[2][samples2-1]<=Fzmin&&FT_buffer[8][samples2-1]>Fzmin ) {//left single support
        px=pxl;
        py=pyl+0.05;
        /*get filtered vertical force*/
        Fz[0]+=2.5*(FT_buffer[8][samples2-1]-Fzmin -Fz[0])*dT;//left
        Fz[1]+=2.5*(0 -Fz[1])*dT;//right
    } else {
        px=0;
        py=0;
        /*get filtered vertical force*/
        Fz[0]+=2.5*(0 -Fz[0])*dT;//left
        Fz[1]+=2.5*(0 -Fz[1])*dT;//right
    }
    //end of computation of ZMP based on force without filtering


    // begin of pushing ZMP buffer
    for ( int j=0;j<2;j++ ) {//remember here the buffer is 2row, for overall zmp
        for ( int i=0;i<samples2-1;i++ ) {
            ZMP_buffer[j][i] = ZMP_buffer[j][i+1];//
        }
    }
    //then push back the latest value into array
    ZMP_buffer[0][samples2-1] = px;
    ZMP_buffer[1][samples2-1] = py;
    // end of pushing ZMP buffer

    // begin of ZMP filtering
    sum0=0;
    sum[0]=0;
    sum[1]=0;
    for ( int i=samples2;i>0;i-- ) {
        sum0+=i;
        sum[0] += i*ZMP_buffer[0][i-1];//ZMP X
        sum[1] += i*ZMP_buffer[1][i-1];//ZMP Y
    }
    ZMP[0]=sum[0]/sum0;// filtered ZMP X
    ZMP[1]=sum[1]/sum0;//filtered ZMP Y

    /*derevative of filtered COP */
    for ( int i=0;i<samples2-1;i++ ) {
        dZMP_buffer[0][i]=dZMP_buffer[0][i+1];
        dZMP_buffer[1][i]=dZMP_buffer[1][i+1];
    }//this shifts the data in array
    dZMP_buffer[0][samples2-1]=(ZMP[0]-zmpx_old)/dT;
    dZMP_buffer[1][samples2-1]=(ZMP[1]-zmpy_old)/dT;
    zmpx_old=ZMP[0];
    zmpy_old=ZMP[1];

    // begin of ZMP derevative filtering
    sum0=0;
    sum[0]=0;
    sum[1]=0;
    for ( int i=samples2;i>0;i-- ) {
        sum0+=i;
        sum[0] += i*dZMP_buffer[0][i-1];//ZMP X
        sum[1] += i*dZMP_buffer[1][i-1];//ZMP Y
    }
    dZMP[0]=sum[0]/sum0;// filtered ZMP X
    dZMP[1]=sum[1]/sum0;//filtered ZMP Y

    /*    desired equilibrium   */
    float px_des=0.005;
    float py_des= 0.00;

    deltaZMPx = ZMP[0]-px_des;
    deltaZMPy = ZMP[1]-py_des;

    // important to do!!
    double Tf1=1.0/3.0; // T=1/f for filter;
    double Tf2=1.0/0.1; // T=1/f for filter;

    if ( (Fz[0]+Fz[1])>2*Fzmin ) {//normalized unit based on GRF/mg
        scale = (Tf1*scale+dT*1.0)/(Tf1+dT);//if load on the ground, coefficient is 1
    } else {
        scale = (Tf2*scale+dT*0.0)/(Tf2+dT);
    }
    // Must normalize!!

    /*************      for static compensation       ************************/
    px_low += 4*(ZMP[0]-px_low)*dT;
    py_low += 4*(ZMP[1]-py_low)*dT;
    float Kvpx=0.5*scale;//0.25*n1, Kvpx=0.8;
    float Kvpy=0.0*scale;//0.2*n1, Kvpy=0.5;
    //Note is px_filtered is to get the low frequency single from measured ZMP to do compensation, should NOT respond to high frequency ZMP signal
    if ( (px==0||py==0)||(Kvpx==0&&Kvpy==0) ) {
        px_compst += 5*(0-px_compst)*dT;
        py_compst += 5*(0-py_compst)*dT;
    } else {
        px_compst += Kvpx*(px_des-px_low)*dT;
        py_compst += Kvpy*(py_des-py_low)*dT;
    }
    if ( px_compst>0.01 ) {
        px_compst=0.01;
    } else if ( px_compst<-0.020 ) {
        px_compst=-0.020;
    }
    if ( py_compst>0.05 ) {
        py_compst=0.05;
    } else if ( py_compst<-0.05 ) {
        py_compst=-0.05;
    }
    /*************      for static compensation       ************************/

    //---- stabilizer law: gain K must < 1 to have negative feedback control
    double Kx=0.20*scale;//Kx=0.15 for EPFL
    double Ky=0.40*scale;//Ky=0.20 for EPFL
    double Cx=-0.01*scale;//-0.02 for EPFL
    double Cy= -0.02*scale;//-0.01 for EPFL
    /*-------------- here is the main law 1 -------------------*/
    deltaHip[0] = Kx*deltaZMPx +  Cx*dZMP[0] + 0.0*px_compst;//delta hip x we do not use integral terms now
    deltaHip[1] = Ky*deltaZMPy +  Cy*dZMP[1] + 0.0*py_compst;//delta hip y

    double Fext= 1-(FT_filter[2]+FT_filter[8])/G; // include the bias force to counterbalance gravity
    double Kz = 10.0;   // stiffness normalized by mass
    double Cz = 9.0;    // for unit mass

    deltaHip[2] = scale*(Fext*dT + Cz*deltaHip[2])/(Kz*dT+Cz);//delta hip z

    // put some threshold for the hip position modification
    if ( deltaHip[2]>0.005 ) {
        deltaHip[2]=0.005;
    } else if ( deltaHip[2]<-0.1 ) {
        deltaHip[2]=-0.1;
    }

    if ( deltaHip[0]>0.10 ) {
        deltaHip[0]=0.10;
    } else if ( deltaHip[0]<-0.10 ) {
        deltaHip[0]=-0.10;
    }

    if ( deltaHip[1]>0.15 ) {
        deltaHip[1]=0.15;
    } else if ( deltaHip[1]<-0.15 ) {
        deltaHip[1]=-0.15;
    }
}
