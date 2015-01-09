/*
 * mc_hipwr_esc.h
 * 
 *  HiPower Motor Controlleer 
 *  based on TI TMS320F28335 - Delfino Microcontroller
 *  High-Performance 32-Bit CPU 150 Mhz
 *  
 *  http://www.ti.com/product/tms320f28335
 *  
 *  Created on: Dec 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_MC_LOWPWR_ESC_H__
#define __IIT_ECAT_ADVR_MC_LOWPWR_ESC_H__

#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/advr/esc.h>
#include <iit/ecat/utils.h>
#include <map>

namespace iit {
namespace ecat {
namespace advr {

typedef struct {
    int Block_control;
    int nonius_offset_low;
    float PosGainP;
    float PosGainI;
    float PosGainD;
    float TorGainP;
    float TorGainI;
    float TorGainD;
    float Torque_Mult;
    float Pos_I_lim;
    float Tor_I_lim;
    float Min_pos;
    float Max_pos;
    int nonius_offset_high;
    float Max_tor;
    float Max_cur;
    int Enc_offset_1;
    int Enc_offset_2;
    float Torque_Offset;
    int16_t ConfigFlags;
    int16_t ConfigFlags2;
    int NumEncoderLines;
    float ImpedancePosGainP;
    int nonius_offset2_low;
    float ImpedancePosGainD;
    int Num_Abs_counts_rev;
    int MaxPWM;
    float Gearbox_ratio;
    int ulCalPosition;
    int Cal_Abs_Position;
    int Cal_Abs2_Position;
    int nonius_offset2_high;
    
} LPtFlashParameters;

typedef struct 
{
    char        firmware_version[8];
    uint16_t    ack_board_fault;
    uint16_t    set_ctrl_status;
    uint16_t    get_ctrl_status;
    float       V_batt_filt_100ms;
    float       T_inv_filt_100ms;
    float       T_mot1_filt_100ms;
    uint16_t    flash_params_cmd;
    uint16_t    flash_params_cmd_ack;     
} LPtParameters;

/**
 *  
 **/ 

class LPESC : public McESC
{

public:
    LPESC(const ec_slavet& slave_descriptor) :
           McESC(slave_descriptor) {
    }

    virtual ~LPESC(void) { DPRINTF("~%s %d\n", typeid(this).name(), position); }

public:

    static LPtFlashParameters flash_param;
    static LPtParameters      param;


    static const objd_t SDOs[];
    //
    static const objd_t * SDOs6000;
    static const objd_t * SDOs7000;
    static const objd_t * SDOs8000;
    static const objd_t * SDOs8001;
    
    virtual const objd_t* get_SDOs() { return SDOs; };
    virtual const objd_t* get_SDOs6000() { return SDOs6000; };
    virtual const objd_t* get_SDOs7000() { return SDOs7000; };
    virtual const objd_t* get_SDOs8000() { return SDOs8000; };
    virtual const objd_t* get_SDOs8001() { return SDOs8001; };

};


}
}
}
#endif /* IIT_ECAT_ADVR_ESC_H_ */
