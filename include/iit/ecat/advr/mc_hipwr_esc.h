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

#ifndef __IIT_ECAT_ADVR_MC_HIPWR_ESC_H__
#define __IIT_ECAT_ADVR_MC_HIPWR_ESC_H__

#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/advr/esc.h>
#include <iit/ecat/utils.h>
#include <map>

namespace iit {
namespace ecat {
namespace advr {


typedef struct {

    unsigned long Sensor_type;      // Sensor type: NOT USED

    float TorGainP;
    float TorGainI;
    float TorGainD;
    float TorGainFF;

    float Pos_I_lim;                // Integral limit: NOT USED
    float Tor_I_lim;                // Integral limit: NOT USED

    float Min_pos;
    float Max_pos;
    float Max_vel;
    float Max_tor;
    float Max_cur;

    float Enc_offset;
    float Enc_relative_offset;
    float Phase_angle;
    float Torque_lin_coeff;

    uint64_t   Enc_mot_nonius_calib;  
    uint64_t   Enc_load_nonius_calib;  

} HPtFlashParameters;

typedef struct 
{
    char        firmware_version[8];
    uint16_t    ack_board_fault_all;
    float       Direct_ref;
    float       V_batt_filt_100ms;
    float       Board_Temperature;
    float       T_mot1_filt_100ms;
    uint16_t    ctrl_status_cmd;
    uint16_t    ctrl_status_cmd_ack;
    uint16_t    flash_params_cmd;
    uint16_t    flash_params_cmd_ack;
    uint64_t    abs_enc_mot;
    uint64_t    abs_enc_load;     
    float       angle_enc_mot;
    float       angle_enc_load;     
    float       angle_enc_diff;
    float       iq_ref;     
} HPtParameters;

/**
 *  
 **/ 

class HPESC : public McESC
{
public:
    HPESC(const ec_slavet& slave_descriptor) :
           McESC(slave_descriptor) {
    }

    virtual ~HPESC(void) { DPRINTF("~%s %d\n", typeid(this).name(), position); }

public:

    static HPtFlashParameters flash_param;
    static HPtParameters      param;

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
