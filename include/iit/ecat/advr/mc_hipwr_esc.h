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


struct HiPwrEscSdoTypes {

    // flash

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

    uint64_t    Enc_mot_nonius_calib;  
    uint64_t    Enc_load_nonius_calib;  

    int16_t     Joint_number;
    int16_t     Joint_robot_id;

    // ram

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

};

/**
 *  
 **/ 

class HpESC :
public BasicEscWrapper<McEscPdoTypes,HiPwrEscSdoTypes>,
public PDO_log<McEscPdoTypes>
{
public:
    typedef BasicEscWrapper<McEscPdoTypes,HiPwrEscSdoTypes> Base;
    typedef PDO_log<McEscPdoTypes>                        Log;

    HpESC(const ec_slavet& slave_descriptor) :
        Base(slave_descriptor),
        Log(std::string("/tmp/HpESC_pos"+std::to_string(position)+"_log.txt"),100000)
    {
        _start_log = false;

        init_SDOs();
        init_sdo_lookup();
        // set filename with robot_id
        log_filename = std::string("/tmp/HpESC_"+std::to_string(sdo.Joint_robot_id)+"_log.txt");
        // Paranoid Direct_ref
        float direct_ref = 0.0;
        set_SDO_byname("Direct_ref", direct_ref);
        get_SDO_byname("Direct_ref", direct_ref);
        assert(direct_ref == 0.0);
    }

    virtual ~HpESC(void) {
        delete [] SDOs;
        DPRINTF("~%s %d\n", typeid(this).name(), position);
    }

    virtual void on_readPDO(void) {
        rx_pdo.rtt =  get_time_ns() - rx_pdo.rtt;
        // apply transformation from Motor to Joint 
        rx_pdo.position = M2J(rx_pdo.position,_sgn,_offset); 
        if (_start_log) {
            push_back(rx_pdo);
        }
    }

    virtual void on_writePDO(void) {
        tx_pdo.ts = get_time_ns();
        // apply transformation from Joint to Motor 
        tx_pdo.pos_ref = J2M(tx_pdo.pos_ref,_sgn,_offset);
    }

    virtual const objd_t * get_SDO_objd() { return SDOs; }

    int16_t get_joint_robot_id() {
        //assert(sdo.Joint_robot_id != -1);
        return sdo.Joint_robot_id;
    }

    void print_info(void) {
        DPRINTF("\nJoint id %c%d\tJoint robot id %d\n", (char)(sdo.Joint_number>>8), sdo.Joint_number&0x0F, sdo.Joint_robot_id);
        DPRINTF("min pos %f\tmax pos %f\n", sdo.Min_pos, sdo.Max_pos);
    }

    void init_SDOs(void);


    void set_off_sgn(float offset, int sgn) {
        _offset = offset;
        _sgn = sgn;
    }

    void start_log(bool start) {
        _start_log = start;
    }
private:
    
    float   _offset;
    int     _sgn;
    bool    _start_log;

    objd_t * SDOs;

};



}
}
}
#endif /* IIT_ECAT_ADVR_ESC_H_ */
