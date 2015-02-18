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
#include <iit/ecat/advr/motor_iface.h>
#include <iit/ecat/advr/log_esc.h>
#include <iit/ecat/utils.h>
#include <map>

namespace iit {
namespace ecat {
namespace advr {

struct LoPwrEscSdoTypes {
    
    // flash

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

    int16_t     Joint_number;
    int16_t     Joint_robot_id;

    // ram

    char        firmware_version[8];
    uint16_t    ack_board_fault;
    uint16_t    set_ctrl_status;
    uint16_t    get_ctrl_status;
    float       V_batt_filt_100ms;
    float       T_inv_filt_100ms;
    float       T_mot1_filt_100ms;
    uint16_t    flash_params_cmd;
    uint16_t    flash_params_cmd_ack;     
};

/**
 *  
 **/ 

class LpESC :
public BasicEscWrapper<McEscPdoTypes,LoPwrEscSdoTypes>,
public PDO_log<McEscPdoTypes::pdo_rx>,
public Motor
{
public:
    typedef BasicEscWrapper<McEscPdoTypes,LoPwrEscSdoTypes> Base;
    typedef PDO_log<McEscPdoTypes::pdo_rx>                   Log;

    LpESC(const ec_slavet& slave_descriptor) :
        Base(slave_descriptor),
        Log(std::string("/tmp/LpESC_pos"+std::to_string(position)+"_log.txt"),DEFAULT_LOG_SIZE)
    {

    }
    virtual ~LpESC(void) { 
        delete [] SDOs;
        DPRINTF("~%s %d\n", typeid(this).name(), position);
    }

    int16_t get_joint_robot_id() {
        //assert(sdo.Joint_robot_id != -1);
        return sdo.Joint_robot_id;
    }

    void print_info(void) {
        DPRINTF("\n min pos %f\tmax pos %f \n", sdo.Min_pos, sdo.Max_pos);
    }

    virtual const objd_t * get_SDO_objd() { return SDOs; }

    virtual void init_SDOs(void);

    ///////////////////////////////////////////////////////
    ///
    /// Motor method implementation
    ///
    ///////////////////////////////////////////////////////
    virtual bool am_i_HpESC() { return false; }
    virtual bool am_i_LpESC() { return true; }
    virtual uint16_t get_ESC_type() { return LO_PWR_DC_MC; }

    virtual int init(const YAML::Node & root_cfg) {

        try {
            init_SDOs();
            init_sdo_lookup();
            // set filename with robot_id
            log_filename = std::string("/tmp/LpESC_"+std::to_string(sdo.Joint_robot_id)+"_log.txt");

        } catch (EscWrpError &e ) {

            DPRINTF("Catch Exception %s ... %s\n", __FUNCTION__, e.what());
            return EC_WRP_NOK;
        }

        return EC_WRP_OK;

    }
    virtual int start(int controller_type, float _p, float _i, float _d) {
        return 0;
    }

    virtual int stop(void) {
        return 0;
    }

    virtual void handle_fault(void) {

        fault_t fault;
        fault.all = rx_pdo.fault;
        //fault.bit.
        ack_faults_X(this, fault.all);

    }

    virtual int set_posRef(float joint_pos) {
        return 0;
    }
    virtual int set_torOffs(float tor_offs) {
        return 0;
    }

    virtual int set_posGainP(float p_gain) {
        return 0;
    }
    virtual int set_posGainI(float i_gain) {
        return 0;
    }
    virtual int set_posGainD(float d_gain) {
        return 0;
    }

    virtual void set_off_sgn(float offset, int sgn) {}
    virtual void start_log(bool start) {}

private:
    objd_t * SDOs;

};



}
}
}
#endif /* IIT_ECAT_ADVR_ESC_H_ */
