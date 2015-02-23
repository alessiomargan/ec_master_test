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
    public XDDP_pipe<McEscPdoTypes::pdo_rx,McEscPdoTypes::pdo_tx>,
    public Motor
{
public:
    typedef BasicEscWrapper<McEscPdoTypes,LoPwrEscSdoTypes> Base;
    typedef PDO_log<McEscPdoTypes::pdo_rx>                   Log;
    typedef XDDP_pipe<McEscPdoTypes::pdo_rx,McEscPdoTypes::pdo_tx> Xddp;

    LpESC(const ec_slavet& slave_descriptor) :
        Base(slave_descriptor),
        Log(std::string("/tmp/LpESC_pos"+std::to_string(position)+"_log.txt"),DEFAULT_LOG_SIZE),
        Xddp("LpESC_pos"+std::to_string(position), 8192)
    {

    }
    virtual ~LpESC(void) { 
        delete [] SDOs;
        DPRINTF("~%s %d\n", typeid(this).name(), position);
        print_stat(s_rtt);
    }

    int16_t get_robot_id() {
        //assert(sdo.Joint_robot_id != -1);
        return sdo.Joint_robot_id;
    }

    void print_info(void) {
        DPRINTF("\tJoint id %d\tJoint robot id %d\n", sdo.Joint_number, sdo.Joint_robot_id);
        DPRINTF("\tmin pos %f\tmax pos %f\n", sdo.Min_pos, sdo.Max_pos);
        DPRINTF("\tfw_ver %s\n", sdo.firmware_version);
    }

    virtual const objd_t * get_SDOs() { return SDOs; }

    virtual void init_SDOs(void);

    virtual void on_readPDO(void) {

        if ( rx_pdo.rtt ) {
            rx_pdo.rtt =  get_time_ns() - rx_pdo.rtt;
            s_rtt(rx_pdo.rtt);
        }

        if ( rx_pdo.fault & 0x7FFF) {
            handle_fault();
        }

        // apply transformation from Motor to Joint 
        rx_pdo.position = M2J(rx_pdo.position,_sgn,_offset); 

        if ( _start_log ) {
            push_back(rx_pdo);
        }

        xddp_write(rx_pdo);
    }

    virtual void on_writePDO(void) {

        tx_pdo.ts = get_time_ns();

        // apply transformation from Joint to Motor 
        tx_pdo.pos_ref = J2M(tx_pdo.pos_ref,_sgn,_offset);
    }


    ///////////////////////////////////////////////////////
    ///
    /// Motor method implementation
    ///
    ///////////////////////////////////////////////////////
    virtual bool am_i_HpESC() { return false; }
    virtual bool am_i_LpESC() { return true; }
    virtual uint16_t get_ESC_type() { return LO_PWR_DC_MC; }

    virtual const pdo_rx_t& getRxPDO() const {
        return Base::getRxPDO();
    }
    virtual const pdo_tx_t& getTxPDO() const {
        return Base::getTxPDO();
    }
    virtual void setTxPDO(const pdo_tx_t & pdo_tx) {
        Base::setTxPDO(pdo_tx);
    }

    virtual int init(const YAML::Node & root_cfg) {

        int16_t Joint_robot_id = -1;

        try {
            // !! sgn and offset must set before init_sdo_lookup !!
            init_SDOs();
            init_sdo_lookup();
            getSDO_byname("Joint_robot_id", Joint_robot_id);

        } catch (EscWrpError &e ) {

            DPRINTF("Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what());
            return EC_BOARD_INIT_SDO_FAIL;
        }

        if ( Joint_robot_id > 0 ) {
            try {
                std::string esc_conf_key = std::string("LpESC_"+std::to_string(Joint_robot_id));
                const YAML::Node& esc_conf = root_cfg[esc_conf_key];
                //std::vector<float> conf_pid; 
                if ( esc_conf.Type() != YAML::NodeType::Null ) {
                    esc_conf["sign"] >> _sgn; 
                    esc_conf["pos_offset"] >> _offset;
                    //esc_conf["pid"]["position"] >> conf_pid;
                }
            } catch (YAML::KeyNotFound &e) {
                DPRINTF("Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what());
                return EC_BOARD_KEY_NOT_FOUND;
            }
        } else {
            return EC_BOARD_INVALID_ROBOT_ID;
        }

        log_filename = std::string("/tmp/LpESC_"+std::to_string(sdo.Joint_robot_id)+"_log.txt");


        return EC_WRP_OK;

    }
    virtual int start(int controller_type, float _p, float _i, float _d) {
        return 0;
    }

    virtual int stop(void) {
        return 0;
    }

    virtual void start_log(bool start) {
        Log::start_log(start);
    }

    virtual void handle_fault(void) {

        fault_t fault;
        fault.all = rx_pdo.fault;
        //fault.bit.
        ack_faults_X(this, fault.all);

    }

    /////////////////////////////////////////////
    // set pdo data
    virtual int set_posRef(float joint_pos) {
        tx_pdo.pos_ref = joint_pos;
    }
    virtual int set_torOffs(float tor_offs) {
        tx_pdo.tor_offs = tor_offs;
    }
    virtual int set_posGainP(float p_gain) {
        tx_pdo.PosGainP = p_gain;
    }
    virtual int set_posGainI(float i_gain) {
        tx_pdo.PosGainI = i_gain;
    }
    virtual int set_posGainD(float d_gain) {
        tx_pdo.PosGainD = d_gain;
    }

    virtual void set_off_sgn(float offset, int sgn) {}

private:

    float   _offset;
    int     _sgn;

    stat_t  s_rtt;

    objd_t * SDOs;

};



}
}
}
#endif /* IIT_ECAT_ADVR_ESC_H_ */
