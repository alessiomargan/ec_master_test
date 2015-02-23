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

#include <iit/ecat/ec_master_iface.h>
#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/motor_iface.h>
#include <iit/ecat/advr/log_esc.h>
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


struct HiPwrLogTypes {

    uint64_t	ts;        		    // ns
    float	    pos_ref;   		// rad
    float		tor_offs;
    float		PosGainP;
    float		PosGainI;
    float		PosGainD;
    //                            
    float		temperature; 	// C
    float	    position;   		// rad
    float		velocity;   		// rad/s
    float		torque;     		// Nm
    int32_t     fault;
    uint64_t	rtt;        		// ns

    void fprint(FILE *fp) {
        fprintf(fp, "%lu\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t0x%X\t%lu\n",
                ts,pos_ref,tor_offs,PosGainP,PosGainI,PosGainD,
                temperature,position,velocity,torque,fault,rtt);
    }
    void sprint(char *buff, size_t size) {
        snprintf(buff, size, "%lu\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t0x%X\t%lu\n",
                ts,pos_ref,tor_offs,PosGainP,PosGainI,PosGainD,
                temperature,position,velocity,torque,fault,rtt);
    }
};

/**
*  
**/ 


class HpESC :
    public BasicEscWrapper<McEscPdoTypes,HiPwrEscSdoTypes>,
    public PDO_log<HiPwrLogTypes>,
    public XDDP_pipe<McEscPdoTypes::pdo_rx,McEscPdoTypes::pdo_tx>,
    public Motor
{

public:
    typedef BasicEscWrapper<McEscPdoTypes,HiPwrEscSdoTypes> Base;
    typedef PDO_log<HiPwrLogTypes>                          Log;
    typedef XDDP_pipe<McEscPdoTypes::pdo_rx,McEscPdoTypes::pdo_tx> Xddp;

    HpESC(const ec_slavet& slave_descriptor) :
        Base(slave_descriptor),
        Log(std::string("/tmp/HpESC_pos"+std::to_string(position)+"_log.txt"),DEFAULT_LOG_SIZE),
        Xddp("HpESC_pos"+std::to_string(position), 8192)
    {
        _start_log = false;
        _actual_state = EC_STATE_PRE_OP;
    }

    virtual ~HpESC(void) {

        delete [] SDOs;
        DPRINTF("~%s pos %d\n", typeid(this).name(), position);
        print_stat(s_rtt);
    }

    int16_t get_robot_id() {
        //assert(sdo.Joint_robot_id != -1);
        return sdo.Joint_robot_id;
    }

    void print_info(void) {

        DPRINTF("\tJoint id %c%d\tJoint robot id %d\n", (char)(sdo.Joint_number>>8), sdo.Joint_number&0x0F, sdo.Joint_robot_id);
        DPRINTF("\tmin pos %f\tmax pos %f\n", sdo.Min_pos, sdo.Max_pos);
        DPRINTF("\tfw_ver %s\n", sdo.firmware_version);
    }

protected :

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
            Log::log_t log;
            log.ts = get_time_ns() - _start_log_ts ;
            log.pos_ref     = M2J(tx_pdo.pos_ref,_sgn,_offset);
            log.tor_offs    = tx_pdo.tor_offs;
            log.PosGainP    = tx_pdo.PosGainP;
            log.PosGainI    = tx_pdo.PosGainI;
            log.PosGainD    = tx_pdo.PosGainD;
            // for TEST temperature is motor_enc
            log.temperature = M2J(rx_pdo.temperature,_sgn,_offset);
            //log.temperature = rx_pdo.temperature;
            log.position    = rx_pdo.position;
            log.velocity    = rx_pdo.velocity;
            log.torque      = rx_pdo.torque;  
            log.fault       = rx_pdo.fault;   
            log.rtt         = rx_pdo.rtt;     
            push_back(log);
        }

        xddp_write(rx_pdo);
    }

    virtual void on_writePDO(void) {

        tx_pdo.ts = get_time_ns();
        // apply transformation from Joint to Motor 
        tx_pdo.pos_ref = J2M(tx_pdo.pos_ref,_sgn,_offset);
    }

    virtual int on_readSDO(const objd_t * sdobj)  {

        if ( ! strcmp(sdobj->name, "position") ) {
            rx_pdo.position = M2J(rx_pdo.position,_sgn,_offset);
            //DPRINTF("on_getSDO M2J position %f\n", rx_pdo.position);
        } else if ( ! strcmp(sdobj->name, "Min_pos") ) {
            //DPRINTF("1 on_getSDO M2J min_pos %f\n", sdo.Min_pos);
            sdo.Min_pos = M2J(sdo.Min_pos,_sgn,_offset);
            //DPRINTF("2 on_getSDO M2J min_pos %f\n", sdo.Min_pos);
        } else if ( ! strcmp(sdobj->name, "Max_pos") ) {
            sdo.Max_pos = M2J(sdo.Max_pos,_sgn,_offset);
            //DPRINTF("on_getSDO M2J max_pos %f\n", sdo.Max_pos);
        }
        return EC_BOARD_OK;
    }

    virtual int on_writeSDO(const objd_t * sdo) {

        // do not allow to write sdo that map txPDO
        if ( _actual_state == EC_STATE_OPERATIONAL && sdo->index == 0x7000 ) {
            return EC_WRP_SDO_WRITE_CB_FAIL;
        }
        if ( ! strcmp(sdo->name, "pos_ref") ) {
            tx_pdo.pos_ref = J2M(tx_pdo.pos_ref,_sgn,_offset);
            //DPRINTF("on_setSDO J2M pos_ref %f\n", tx_pdo.pos_ref);
        }
        return EC_BOARD_OK;
    }

    virtual const objd_t * get_SDOs() { return SDOs;}

    void init_SDOs(void);

public :
    ///////////////////////////////////////////////////////
    ///
    /// Motor method implementation
    ///
    ///////////////////////////////////////////////////////
    virtual bool am_i_HpESC() { return true; }
    virtual bool am_i_LpESC() { return false;}
    virtual uint16_t get_ESC_type() {
        if ( product_code == HI_PWR_AC_MC ) return HI_PWR_AC_MC;
        if ( product_code == HI_PWR_DC_MC ) return HI_PWR_DC_MC;
        return NO_TYPE;
    }

    virtual const pdo_rx_t& getRxPDO() const        { return Base::getRxPDO(); }
    virtual const pdo_tx_t& getTxPDO() const        { return Base::getTxPDO(); }
    virtual void setTxPDO(const pdo_tx_t & pdo_tx)  { Base::setTxPDO(pdo_tx); }

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
                std::string esc_conf_key = std::string("HpESC_"+std::to_string(Joint_robot_id));
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

        // redo read SDOs so we can apply _sgn and _offset to transform Min_pos Max_pos to Joint Coordinate 
        readSDO_byname("Min_pos");
        readSDO_byname("Max_pos");
        readSDO_byname("position");

        // set filename with robot_id
        log_filename = std::string("/tmp/HpESC_"+std::to_string(sdo.Joint_robot_id)+"_log.txt");

        // Paranoid Direct_ref
        float direct_ref = 0.0;
        writeSDO_byname("Direct_ref", direct_ref);
        readSDO_byname("Direct_ref", direct_ref);
        assert(direct_ref == 0.0);
            
        return EC_BOARD_OK;
        
    }
    ///////////////////////////////////////////////////////
    /**
     * all done with mailbox 
     * !! in OPERATIONAL DO NOT ALLOW to set_SDO that maps TX_PDO !!
     * @return int 
     */
    virtual int start(int controller_type, float _p, float _i, float _d) {

        float act_position;

        try {
            // ack errors
            // set direct mode and power on modulator
            set_ctrl_status_X(this, CTRL_SET_DIRECT_MODE);
            set_ctrl_status_X(this, CTRL_POWER_MOD_ON);
            // set actual position as reference
            readSDO_byname("position", act_position);
            writeSDO_byname("pos_ref", act_position);
            // set PID gains ... this will set tx_pdo.PosGainP ....
            writeSDO_byname("PosGainP", _p);
            writeSDO_byname("PosGainI", _i);
            writeSDO_byname("PosGainD", _d);

            // set position mode
            set_ctrl_status_X(this, controller_type);
            
        } catch (EscWrpError &e ) {

            DPRINTF("Catch Exception %s ... %s\n", __FUNCTION__, e.what());
            return EC_BOARD_NOK;
        }

        return EC_BOARD_OK;

    }

    virtual int stop(void) {

        return set_ctrl_status_X(this, CTRL_POWER_MOD_OFF);
    }

    virtual void set_off_sgn(float offset, int sgn) {
        _offset = offset;
        _sgn = sgn;
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
