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

namespace hipwr_esc {
static float J2M(float p, int s, float o) { return (M_PI - (s*o) + (s*p)); } 
static float M2J(float p, int s, float o) { return ((p - M_PI + (s*o))/s); } 
//static float J2M(float p, int s, float o) { return p; } 
//static float M2J(float p, int s, float o) { return p; } 
}
    

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

    float PosGainP;
    float PosGainI;
    float PosGainD;
    // ram

    char        firmware_version[8];
    uint32_t    board_enable_mask;
    float       Direct_ref;
    float       V_batt_filt_100ms;
    float       Board_Temperature;
    float       T_mot1_filt_100ms;
    uint16_t    ctrl_status_cmd;
    uint16_t    ctrl_status_cmd_ack;
    uint16_t    flash_params_cmd;
    uint16_t    flash_params_cmd_ack;
    int32_t    abs_enc_mot;
    int32_t    abs_enc_load;     
    float       angle_enc_mot;
    float       angle_enc_load;     
    float       angle_enc_diff;
    float       iq_ref;
    
};


struct HiPwrLogTypes {
    
//     // TX
//     float       pos_ref;    //link
//     int16_t     vel_ref;    //link
//     int16_t     tor_ref;    //link
//     uint16_t    gains[5];
//     uint16_t    fault_ack;
//     uint16_t    ts;
//     uint16_t    op_idx_aux; // op [get/set] , idx
//     float       aux_set;    // set value
//     
//     // RX
//     float        link_pos;           // rad
//     float        motor_pos;          // rad
//     float        link_vel;           // radTBD on the firmware 
//     int16_t      motor_vel;          // rad/s
//     int16_t      torque;             // Nm
//     uint16_t     max_temperature;    // C
//     uint16_t     fault;
//     uint16_t     rtt;                // us
//     uint16_t     op_idx_ack;         // op [ack/nack] , idx
//     float        aux_get;            // get value or nack erro code
//         
// 
//     void fprint(FILE *fp) {
//         fprintf(fp, "%f\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t0x%X\t%d\t%d\t%f%f\t%f\t%f\t%d\t%d\t%d\t0x%X\t%d\t%d\t%f\t\n", 
//                     // TX
//                     pos_ref,vel_ref,tor_ref,
//                     gains[0],gains[1],gains[2],gains[3],gains[4],
//                     fault_ack,ts,op_idx_aux,aux_set,
//                     // RX
//                     link_pos,motor_pos,link_vel,motor_vel,
//                     torque,max_temperature,fault,rtt, op_idx_ack,aux_get);
//     }
//     int sprint(char *buff, size_t size) {
//         return snprintf(buff, size, "%f\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t0x%X\t%d\t%d\t%f%f\t%f\t%f\t%d\t%d\t%d\t0x%X\t%d\t%d\t%f\t\n", 
//                                     // TX
//                                     pos_ref,vel_ref,tor_ref,
//                                     gains[0],gains[1],gains[2],gains[3],gains[4],
//                                     fault_ack,ts,op_idx_aux,aux_set,
//                                     // RX
//                                     link_pos,motor_pos,link_vel,motor_vel,
//                                     torque,max_temperature,fault,rtt, op_idx_ack,aux_get);
//     }
//         
    
    uint64_t                ts_rx;           // ns
    uint64_t                ts_tx;           // ns
    McEscPdoTypes::pdo_rx   rx_pdo;
    McEscPdoTypes::pdo_tx   tx_pdo;


    void fprint(FILE *fp) {
            fprintf(fp, "%lu\t", ts_rx);
            fprintf(fp, "%lu\t", ts_tx);
            rx_pdo.fprint(fp);
            tx_pdo.fprint(fp);
        }
    int sprint(char *buff, size_t size) {
        int l = snprintf(buff, size, "%lu\t%lu\t", ts_rx, ts_tx);
        int l_rx = rx_pdo.sprint(buff+l,size-l); 
        return l + l_rx + tx_pdo.sprint(buff+l+l_rx, size-l-l_rx);
    }
       
};

/**
*  
**/ 


class HpESC :
    public BasicEscWrapper<McEscPdoTypes,HiPwrEscSdoTypes>,
    public PDO_log<HiPwrLogTypes>,
    public Motor
{

public:
    typedef BasicEscWrapper<McEscPdoTypes,HiPwrEscSdoTypes> Base;
    typedef PDO_log<HiPwrLogTypes>                          Log;
    
    HpESC(const ec_slavet& slave_descriptor) :
        Base(slave_descriptor),
        Log(std::string("/tmp/HpESC_pos"+std::to_string(position)+"_log.txt"),DEFAULT_LOG_SIZE)
    {
        _start_log = false;
        //_actual_state = EC_STATE_PRE_OP;
    }

    virtual ~HpESC(void) {

        delete [] SDOs;
        DPRINTF("~%s pos %d\n", typeid(this).name(), position);
        print_stat(s_rtt);
    }

    virtual int16_t get_robot_id() {
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
            rx_pdo.rtt =  (uint16_t)(get_time_ns()/1000 - rx_pdo.rtt);
            s_rtt(rx_pdo.rtt);
        }

        if ( rx_pdo.fault & 0x7FFF) {
            handle_fault();
        } else {
            // clean any previuos fault ack !! 
            tx_pdo.fault_ack = 0;
        }

        // apply transformation from Motor to Joint 
        rx_pdo.link_pos = hipwr_esc::M2J(rx_pdo.link_pos,_sgn,_offset); 
        rx_pdo.motor_pos = hipwr_esc::M2J(rx_pdo.motor_pos,_sgn,_offset); 
        
//         // TX
//         float       pos_ref;    //link
//         int16_t     vel_ref;    //link
//         int16_t     tor_ref;    //link
//         uint16_t    gains[5];
//         uint16_t    fault_ack;
//         uint16_t    ts;
//         uint16_t    op_idx_aux; // op [get/set] , idx
//         float       aux_set;    // set value
//         
//         // RX
//         float        link_pos;           // rad
//         float        motor_pos;          // rad
//         float        link_vel;           // radTBD on the firmware 
//         int16_t      motor_vel;          // rad/s
//         int16_t      torque;             // Nm
//         uint16_t     max_temperature;    // C
//         uint16_t     fault;
//         uint16_t     rtt;                // us
//         uint16_t     op_idx_ack;         // op [ack/nack] , idx
//         float        aux_get;            // get value or nack erro code
//         
//         if ( _start_log ) {
//             Log::log_t log;
//             // TX
//             log.ts = get_time_ns() - _start_log_ts ;
//             log.pos_ref     =   hipwr_esc::M2J(tx_pdo.pos_ref,_sgn,_offset);
//             log.vel_ref     =   tx_pdo.vel_ref;
//             log.tor_ref     =   tx_pdo.tor_ref;
//             log.gains[0]    =   tx_pdo.gains[0];
//             log.gains[1]    =   tx_pdo.gains[1];
//             log.gains[2]    =   tx_pdo.gains[2];
//             log.gains[3]    =   tx_pdo.gains[3];
//             log.gains[4]    =   tx_pdo.gains[4];
//             log.fault_ack   =   tx_pdo.fault_ack;
//             log.ts          =   tx_pdo.ts;
//             log.op_idx_aux  =   tx_pdo.op_idx_aux;
//             log.aux_set     =   tx_pdo.aux;
//             // RX
//             log.link_pos        = rx_pdo.link_pos;
//             log.motor_pos       = rx_pdo.motor_pos;
//             log.link_vel        = rx_pdo.link_vel;
//             log.link_vel        = rx_pdo.link_vel;
//             log.max_temperature = rx_pdo.max_temperature;  
//             log.fault           = rx_pdo.fault;   
//             log.rtt             = rx_pdo.rtt;     
//             log.op_idx_aux      = rx_pdo.op_idx_ack;
//             log.aux_get         = rx_pdo.aux;
//             push_back(log);
//         }
        
        if ( _start_log ) {
            log.ts_rx = get_time_ns() - _start_log_ts ;
            log.rx_pdo = rx_pdo;
        }

    }

    virtual void on_writePDO(void) {

        tx_pdo.ts = (uint16_t)(get_time_ns()/1000);
        // apply transformation from Joint to Motor 
        //tx_pdo.pos_ref = J2M(tx_pdo.pos_ref,_sgn,_offset);
        
        if ( _start_log ) {
            log.ts_tx = get_time_ns() - _start_log_ts ;
            log.tx_pdo = tx_pdo;
            push_back(log);             // NOTE push the log on TX
        }
    }

    virtual int on_readSDO(const objd_t * sdobj)  {

        if ( ! strcmp(sdobj->name, "link_pos") ) {
            rx_pdo.link_pos = hipwr_esc::M2J(rx_pdo.link_pos,_sgn,_offset);
            //DPRINTF("on_getSDO M2J link_pos %f\n", rx_pdo.link_pos);
        } else if ( ! strcmp(sdobj->name, "motor_pos") ) {
            rx_pdo.motor_pos = hipwr_esc::M2J(rx_pdo.motor_pos,_sgn,_offset);
        } else if ( ! strcmp(sdobj->name, "Min_pos") ) {
            sdo.Min_pos = hipwr_esc::M2J(sdo.Min_pos,_sgn,_offset);
        } else if ( ! strcmp(sdobj->name, "Max_pos") ) {
            sdo.Max_pos = hipwr_esc::M2J(sdo.Max_pos,_sgn,_offset);
        }
        return EC_BOARD_OK;
    }

    virtual int on_writeSDO(const objd_t * sdo) {

        // do not allow to write sdo that map txPDO
        //if ( _actual_state == EC_STATE_OPERATIONAL && sdo->index == 0x7000 ) {
        //    return EC_WRP_SDO_WRITE_CB_FAIL;
        //}
        if ( ! strcmp(sdo->name, "pos_ref") ) {
            tx_pdo.pos_ref = hipwr_esc::J2M(tx_pdo.pos_ref,_sgn,_offset);
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

        Joint_robot_id = -1;

        try {
            // !! sgn and offset must set before init_sdo_lookup !!
            init_SDOs();
            init_sdo_lookup();
            readSDO_byname("Joint_robot_id", Joint_robot_id);
	    readSDO_byname("Joint_number");
	    readSDO_byname("firmware_version");

        } catch (EscWrpError &e ) {

            DPRINTF("Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what());
            return EC_BOARD_INIT_SDO_FAIL;
        }

        if ( Joint_robot_id > 0 ) {
	    try {
                std::string esc_conf_key = std::string("HpESC_"+std::to_string(Joint_robot_id));
                if ( read_conf(esc_conf_key, root_cfg) != EC_WRP_OK ) {
		    esc_conf_key = std::string("HpESC_X");
		    if ( read_conf(esc_conf_key, root_cfg) != EC_WRP_OK ) {
			DPRINTF("NO config for HpESC_%d in %s\n", Joint_robot_id, __PRETTY_FUNCTION__);
			return EC_BOARD_KEY_NOT_FOUND;
		    }
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
	readSDO_byname("Max_vel");
	readSDO_byname("Max_tor");
	readSDO_byname("Max_cur");
	readSDO_byname("link_pos");

	// set filename with robot_id
        log_filename = std::string("/tmp/HpESC_"+std::to_string(sdo.Joint_robot_id)+"_log.txt");
    
        // Paranoid Direct_ref
        float direct_ref = 0.0;
        writeSDO_byname("Direct_ref", direct_ref);
        readSDO_byname("Direct_ref", direct_ref);
        assert(direct_ref == 0.0);
        
        // we log when receive PDOs
        memset(&log, 0, sizeof(log));
        start_log(true);
        
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
        uint16_t fault;
        uint32_t enable_mask = 0x0;
        uint16_t gain;
        float max_vel = 3.0;

        DPRINTF("Start motor[%d] 0x%02X %.2f %.2f %.2f\n", Joint_robot_id, controller_type, _p, _i, _d);
        
        try {
            set_ctrl_status_X(this, CTRL_POWER_MOD_OFF);
             // set PID gains ... this will NOT set tx_pdo.gainP ....
            writeSDO_byname("PosGainP", _p);
            writeSDO_byname("PosGainI", _i);
            writeSDO_byname("PosGainD", _d);
            // this will SET tx_pdo.gainP
            gain = (uint16_t)_p;
            writeSDO_byname("gainP", gain);
            gain = (uint16_t)_d;
            writeSDO_byname("gainD", gain);
            // pdo gains will be used in OP
            writeSDO_byname("board_enable_mask", enable_mask);
            writeSDO_byname("Max_vel", max_vel);
            
            // set actual position as reference
            readSDO_byname("link_pos", act_position);
            writeSDO_byname("pos_ref", act_position);
            // set direct mode and power on modulator
            set_ctrl_status_X(this, CTRL_SET_DIRECT_MODE);
            set_ctrl_status_X(this, CTRL_POWER_MOD_ON);
            
            readSDO_byname("fault", fault);
            handle_fault();
            
            // set position mode
            set_ctrl_status_X(this, controller_type);
            
            // enable trajectory_gen
            //enable_mask = 0x2;
            //writeSDO_byname("board_enable_mask", enable_mask);
            
        } catch (EscWrpError &e ) {

            DPRINTF("Catch Exception %s ... %s\n", __FUNCTION__, e.what());
            return EC_BOARD_NOK;
        }

        return EC_BOARD_OK;

    }
    
    virtual int start(int controller_type) {
        
        std::vector<float> pid;
        try {
            pid = node_cfg["pid"]["mix_position"].as<std::vector<float>>();
            assert(pid.size() == 3);
        } catch (std::exception &e) {
                DPRINTF("Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what());
                return EC_BOARD_NOK;
            }
        return start(controller_type, pid[0], pid[1], pid[2]);        
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
        tx_pdo.fault_ack = fault.all & 0x7FFF;
    }

    /////////////////////////////////////////////
    // set pdo data
    virtual int set_posRef(float joint_pos) { tx_pdo.pos_ref = hipwr_esc::J2M(joint_pos,_sgn,_offset); }
//     virtual int set_torOffs(float tor_offs) { /*tx_pdo.tor_offs = tor_offs;*/ }
//     virtual int set_posGainP(float p_gain)  { tx_pdo.gainP = p_gain;   }
//     virtual int set_posGainI(float i_gain)  { /*tx_pdo.PosGainI = i_gain;*/   }
//     virtual int set_posGainD(float d_gain)  { tx_pdo.gainD = d_gain;   }

    virtual int move_to(float pos_ref, float step) {
        
        float pos, tx_pos_ref;
        
        try {
            readSDO_byname("link_pos", pos);
            readSDO_byname("pos_ref_fb", tx_pos_ref);
            tx_pos_ref = hipwr_esc::M2J(tx_pos_ref,_sgn,_offset);        
            
            if ( fabs(pos - pos_ref) > step ) {
                if ( pos > pos_ref ) {
                    tx_pos_ref -= step; 
                } else {
                    tx_pos_ref += step;
                }
                    
                writeSDO_byname("pos_ref", tx_pos_ref);
                //DPRINTF("%d move to %f %f %f\n", Joint_robot_id, pos_ref, tx_pos_ref, pos);
                return 0;
               
            } else {
                DPRINTF("%d move to %f %f %f\n", Joint_robot_id, pos_ref, tx_pos_ref, pos);
                return 1;

            }

        } catch (EscWrpError &e ) {
            DPRINTF("Catch Exception %s ... %s\n", __FUNCTION__, e.what());
            return 0;
        }

    }

    /**
     * - set "Enc_offset" = 0
     * - set "Calibration_angle" motor coordinate : Motor 3.14159 --> Joint 0
     * - set "ctrl_status_cmd" =  0x00AB CTRL_SET_ZERO_POSITION
     * - save params to flash
     * - 
     */
    int set_zero_position(float calibration_angle) {
        
        float enc_offset;
        
        // do it in PREOP
        enc_offset = 0.0;
        DPRINTF("%d : set zero pos %f\n", position, calibration_angle);
                
        try {
            writeSDO_byname("Enc_offset", enc_offset);
            writeSDO_byname("Calibration_angle", calibration_angle);
        } catch (EscWrpError &e ) {
                DPRINTF("Catch Exception %s ... %s\n", __FUNCTION__, e.what());
                return EC_BOARD_ZERO_POS_FAIL;
        }
        set_ctrl_status_X(this, CTRL_SET_ZERO_POSITION);
        set_flash_cmd_X(this, FLASH_SAVE);
        
        return EC_BOARD_OK;
    }
    

private:

    int read_conf(std::string conf_key, const YAML::Node & root_cfg) {
	
	if ( ! root_cfg[conf_key] ) {
    	    return EC_BOARD_KEY_NOT_FOUND;
	}
	    
	DPRINTF("Using config %s\n", conf_key.c_str());
	    
	_sgn = root_cfg[conf_key]["sign"].as<int>(); 
	_offset = root_cfg[conf_key]["pos_offset"].as<float>();
	_offset = DEG2RAD(_offset);
	
	return EC_WRP_OK;
    }
    
    Log::log_t log;

    int16_t Joint_robot_id;
    
    YAML::Node node_cfg;
    
    float   _offset;
    int     _sgn;

    stat_t  s_rtt;

    objd_t * SDOs;

};



}
}
}
#endif /* IIT_ECAT_ADVR_ESC_H_ */
