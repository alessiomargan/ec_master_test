/*
 * mc_lopwr_sph_esc.h
 * 
 *  LoPower Motor Controlleer 
 *  based on TI TM4C123AH6PM - Tiva Microcontroller
 *  High performance 32-Bit ARM Cortex M4F
 *  
 *  http://www.ti.com/product/tm4c123ah6pm
 *  
 *  Created on: Dec 2015
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_MC_LOWPWR_SPHBRK_ESC_H__
#define __IIT_ECAT_ADVR_MC_LOWPWR_SPHBRK_ESC_H__

#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/advr/esc.h>
//#include <iit/ecat/advr/motor_iface.h>
#include <iit/ecat/advr/log_esc.h>
#include <iit/ecat/utils.h>
#include <map>

namespace iit {
namespace ecat {
namespace advr {


struct LoPwrSphBrkEscSdoTypes {
    
    // flash
    int     Block_control;
    int     nonius_offset_low;
    float   PosGainP;
    float   PosGainI;
    float   PosGainD;
    float   TorGainP;
    float   TorGainI;
    float   TorGainD;
    float   Torque_Mult;
    float   Pos_I_lim;
    float   Tor_I_lim;
    float   Min_pos;
    float   Max_pos;
    int     nonius_offset_high;
    float   Max_tor;
    float   Max_cur;
    int     Enc_offset_1;
    int     Enc_offset_2;
    float   Torque_Offset;
    int16_t ConfigFlags;
    int16_t ConfigFlags2;
    int     NumEncoderLines;
    float   ImpedancePosGainP;
    int     nonius_offset2_low;
    float   ImpedancePosGainD;
    int     Num_Abs_counts_rev;
    int     MaxPWM;
    float   Gearbox_ratio;
    int     ulCalPosition;
    int     Cal_Abs_Position;
    int     Cal_Abs2_Position;
    int     nonius_offset2_high;
    int16_t Joint_number;
    int16_t Joint_robot_id;
    float   Target_velocity;
    float   vel_gain_P;
    float   vel_gain_I;
    float   vel_gain_D;
    float   vel_gain_Ilim;
    float   force_gain_P;
    float   force_gain_I;
    float   force_gain_D;
    float   analog_1_mult;
    float   analog_2_mult;
    float   analog_1_offset;
    float   analog_2_offset;
        
    // ram
    char        firmware_version[8];
    uint16_t    enable_pdo_gains;
    uint16_t    set_ctrl_status;
    uint16_t    get_ctrl_status;
    float       direct_ref;
    float       abs_pos;
    float       m_current;
    uint16_t    flash_params_cmd;
    uint16_t    flash_params_cmd_ack;     
};

struct McSphBrkEscPdoTypes {
    // TX  slave_input -- master output
    struct pdo_tx {
        float       pos_ref;
        uint16_t    fault_ack;
        uint16_t    gainP;
        uint16_t    gainD;
        uint16_t    ts;

        void fprint(FILE *fp) {
            fprintf(fp, "%f\t0x%X\t%d\t%d\t%d\n", pos_ref,fault_ack,gainP,gainD,ts);
        }
        int sprint(char *buff, size_t size) {
            return snprintf(buff, size, "%f\t0x%X\t%d\t%d\t%d", pos_ref,fault_ack,gainP,gainD,ts);
        }

    }  __attribute__((__packed__));  // 12 bytes

    // RX  slave_output -- master input
    struct pdo_rx {
        float       link_pos;     // rad
        float       motor_pos;     // rad
        float       link_vel;     // rad/s
        float       motor_vel;     // rad/s
        float       pos_ref_fb;   // rad
        uint16_t    temperature;  // C * 10
        int16_t     torque;       // Nm * 100
        uint16_t    fault;
        uint16_t    rtt;          //
        float       motor_pos_2;     // rad
        float       motor_vel_2;     // rad/s
        int16_t     analog_1;       
        int16_t     analog_2;       
	float       aux;
	
        void fprint(FILE *fp) {
            fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%d\t%d\t0x%X\t%d\t%f\t%f\t%d\t%d\t%f\n",
		    link_pos,motor_pos,link_vel,motor_vel,pos_ref_fb,temperature,torque,fault,rtt,motor_pos_2,motor_vel_2,analog_1,analog_2,aux);
        }
        int sprint(char *buff, size_t size) {
            return snprintf(buff, size, "%f\t%f\t%f\t%f\t%f\t%d\t%d\t0x%X\t%d\t%f\t%f\t%d\t%d\t%f\n",
		    link_pos,motor_pos,link_vel,motor_vel,pos_ref_fb,temperature,torque,fault,rtt,motor_pos_2,motor_vel_2,analog_1,analog_2,aux);
        }
        void to_map(jmap_t & jpdo) {
            JPDO(link_pos);
            JPDO(motor_pos);
            JPDO(link_vel);
            JPDO(motor_vel);
            JPDO(pos_ref_fb); 
            JPDO(temperature);
            JPDO(torque);
            JPDO(fault);
            JPDO(rtt);
	    JPDO(motor_pos_2);
            JPDO(motor_vel_2);
            JPDO(analog_1);
            JPDO(analog_2);
            
        }
        
    }  __attribute__((__packed__)); // 28 bytes
};


struct LoPwrSphBrkLogTypes {

    uint64_t    		ts;           // ns
    McSphBrkEscPdoTypes::pdo_rx	rx_pdo;

    void fprint(FILE *fp) {
            fprintf(fp, "%lu\t", ts);
	    rx_pdo.fprint(fp);
        }
    int sprint(char *buff, size_t size) {
	int l = snprintf(buff, size, "%lu\t", ts); 
	return l + rx_pdo.sprint(buff+l,size-l);
    }
};

/**
 *  
 **/ 

class LpSphBrkESC :
    public BasicEscWrapper<McSphBrkEscPdoTypes,LoPwrSphBrkEscSdoTypes>,
    public PDO_log<LoPwrSphBrkLogTypes>
    //public MotorSph
{
public:
    typedef BasicEscWrapper<McSphBrkEscPdoTypes,LoPwrSphBrkEscSdoTypes> 	Base;
    typedef PDO_log<LoPwrSphBrkLogTypes>                   			Log;

    LpSphBrkESC(const ec_slavet& slave_descriptor) :
        Base(slave_descriptor),
        Log(std::string("/tmp/LpSphBrkESC_pos"+std::to_string(position)+"_log.txt"),DEFAULT_LOG_SIZE)
    {
	//_actual_state = EC_STATE_PRE_OP;
    }
    
    virtual ~LpSphBrkESC(void) { 
        delete [] SDOs;
        DPRINTF("~%s %d\n", typeid(this).name(), position);
        print_stat(s_rtt);
    }

    virtual int16_t get_robot_id() {
        //assert(sdo.Joint_robot_id != -1);
        return sdo.Joint_robot_id;
    }

    void print_info(void) {
        DPRINTF("\tJoint id %d\tJoint robot id %d\n", sdo.Joint_number, sdo.Joint_robot_id);
        DPRINTF("\tmin pos %f\tmax pos %f\tmax vel %f\n", sdo.Min_pos, sdo.Max_pos, sdo.Target_velocity);
        DPRINTF("\tfw_ver %s\n", sdo.firmware_version);
    }

    virtual const objd_t * get_SDOs() { return SDOs; }

    virtual void init_SDOs(void);

    virtual void on_readPDO(void) {

        if ( rx_pdo.rtt ) {
            rx_pdo.rtt =  (uint16_t)(get_time_ns()/1000) - rx_pdo.rtt;
            //DPRINTF(">>>> %d\n", rx_pdo.rtt);  
	    s_rtt(rx_pdo.rtt);
        }

        if ( rx_pdo.fault ) {
            handle_fault();
        } else {
            // clean any previuos fault ack !! 
            tx_pdo.fault_ack = 0;
        }

        // apply transformation from Motor to Joint 
        //rx_pdo.link_pos = lopwr_esc::M2J(rx_pdo.link_pos,_sgn,_offset); 
        //rx_pdo.motor_pos = lopwr_esc::M2J(rx_pdo.motor_pos,_sgn,_offset); 
        //rx_pdo.pos_ref_fb  = lopwr_esc::M2J(rx_pdo.pos_ref_fb,_sgn,_offset);

	if ( _start_log ) {
	    Log::log_t log;
            log.ts = get_time_ns() - _start_log_ts ;
	    log.rx_pdo = rx_pdo;
            push_back(log);
        }
    }

    virtual void on_writePDO(void) {

        tx_pdo.ts = (uint16_t)(get_time_ns()/1000);
    }

    virtual int on_readSDO(const objd_t * sdobj)  {

	return EC_BOARD_OK;
    }

    virtual int on_writeSDO(const objd_t * sdo) {

        // do not allow to write sdo that map txPDO
        //if ( _actual_state == EC_STATE_OPERATIONAL && sdo->index == 0x7000 ) {
        //    return EC_WRP_SDO_WRITE_CB_FAIL;
        //}
        return EC_BOARD_OK;
    }

    ///////////////////////////////////////////////////////
    ///
    /// Motor method implementation
    ///
    ///////////////////////////////////////////////////////
    virtual bool am_i_HpESC() { return false; }
    virtual bool am_i_LpESC() { return true; }
    virtual uint16_t get_ESC_type() { return LO_PWR_SPH_MCBRK; }

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

	std::string esc_conf_key;
        Joint_robot_id = -1;

        try {
            // !! sgn and offset must set before init_sdo_lookup !!
            init_SDOs();
            init_sdo_lookup();
            readSDO_byname("Joint_robot_id", Joint_robot_id);
            readSDO_byname("Joint_number");
	    readSDO_byname("fw_ver");

        } catch (EscWrpError &e ) {

            DPRINTF("Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what());
            return EC_BOARD_INIT_SDO_FAIL;
        }

        if ( Joint_robot_id > 0 ) {
            try {
                esc_conf_key = std::string("LpSphBrkESC_"+std::to_string(Joint_robot_id));
		if ( read_conf(esc_conf_key, root_cfg) != EC_WRP_OK ) {
		    esc_conf_key = std::string("LpSphBrkESC_X");
		    if ( read_conf(esc_conf_key, root_cfg) != EC_WRP_OK ) {
			DPRINTF("NO config for LpSphBrkESC_%d in %s\n", Joint_robot_id, __PRETTY_FUNCTION__);
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
	readSDO_byname("Target_velocity");
	readSDO_byname("link_pos");

        log_filename = std::string("/tmp/LpSphBrkESC_"+std::to_string(sdo.Joint_robot_id)+"_log.txt");
        
	// we log when receive PDOs
        start_log(true);
            
        return EC_WRP_OK;

    }
    virtual int start(int controller_type, float _p, float _i, float _d) {
        
        float act_position, test_ref;
        int32_t fault;

        try {
            set_ctrl_status_X(this, CTRL_POWER_MOD_OFF);
	    // set actual position as reference
            readSDO_byname("link_pos", act_position);
            writeSDO_byname("pos_ref", act_position);
            DPRINTF("start %f\n", act_position);
	    // set direct mode and power on modulator
            set_ctrl_status_X(this, CTRL_SET_DIRECT_MODE);
            set_ctrl_status_X(this, CTRL_POWER_MOD_ON);
	    // set position mode
            set_ctrl_status_X(this, CTRL_SET_POS_MODE);
            
        } catch (EscWrpError &e ) {

            DPRINTF("Catch Exception %s ... %s\n", __FUNCTION__, e.what());
            return EC_BOARD_NOK;
        }

        return EC_BOARD_OK;

    }

    virtual int start(int controller_type) {
        // pid not used
        return start(controller_type, 0, 0, 0);        
    }

    virtual int stop(void) {
        return set_ctrl_status_X(this, CTRL_POWER_MOD_OFF);
    }

    virtual void start_log(bool start) {
        Log::start_log(start);
    }

    virtual void handle_fault(void) {

        fault_t fault;
        fault.all = rx_pdo.fault;
        //fault.bit.
        tx_pdo.fault_ack = fault.all & 0xFFFF;
        //ack_faults_X(this, fault.all);

    }

    /////////////////////////////////////////////
    // set pdo data
    virtual int set_posRef(float joint_pos) { tx_pdo.pos_ref = joint_pos; }
    virtual int set_torOffs(float tor_offs) { /*tx_pdo.tor_offs = tor_offs;*/ }
    virtual int set_posGainP(float p_gain)  { /*tx_pdo.PosGainP = p_gain;*/   }
    virtual int set_posGainI(float i_gain)  { /*tx_pdo.PosGainI = i_gain;*/   }
    virtual int set_posGainD(float d_gain)  { /*tx_pdo.PosGainD = d_gain;*/   }

    virtual int move_to(float pos_ref, float step) {
    
        float pos, tx_pos_ref;
        
        try {
            readSDO_byname("link_pos", pos);
            readSDO_byname("pos_ref_fb", tx_pos_ref);
            //tx_pos_ref = pos_ref;
            if ( fabs(pos - pos_ref) > step ) {
                if ( pos > pos_ref ) {
                    tx_pos_ref -= step*2; 
                } else {
                    tx_pos_ref += step*2;
                }
                    
                writeSDO_byname("pos_ref", tx_pos_ref);
                DPRINTF("%d move to %f %f %f\n", Joint_robot_id, pos_ref, tx_pos_ref, pos);
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

    virtual void set_off_sgn(float offset, int sgn) {}

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
    
    int16_t Joint_robot_id;
    
    float   _offset;
    int     _sgn;

    stat_t  s_rtt;

    objd_t * SDOs;

};



}
}
}
#endif /* IIT_ECAT_ADVR_ESC_H_ */
