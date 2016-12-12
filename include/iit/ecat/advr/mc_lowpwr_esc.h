/*
 * mc_lopwr_esc.h
 * 
 *  LoPower Motor Controlleer 
 *  based on TI TM4C123AH6PM - Tiva Microcontroller
 *  High performance 32-Bit ARM Cortex M4F
 *  
 *  http://www.ti.com/product/tm4c123ah6pm
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
#include <iit/ecat/advr/pipes.h>
#include <iit/ecat/utils.h>
#include <map>

namespace iit {
namespace ecat {
namespace advr {

namespace lopwr_esc {
static float J2M(float p, int s, float o) { return ((s*o) + (s*p)); }
static float M2J(float p, int s, float o) { return ((p + (s*o))/s); }
//static float J2M ( float p, int s, float o ) { return p; }
//static float M2J ( float p, int s, float o ) { return p; }
}

struct LoPwrEscSdoTypes {
    
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
    // ram

    char        firmware_version[8];
    uint16_t    enable_pdo_gains;
    uint16_t    set_ctrl_status;
    uint16_t    get_ctrl_status;
    float       V_batt_filt_100ms;
    float       T_inv_filt_100ms;
    float       T_mot1_filt_100ms;
    uint16_t    flash_params_cmd;
    uint16_t    flash_params_cmd_ack;
};


struct LoPwrLogTypes {

    uint64_t                ts;     // ns
    float                   pos_ref;
    McEscPdoTypes::pdo_rx   rx_pdo;

    void fprint ( FILE *fp ) {
        fprintf ( fp, "%lu\t%f\t", ts, pos_ref );
        rx_pdo.fprint ( fp );
    }
    int sprint ( char *buff, size_t size ) {
        int l = snprintf ( buff, size, "%lu\t%f\t", ts, pos_ref );
        return l + rx_pdo.sprint ( buff+l,size-l );
    }
};

/**
 *
 **/

class LpESC :
    public BasicEscWrapper<McEscPdoTypes,LoPwrEscSdoTypes>,
    public PDO_log<LoPwrLogTypes>,
    public XDDP_pipe,
    public Motor
{
public:
    typedef BasicEscWrapper<McEscPdoTypes,LoPwrEscSdoTypes> 	Base;
    typedef PDO_log<LoPwrLogTypes>                   		Log;

    LpESC ( const ec_slavet& slave_descriptor ) :
        Base ( slave_descriptor ),
        Log ( std::string ( "/tmp/LpESC_pos"+std::to_string ( position ) +"_log.txt" ),DEFAULT_LOG_SIZE ),
        XDDP_pipe ()
    {
         _start_log = false;
        //_actual_state = EC_STATE_PRE_OP;
    }

    virtual ~LpESC ( void ) {
        delete [] SDOs;
        DPRINTF ( "~%s %d\n", typeid ( this ).name(), position );
        print_stat ( s_rtt );
    }

    virtual int16_t get_robot_id() {
        //assert(sdo.Joint_robot_id != -1);
        return sdo.Joint_robot_id;
    }

    void print_info ( void ) {
        DPRINTF ( "\tJoint id %d\tJoint robot id %d\n", sdo.Joint_number, sdo.Joint_robot_id );
        DPRINTF ( "\tmin pos %f\tmax pos %f\tmax vel %f\n", sdo.Min_pos, sdo.Max_pos, sdo.Target_velocity );
        DPRINTF ( "\tPosGainP: %f PosGainI: %f PosGainD: %f I lim: %f\n", sdo.PosGainP, sdo.PosGainI, sdo.PosGainD, sdo.Pos_I_lim );
        DPRINTF ( "\tImpPosGainP :%f ImpPosGainD:%f\n", sdo.ImpedancePosGainP, sdo.ImpedancePosGainD );
        DPRINTF ( "\tTorGainP:%f TorGainI:%f Tor_I_lim:%f\n", sdo.TorGainP, sdo.TorGainI, sdo.Tor_I_lim );
        DPRINTF ( "\tfw_ver %s\n", std::string((const char *)sdo.firmware_version,8).c_str() );
    }

    virtual const objd_t * get_SDOs() {
        return SDOs;
    }

    virtual void init_SDOs ( void );

protected :
    
    virtual bool am_i_HpESC() { return false; }

    virtual bool am_i_LpESC() { return true; }

    
    virtual void on_readPDO ( void ) {

        if ( rx_pdo.rtt ) {
            rx_pdo.rtt = ( uint16_t ) ( get_time_ns() /1000 ) - rx_pdo.rtt;
            //DPRINTF(">> %s >> %d\n", __PRETTY_FUNCTION__, rx_pdo.rtt);
            s_rtt ( rx_pdo.rtt );
        }

        if ( rx_pdo.fault ) {
            handle_fault();
        } else {
            // clean any previuos fault ack !!
            tx_pdo.fault_ack = 0;
        }

        // apply transformation from Motor to Joint
        rx_pdo.link_pos = lopwr_esc::M2J ( rx_pdo.link_pos,_sgn,_offset );
        rx_pdo.motor_pos = lopwr_esc::M2J ( rx_pdo.motor_pos,_sgn,_offset );
        //rx_pdo.pos_ref_fb  = lopwr_esc::M2J ( rx_pdo.pos_ref_fb,_sgn,_offset );

        if ( _start_log ) {
            Log::log_t log;
            log.ts = get_time_ns() - _start_log_ts ;
            log.pos_ref = lopwr_esc::M2J ( tx_pdo.pos_ref,_sgn,_offset );
            log.rx_pdo = rx_pdo;
            push_back ( log );
        }
        
        xddp_write ( rx_pdo );
    }

    virtual void on_writePDO ( void ) {

        tx_pdo.ts = ( uint16_t ) ( get_time_ns() /1000 );
        // NOOOOOOOOOOOO
        // NOT HERE !!! use set_posRef to apply transformation from Joint to Motor
        //tx_pdo.pos_ref = lopwr_esc::J2M(tx_pdo.pos_ref,_sgn,_offset);

    }

    virtual int on_readSDO ( const objd_t * sdobj )  {

        if ( ! strcmp ( sdobj->name, "link_pos" ) ) {
            rx_pdo.link_pos = lopwr_esc::M2J ( rx_pdo.link_pos,_sgn,_offset );
            //DPRINTF("on_getSDO M2J link_pos %f\n", rx_pdo.position);
        } else if ( ! strcmp ( sdobj->name, "motor_pos" ) ) {
            rx_pdo.motor_pos = lopwr_esc::M2J ( rx_pdo.motor_pos,_sgn,_offset );
        } else if ( ! strcmp ( sdobj->name, "Min_pos" ) ) {
            sdo.Min_pos = lopwr_esc::M2J ( sdo.Min_pos,_sgn,_offset );
        } else if ( ! strcmp ( sdobj->name, "Max_pos" ) ) {
            sdo.Max_pos = lopwr_esc::M2J ( sdo.Max_pos,_sgn,_offset );
        }
        return EC_BOARD_OK;
    }

    virtual int on_writeSDO ( const objd_t * sdo ) {

        // do not allow to write sdo that map txPDO
        //if ( _actual_state == EC_STATE_OPERATIONAL && sdo->index == 0x7000 ) {
        //    return EC_WRP_SDO_WRITE_CB_FAIL;
        //}
        if ( ! strcmp ( sdo->name, "pos_ref" ) ) {
            tx_pdo.pos_ref = lopwr_esc::J2M ( tx_pdo.pos_ref,_sgn,_offset );
            //DPRINTF("on_setSDO J2M pos_ref %f\n", tx_pdo.pos_ref);
        }
        return EC_BOARD_OK;
    }

public:        
    ///////////////////////////////////////////////////////
    ///
    /// Motor method implementation
    ///
    ///////////////////////////////////////////////////////
    virtual uint16_t get_ESC_type() {
        return LO_PWR_DC_MC;
    }

    virtual const pdo_rx_t& getRxPDO() const {
        return Base::getRxPDO();
    }
    virtual const pdo_tx_t& getTxPDO() const {
        return Base::getTxPDO();
    }
    virtual void setTxPDO ( const pdo_tx_t & pdo_tx ) {
        Base::setTxPDO ( pdo_tx );
    }

    virtual int init ( const YAML::Node & root_cfg ) {

        std::string esc_conf_key;
        Joint_robot_id = -1;

        try {
            // !! sgn and offset must set before init_sdo_lookup !!
            init_SDOs();
            init_sdo_lookup();
            readSDO_byname ( "Joint_robot_id", Joint_robot_id );
            readSDO_byname ( "Joint_number" );
            readSDO_byname ( "fw_ver" );
            
        } catch ( EscWrpError &e ) {

            DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
            return EC_BOARD_INIT_SDO_FAIL;
        }

        if ( Joint_robot_id > 0 ) {
            try {
                esc_conf_key = std::string ( "LpESC_"+std::to_string ( Joint_robot_id ) );
                if ( read_conf ( esc_conf_key, root_cfg ) != EC_WRP_OK ) {
                    esc_conf_key = std::string ( "LpESC_X" );
                    if ( read_conf ( esc_conf_key, root_cfg ) != EC_WRP_OK ) {
                        DPRINTF ( "NO config for LpESC_%d in %s\n", Joint_robot_id, __PRETTY_FUNCTION__ );
                        return EC_BOARD_KEY_NOT_FOUND;
                    }
                }
            } catch ( YAML::KeyNotFound &e ) {
                DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
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

        // redo read SDOs so we can apply _sgn and _offset to transform Min_pos Max_pos to Joint Coordinate
        readSDO_byname ( "Min_pos" );
        readSDO_byname ( "Max_pos" );
        readSDO_byname ( "Target_velocity" );
        readSDO_byname ( "link_pos" );
        readSDO_byname ( "pos_gain_P");
        readSDO_byname ( "pos_gain_I");
        readSDO_byname ( "pos_gain_D");
        readSDO_byname ( "pos_integral_limit");
        readSDO_byname ( "ImpedancePosGainP");
        readSDO_byname ( "ImpedancePosGainD");
        readSDO_byname ( "tor_gain_P");
        readSDO_byname ( "tor_gain_I");
        readSDO_byname ( "tor_integral_limit");

        log_filename = std::string ( "/tmp/LpESC_"+std::to_string ( sdo.Joint_robot_id ) +"_log.txt" );

        // we log when receive PDOs
        start_log ( true );

        XDDP_pipe::init ("Motor_id_"+std::to_string ( get_robot_id() ));
        
        return EC_WRP_OK;

    }
    
    //virtual int start ( int controller_type, float _p, float _i, float _d ) {
    virtual int start ( int controller_type, const std::vector<float> &gains ) {

        std::ostringstream oss;
        float act_position;
        uint16_t fault;
        //uint16_t gain;
        float gain;
        float test_ref;

        try {
            set_ctrl_status_X ( this, CTRL_POWER_MOD_OFF );
            
            if ( controller_type == CTRL_SET_POS_MODE ) {
                
                writeSDO_byname( "pos_gain_P", gains[0] );
                writeSDO_byname( "pos_gain_I", gains[1] );
                writeSDO_byname( "pos_gain_D", gains[2] );
                DPRINTF ( "\tPosGain %f %f %f\n", gains[0], gains[1], gains[2] );
            
            } else if ( controller_type == CTRL_SET_IMPED_MODE ) {

                // Impedance gains : position PD torque PI 
                // PosGainP
                writeSDO_byname( "gain_0", (uint16_t)(gains[0]/100.0));
                // TorGainP
                writeSDO_byname( "gain_1", (uint16_t)(sdo.TorGainP*1000));
                // PosGainD
                writeSDO_byname( "gain_2", (uint16_t)gains[2]);
                // TorGainD
                //writeSDO_byname( "gain_3", (uint16_t)(sdo.TorGainP));
                // TorGainIs
                writeSDO_byname( "gain_4", (uint16_t)(sdo.TorGainI*1000));
                
                oss << tx_pdo;
                DPRINTF ( "\ttx_pdo %s\n", oss.str().c_str() );
            
            }
            else if ( controller_type == CTRL_SET_VOLT_MODE ) {
            
                int16_t configFlags;
                readSDO_byname("ConfigFlags", configFlags);
                // #define CONFIGFLAG_USE_VOLTAGE_MODE_LIMITS_BIT    8
                configFlags |= (0x1 << 8); 
                writeSDO_byname("ConfigFlags", configFlags);
            }
            
            // set actual position as reference
            readSDO_byname ( "link_pos", act_position );
            writeSDO_byname ( "pos_ref", act_position );
            DPRINTF ( "%s\n\tlink_pos %f pos_ref %f\n", __PRETTY_FUNCTION__,
                      act_position,
                      lopwr_esc::M2J(tx_pdo.pos_ref,_sgn,_offset) );
            oss << tx_pdo;
            DPRINTF ( "\ttx_pdo %s\n", oss.str().c_str() );
            // set direct mode and power on modulator
            set_ctrl_status_X ( this, CTRL_SET_DIRECT_MODE );
            set_ctrl_status_X ( this, CTRL_POWER_MOD_ON );

            readSDO_byname ( "fault", fault );
            handle_fault();

            // set controller mode
            set_ctrl_status_X ( this, controller_type );


        } catch ( EscWrpError &e ) {

            DPRINTF ( "Catch Exception %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
            return EC_BOARD_NOK;
        }

        return EC_BOARD_OK;

    }

    virtual int start ( int controller_type ) {
        
        std::vector<float> gains = {0.0, 0.0, 0.0};
        
        if ( controller_type == CTRL_SET_POS_MODE ) {
            // use default value read in init()
            gains[0] = sdo.PosGainP;
            gains[1] = sdo.PosGainI;
            gains[2] = sdo.PosGainD;
            
            if ( node_cfg["pid"]["position"] ) {
                try {
                    gains = node_cfg["pid"]["position"].as<std::vector<float>>();
                    assert ( gains.size() == 3 );
                } catch ( std::exception &e ) {
                    DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
                }
            }
        } 
        if ( controller_type == CTRL_SET_IMPED_MODE ) {
            // Impedance gains : [ Pos_P, Pos_D torque PI 
            // use default value read in init()
            gains[0] = sdo.ImpedancePosGainP;
            gains[1] = 0,0; // not used
            gains[2] = sdo.ImpedancePosGainD;

            if ( node_cfg["pid"]["impedance"] ) {
                try {
                    gains = node_cfg["pid"]["impedance"].as<std::vector<float>>();
                    assert ( gains.size() == 3 );
                    DPRINTF ( "using yaml values\n");  
                } catch ( std::exception &e ) {
                    DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
                }
            }
        } 

        //return start ( controller_type, gains[0], gains[1], gains[2] );
        return start ( controller_type, gains );
    }

    virtual int stop ( void ) {
        return set_ctrl_status_X ( this, CTRL_POWER_MOD_OFF );
    }

    virtual void start_log ( bool start ) {
        Log::start_log ( start );
    }

    virtual void handle_fault ( void ) {

        fault_t fault;
        fault.all = rx_pdo.fault;
        //fault.bit.
        tx_pdo.fault_ack = fault.all & 0xFFFF;
        //ack_faults_X(this, fault.all);

    }

    /////////////////////////////////////////////
    // set pdo data
    virtual int set_posRef ( float joint_pos ) {
        tx_pdo.pos_ref = lopwr_esc::J2M(joint_pos,_sgn,-_offset);
    }
    virtual int set_velRef ( float joint_vel ) {
        tx_pdo.vel_ref = joint_vel;
    }
    virtual int set_torRef ( float joint_tor ) {
        tx_pdo.tor_ref = joint_tor;
    }
#if 0
    virtual int set_torOffs ( float tor_offs ) {
        /*tx_pdo.tor_offs = tor_offs;*/
    }
    virtual int set_posGainP ( float p_gain )  {
        tx_pdo.gain_0 = p_gain;
    }
    virtual int set_posGainI ( float i_gain )  {
        tx_pdo.gain_2 = i_gain;
    }
    virtual int set_posGainD ( float d_gain )  {
        tx_pdo.gain_1 = d_gain;
    }
#endif

    virtual int move_to(float pos_ref, float step) {
    
        float pos, tx_pos_ref;

        try {
            readSDO_byname ( "link_pos", pos );
            readSDO_byname ( "pos_ref", tx_pos_ref );
            //tx_pos_ref = pos_ref;
            if ( fabs ( pos - pos_ref ) > step ) {
                if ( pos > pos_ref ) {
                    tx_pos_ref -= step*2;
                } else {
                    tx_pos_ref += step*2;
                }

                writeSDO_byname ( "pos_ref", tx_pos_ref );
                DPRINTF ( "%d move to %f %f %f\n", Joint_robot_id, pos_ref, tx_pos_ref, pos );
                return 0;
            } else {
                DPRINTF ( "%d move to %f %f %f\n", Joint_robot_id, pos_ref, tx_pos_ref, pos );
                return 1;
            }

        } catch ( EscWrpError &e ) {
            DPRINTF ( "Catch Exception %s ... %s\n", __FUNCTION__, e.what() );
            return 0;
        }

    }

    virtual void set_off_sgn(float offset, int sgn) {}

private:

    int read_conf ( std::string conf_key, const YAML::Node & root_cfg ) {

        if ( ! root_cfg[conf_key] ) {
            return EC_BOARD_KEY_NOT_FOUND;
        }

        DPRINTF ( "Using config %s\n", conf_key.c_str() );
        
        node_cfg = root_cfg[conf_key];
        
        _sgn = node_cfg["sign"].as<int>();
        _offset = node_cfg["pos_offset"].as<float>();
        _offset = DEG2RAD ( _offset );
        set_control_mode(root_cfg[conf_key]["control_mode"].as<std::string>());

        return EC_WRP_OK;
    }

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
