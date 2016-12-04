/*
 * mc_centAC_esc.h
 *
 *  Motor Controlleer
 *  based on TI F28M3x - Concerto Microcontroller C28x + ARM M3 32bit MCUs
 *  
 *  http://www.ti.com/product/f28m36p63c2
 *
 *  Created on: Sept 2016
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_MC_CENTAC_ESC_H__
#define __IIT_ECAT_ADVR_MC_CENTAC_ESC_H__

#include <iit/ecat/ec_master_iface.h>
#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/motor_iface.h>
#include <iit/ecat/advr/log_esc.h>
#include <iit/ecat/advr/pipes.h>
#include <iit/ecat/utils.h>
#include <protobuf/ecat_pdo.pb.h>

#include <map>

namespace iit {
namespace ecat {
namespace advr {

namespace centac_esc {
static float J2M ( float p, int s, float o ) {
    return ( M_PI - ( s*o ) + ( s*p ) );
}
static float M2J ( float p, int s, float o ) {
    return ( ( p - M_PI + ( s*o ) ) /s );
}
}



struct CentAcEscSdoTypes {

    // flash
    uint16_t    Hardware_configuration; // 0xABCD
                                        // A = curr_sensor_type:    0 = none    1 = 6A          2 = 10A             3 = 20A         4 = 35A
                                        // B = link enc type:       0 = none    1 = 19-bit      2 = 20-bit
                                        // C = torque_sensor_type:  0 = none    1 = analog DSP  2 = analog_ext_ADC  3 = defl 19-bit 4 = defl 20-bit
                                        // D = number of pole pair

    uint16_t    Motor_gear_ratio;
    float       Motor_electrical_phase_angle;
    float       Torsion_bar_stiffness;
    float       CurrGainP;
    float       CurrGainI;
    float       Max_cur;
    float       Max_tor;
    float       Max_vel;
    float       Joint_Min_pos;
    float       Joint_Max_pos;
    float       Calibration_angle;
    float       Enc_offset;
    int         Serial_number_A;
    int         Joint_robot_id;

    // ram
    char        m3_fw_ver[8];
    char        c28_fw_ver[8];
    uint16_t    ctrl_status_cmd;
    uint16_t    ctrl_status_cmd_ack;
    uint16_t    flash_params_cmd;
    uint16_t    flash_params_cmd_ack;
    float       iq_ref;
    uint16_t    fault;
    float       v_batt;
    float       i_batt;
    float       torque_read;
    float       board_temp;
    float       motor_temp;
    
    // aux pdo
    float       pos_ref_fb;
    float       iq_ref_fb;
    float       iq_out_fb;
    float       torque_no_average;
    float       torque_no_calibrated;
    float       board_temp_fb;
    float       motor_temp_fb;
    float       i_batt_fb;

};

struct PROC_FAULT {
    uint16_t  m3_bits:12;
    uint16_t  c28_bits:4;
};

struct BIT_FAULT {

       uint16_t  m3_rxpdo_pos_ref:1;
       uint16_t  m3_rxpdo_vel_ref:1;
       uint16_t  m3_rxpdo_tor_ref:1;
       uint16_t  m3_flag_1:1;
       uint16_t  m3_fault_hardware:1;
       uint16_t  m3_params_out_of_range:1;
       uint16_t  m3_flag_2:1;
       uint16_t  m3_flag_3:1;
       uint16_t  m3_link_enc_error_reading:1;
       uint16_t  m3_link_enc_hw_error:1;
       uint16_t  m3_defl_enc_error_reading:1;
       uint16_t  m3_defl_enc_hw_error:1;
 

       uint16_t  c28_motor_enc_error_reading:1;
       uint16_t  c28_motor_enc_hw_error:1;
       uint16_t  c28_Max_cur_limited_for_temp:1;
       uint16_t  c28_irq_alive:1;
};

typedef union{
    uint16_t all;
    struct PROC_FAULT proc_fault;
    struct BIT_FAULT bit;
} centAC_fault_t;



struct CentAcLogTypes {

    uint64_t                ts;         // ns
    float                   pos_ref;
    uint8_t                 m3_link_enc_error_reading;
    uint8_t                 m3_link_enc_hw_error;
    uint8_t                 m3_defl_enc_error_reading;
    uint8_t                 m3_defl_enc_hw_error;
    uint8_t                 c28_motor_enc_error_reading;
    uint8_t                 c28_motor_enc_hw_error;
    //
    McEscPdoTypes::pdo_rx   rx_pdo;
    
    void fprint ( FILE *fp ) {
        fprintf ( fp, "%lu\t%f\t%d\t%d\t%d\t%d\t%d\t%d\t", ts, pos_ref,
                  m3_link_enc_error_reading, m3_link_enc_hw_error,
                  m3_defl_enc_error_reading, m3_defl_enc_hw_error,
                  c28_motor_enc_error_reading, c28_motor_enc_hw_error);
        rx_pdo.fprint ( fp );
    }
    int sprint ( char *buff, size_t size ) {
        int l = snprintf ( buff, size, "%lu\t%f\t%d\t%d\t%d\t%d\t%d\t%d\t", ts, pos_ref,
                  m3_link_enc_error_reading, m3_link_enc_hw_error,
                  m3_defl_enc_error_reading, m3_defl_enc_hw_error,
                  c28_motor_enc_error_reading, c28_motor_enc_hw_error);
        return l + rx_pdo.sprint ( buff+l,size-l );
    }
};


/**
*
**/


class CentAcESC :
    public BasicEscWrapper<McEscPdoTypes,CentAcEscSdoTypes>,
    public PDO_log<CentAcLogTypes>,
    public XDDP_pipe,
    public Motor {

public:
    typedef BasicEscWrapper<McEscPdoTypes,CentAcEscSdoTypes> Base;
    typedef PDO_log<CentAcLogTypes>                          Log;

    CentAcESC ( const ec_slavet& slave_descriptor ) :
        Base ( slave_descriptor ),
        Log ( std::string ( "/tmp/CentAcESC"+std::to_string ( position ) +"_log.txt" ),DEFAULT_LOG_SIZE ),
        XDDP_pipe ()
    {
        _start_log = false;
    }

    virtual ~CentAcESC ( void ) {

        delete [] SDOs;
        DPRINTF ( "~%s pos %d\n", typeid ( this ).name(), position );
        print_stat ( s_rtt );
    }

    virtual int16_t get_robot_id() {
        
        return sdo.Joint_robot_id;
    }

    void print_info ( void ) {

        DPRINTF ( "\tJoint serial# %d\tJoint robot id %d\n", sdo.Serial_number_A, sdo.Joint_robot_id );
        DPRINTF ( "\tmin pos %f\tmax pos %f\n", sdo.Joint_Min_pos, sdo.Joint_Max_pos );
        DPRINTF ( "\tfw_ver m3 %s\tc28 %s\n",
                  std::string((const char *)sdo.m3_fw_ver,8).c_str(),
                  std::string((const char *)sdo.c28_fw_ver,8).c_str() );
    }

    virtual const objd_t * get_SDOs() {
        
        return SDOs;
    }

    void init_SDOs ( void );

protected :
    
    virtual bool am_i_HpESC() { return true; }

    virtual bool am_i_LpESC() { return false; }

    virtual void on_readPDO ( void ) {
        
        centAC_fault_t fault;

        if ( rx_pdo.rtt ) {
            rx_pdo.rtt = ( uint16_t ) ( get_time_ns() /1000 - rx_pdo.rtt );
            s_rtt ( rx_pdo.rtt );
        }

        ///////////////////////////////////////////////////
        // - faults
        if ( rx_pdo.fault & 0x7FFF ) {
            handle_fault();
        } else {
            // clean any previuos fault ack !!
            tx_pdo.fault_ack = 0;
        }

        ///////////////////////////////////////////////////
        // - transformation from Motor to Joint
        rx_pdo.link_pos = centac_esc::M2J ( rx_pdo.link_pos,_sgn,_offset );
        rx_pdo.motor_pos = centac_esc::M2J ( rx_pdo.motor_pos,_sgn,_offset );
        
        ///////////////////////////////////////////////////
        // - scaling values
        rx_pdo.link_vel /= 1000;
        rx_pdo.motor_vel /= 1000;
        
        ///////////////////////////////////////////////////
        // - pdo_aux 
        curr_pdo_aux = &pdo_aux_it->second;
        curr_pdo_aux->on_rx(rx_pdo);
        // if pos_ref_fb apply transformation
        if ( curr_pdo_aux->get_idx() == 1 ) {
            rx_pdo.aux  = centac_esc::M2J ( rx_pdo.aux,_sgn,_offset );
        }
        
        ///////////////////////////////////////////////////
        // - logging
        if ( _start_log ) {
            Log::log_t log;
            log.ts      = get_time_ns() - _start_log_ts ;
            log.pos_ref = centac_esc::M2J ( tx_pdo.pos_ref,_sgn,_offset );
            //
            fault.all = rx_pdo.fault;
            log.m3_link_enc_error_reading = fault.bit.m3_link_enc_error_reading; 
            log.m3_link_enc_hw_error = fault.bit.m3_link_enc_hw_error; 
            log.m3_defl_enc_error_reading = fault.bit.m3_defl_enc_error_reading; 
            log.m3_defl_enc_hw_error = fault.bit.m3_defl_enc_hw_error; 
            log.c28_motor_enc_error_reading = fault.bit.c28_motor_enc_error_reading; 
            log.c28_motor_enc_hw_error = fault.bit.c28_motor_enc_hw_error; 
            rx_pdo.fault &= 0x7FFF; 
            log.rx_pdo  = rx_pdo;
            push_back ( log );
        }

        ///////////////////////////////////////////////////
        // - ipc 
        xddp_write( rx_pdo );
        //std::string sz_string;
        //pb_toString( &sz_string , rx_pdo );
        //xddp_write( sz_string.c_str() );
        
    }

    virtual void on_writePDO ( void ) {

        tx_pdo.ts = ( uint16_t ) ( get_time_ns() /1000 );
        
        ///////////////////////////////////////////////////
        // pdo_aux 
        if ( ++pdo_aux_it == pdo_auxes_map.end() ) { pdo_aux_it = pdo_auxes_map.begin(); }
        curr_pdo_aux = &pdo_aux_it->second;
        curr_pdo_aux->on_tx(tx_pdo);
        
        // NOOOOOOOOOOOO
        // NOT HERE !!! use set_posRef to apply transformation from Joint to Motor
        //tx_pdo.pos_ref = hipwr_esc::J2M(tx_pdo.pos_ref,_sgn,_offset);
    }

    virtual int on_readSDO ( const objd_t * sdobj )  {

        if ( ! strcmp ( sdobj->name, "link_pos" ) ) {
            rx_pdo.link_pos = centac_esc::M2J ( rx_pdo.link_pos,_sgn,_offset );
            //DPRINTF("on_getSDO M2J link_pos %f\n", rx_pdo.link_pos);
        } else if ( ! strcmp ( sdobj->name, "motor_pos" ) ) {
            rx_pdo.motor_pos = centac_esc::M2J ( rx_pdo.motor_pos,_sgn,_offset );
        } else if ( ! strcmp ( sdobj->name, "Min_pos" ) ) {
            sdo.Joint_Min_pos = centac_esc::M2J ( sdo.Joint_Min_pos,_sgn,_offset );
        } else if ( ! strcmp ( sdobj->name, "Max_pos" ) ) {
            sdo.Joint_Max_pos = centac_esc::M2J ( sdo.Joint_Max_pos,_sgn,_offset );
        }
        return EC_BOARD_OK;
    }

    virtual int on_writeSDO ( const objd_t * sdo ) {

        // do not allow to write sdo that map txPDO
        //if ( _actual_state == EC_STATE_OPERATIONAL && sdo->index == 0x7000 ) {
        //    return EC_WRP_SDO_WRITE_CB_FAIL;
        //}
        if ( ! strcmp ( sdo->name, "pos_ref" ) ) {
            tx_pdo.pos_ref = centac_esc::J2M ( tx_pdo.pos_ref,_sgn,_offset );
            //DPRINTF("on_setSDO J2M pos_ref %f\n", tx_pdo.pos_ref);
        }
        return EC_BOARD_OK;
    }


public :
    ///////////////////////////////////////////////////////
    ///
    /// Motor method implementation
    ///
    ///////////////////////////////////////////////////////
    virtual uint16_t get_ESC_type() {
        if ( product_code == CENT_AC ) return CENT_AC;
        return NO_TYPE;
    }
    virtual const pdo_rx_t& getRxPDO() const { return Base::getRxPDO(); }
    virtual const pdo_tx_t& getTxPDO() const { return Base::getTxPDO(); }
    virtual void setTxPDO ( const pdo_tx_t & pdo_tx ) { Base::setTxPDO ( pdo_tx ); }

    virtual int init ( const YAML::Node & root_cfg ) {

        Joint_robot_id = -1;

        try {
            // !! sgn and offset must set before init_sdo_lookup !!
            init_SDOs();
            init_sdo_lookup();
            
            pos_ref_fb_aux = PDO_aux(getSDObjd("pos_ref_fb"));
            iq_ref_fb_aux = PDO_aux(getSDObjd("iq_ref_fb"));
            iq_out_fb_aux = PDO_aux(getSDObjd("iq_out_fb"));
            torque_no_average_aux = PDO_aux(getSDObjd("torque_no_average"));
            torque_no_calibrated_aux = PDO_aux(getSDObjd("torque_no_calibrated"));
            motor_temp_fb_aux = PDO_aux(getSDObjd("motor_temp_fb"));
            board_temp_fb_aux = PDO_aux(getSDObjd("board_temp_fb"));
            i_batt_fb_aux = PDO_aux(getSDObjd("i_batt_fb"));
            // fill map, select which aux  
            pdo_auxes_map["pos_ref_fb"] = pos_ref_fb_aux;
            //pdo_auxes_map["iq_ref_fb"] = iq_ref_fb_aux;
            //pdo_auxes_map["iq_out_fb"] = iq_out_fb_aux;
            //pdo_auxes_map["torque_no_average"] = torque_no_average_aux;
            //pdo_auxes_map["torque_no_calibrated"] = torque_no_calibrated_aux;
            //pdo_auxes_map["motor_temp_fb"] = motor_temp_fb_aux;
            //pdo_auxes_map["board_temp_fb"] = board_temp_fb_aux;
            //pdo_auxes_map["i_batt_fb"] = i_batt_fb_aux;
            
            pdo_aux_it = pdo_auxes_map.begin();
            curr_pdo_aux = &pdo_aux_it->second; //&pos_ref_fb_aux;
            
            readSDO_byname ( "Joint_robot_id", Joint_robot_id );
            readSDO_byname ( "Serial_Number_A" );
            readSDO_byname ( "m3_fw_ver" );
            readSDO_byname ( "c28_fw_ver" );

        } catch ( EscWrpError &e ) {

            DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
            return EC_BOARD_INIT_SDO_FAIL;
        }

        if ( Joint_robot_id > 0 ) {
            try {
                std::string esc_conf_key = std::string ( "CentAcESC_"+std::to_string ( Joint_robot_id ) );
                if ( read_conf ( esc_conf_key, root_cfg ) != EC_WRP_OK ) {
                    esc_conf_key = std::string ( "CentAcESC_X" );
                    if ( read_conf ( esc_conf_key, root_cfg ) != EC_WRP_OK ) {
                        DPRINTF ( "NO config for CentAcESC_%d in %s\n", Joint_robot_id, __PRETTY_FUNCTION__ );
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
        readSDO_byname ( "Min_pos" );
        readSDO_byname ( "Max_pos" );
        readSDO_byname ( "Max_vel" );
        readSDO_byname ( "Max_tor" );
        readSDO_byname ( "Max_cur" );
        readSDO_byname ( "link_pos" );

        float max_cur = node_cfg["max_current_A"].as<float>();
        writeSDO_byname ( "Max_cur", max_cur );

        // set filename with robot_id
        log_filename = std::string ( "/tmp/CentAcESC_"+std::to_string ( sdo.Joint_robot_id ) +"_log.txt" );

        // Paranoid Direct_ref
        float direct_ref = 0.0;
        writeSDO_byname ( "Direct_ref", direct_ref );
        readSDO_byname ( "Direct_ref", direct_ref );
        assert ( direct_ref == 0.0 );

        // we log when receive PDOs
        start_log ( true );

        XDDP_pipe::init ("Motor_id_"+std::to_string ( sdo.Joint_robot_id ) );
        
        return EC_BOARD_OK;

    }
    ///////////////////////////////////////////////////////
    /**
     * all done with mailbox
     * !! in OPERATIONAL DO NOT ALLOW to set_SDO that maps TX_PDO !!
     * @return int
     */
    virtual int start ( int controller_type, const std::vector<float> &gains ) {

        std::ostringstream oss;
        float act_position;
        uint16_t fault;
        uint16_t gain;
        //float gain;
        
        DPRINTF ( "Start motor[%d] 0x%02X\n",
                  Joint_robot_id, controller_type);

        try {
            set_ctrl_status_X ( this, CTRL_POWER_MOD_OFF );
            
            // set tx_pdo.gainP
            // pdo gains will be used in OP
            
            if ( controller_type == CTRL_SET_POS_MODE ) {
                // pos_Kp
                //gain = ( uint16_t ) _p;
                gain = (uint16_t)gains[0];
                writeSDO_byname ( "gain_0", gain );
                // pos_Kd
                //gain = ( uint16_t ) _d;
                gain = (uint16_t)gains[2];
                writeSDO_byname ( "gain_1", gain );
                // pos_Ki
                //gain = ( uint16_t ) _d;
                gain = (uint16_t)gains[1];
                writeSDO_byname ( "gain_4", gain );
                
            } else if ( controller_type == CTRL_SET_IMPED_MODE ) {
                // pos_Kp
                gain = (uint16_t)gains[0];
                writeSDO_byname ( "gain_0", gain );
                // pos_Kd
                gain = (uint16_t)gains[1];
                writeSDO_byname ( "gain_1", gain );
                // tor_Kp
                gain = (uint16_t)(gains[2] * 10000);
                writeSDO_byname ( "gain_2", gain );
                // tor_Kd
                gain = (uint16_t)(gains[3] * 10000);
                writeSDO_byname ( "gain_3", gain );
                // tor_Ki
                gain = (uint16_t)(gains[4] * 10000);
                writeSDO_byname ( "gain_4", gain );
                
            }
            
            // set actual position as reference
            //readSDO_byname ( "link_pos", act_position );
            readSDO_byname ( "motor_pos", act_position );
            writeSDO_byname ( "pos_ref", act_position );
            DPRINTF ( "%s\n\tlink_pos %f pos_ref %f\n", __PRETTY_FUNCTION__,
                      act_position,
                      centac_esc::M2J(tx_pdo.pos_ref,_sgn,_offset) );
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

            DPRINTF ( "Catch Exception %s ... %s\n", __FUNCTION__, e.what() );
            return EC_BOARD_NOK;
        }

        return EC_BOARD_OK;

    }

    virtual int start ( int controller_type ) {

        std::vector<float> gains;
        
        if ( controller_type == CTRL_SET_POS_MODE ) {
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

            if ( node_cfg["pid"]["impedance"] ) {
                try {
                    gains = node_cfg["pid"]["impedance"].as<std::vector<float>>();
                    assert ( gains.size() == 5 );
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

        centAC_fault_t fault;
        fault.all = rx_pdo.fault;
        //DPRINTF("[%d]fault 0x%04X\n", Joint_robot_id, fault.all );
        //fault.bit.
        tx_pdo.fault_ack = fault.all & 0x7FFF;
    }

    /////////////////////////////////////////////
    // set pdo data
    virtual int set_posRef ( float joint_pos ) {
        tx_pdo.pos_ref = centac_esc::J2M(joint_pos,_sgn,_offset);
    }
    virtual int set_velRef ( float joint_vel ) {
        tx_pdo.vel_ref = joint_vel;
    }
    virtual int set_torRef ( float joint_tor ) {
        tx_pdo.tor_ref = joint_tor;
    }
    virtual int set_ivRef ( float joint_iv ) {
        sdo.iq_ref = joint_iv;
    // TODO
    }

#if 0
    virtual int set_torOffs ( float tor_offs ) {
        /*tx_pdo.tor_offs = tor_offs;*/
    }
    virtual int set_posGainP ( float p_gain )  {
        tx_pdo.gain_kp_l = p_gain;
    }
    virtual int set_posGainI ( float i_gain )  {
        tx_pdo.gain_ki = i_gain;
    }
    virtual int set_posGainD ( float d_gain )  {
        tx_pdo.gain_kd_l = d_gain;
    }
#endif
    virtual int move_to ( float pos_ref, float step ) {

        float       pos, link_pos, motor_pos, tx_pos_ref;
        uint16_t    fault;

        try {
            readSDO_byname ( "fault", fault );
            handle_fault();
            readSDO_byname ( "link_pos", link_pos );
            readSDO_byname ( "motor_pos", motor_pos );
            readSDO_byname ( "pos_ref", tx_pos_ref );
            tx_pos_ref = centac_esc::M2J ( tx_pos_ref,_sgn,_offset );

            // use motor_position !!!
            pos = motor_pos;
            
            if ( fabs ( pos - pos_ref ) > step ) {
                if ( pos > pos_ref ) {
                    tx_pos_ref -= step;
                } else {
                    tx_pos_ref += step;
                }

                writeSDO_byname ( "pos_ref", tx_pos_ref );
                DPRINTF("%d move to %f %f %f\n", Joint_robot_id, pos_ref, tx_pos_ref, pos);
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

    /**
     * - set "Enc_offset" = 0
     * - set "Calibration_angle" motor coordinate : Motor 3.14159 --> Joint 0
     * - set "ctrl_status_cmd" =  0x00AB CTRL_SET_ZERO_POSITION
     * - save params to flash
     * -
     */
    int set_zero_position ( float calibration_angle ) {

        float enc_offset;

        // do it in PREOP
        enc_offset = 0.0;
        DPRINTF ( "%d : set zero pos %f\n", position, calibration_angle );

        try {
            writeSDO_byname ( "Enc_offset", enc_offset );
            writeSDO_byname ( "Calibration_angle", calibration_angle );
        } catch ( EscWrpError &e ) {
            DPRINTF ( "Catch Exception %s ... %s\n", __FUNCTION__, e.what() );
            return EC_BOARD_ZERO_POS_FAIL;
        }
        set_ctrl_status_X ( this, CTRL_SET_ZERO_POSITION );
        set_flash_cmd_X ( this, FLASH_SAVE );

        return EC_BOARD_OK;
    }

    int run_torque_calibration ( void ) {

        uint16_t cmd, ack;

        cmd = 0x0085;
        writeSDO_byname ( "ctrl_status_cmd", cmd );
        DPRINTF ( "run_torque_calibration ... \n");
        return EC_BOARD_OK;
    }

private:

    int read_conf ( std::string conf_key, const YAML::Node & root_cfg ) {

        if ( ! root_cfg[conf_key] ) {
            return EC_BOARD_KEY_NOT_FOUND;
        }

        DPRINTF ( "\tUsing config %s\n", conf_key.c_str() );
        
        node_cfg = root_cfg[conf_key];
        
        _sgn = node_cfg["sign"].as<int>();
        _offset = node_cfg["pos_offset"].as<float>();
        _offset = DEG2RAD ( _offset );
        set_control_mode(node_cfg["control_mode"].as<std::string>());
//         set_control_mode("pos_3b");s

        return EC_WRP_OK;
    }
    
    void pb_toString( std::string * pb_str , const pdo_rx_t _pdo_rx) {
            static struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            // Header
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_sec(ts.tv_sec);
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_nsec(ts.tv_nsec);
            // Type
            pb_rx_pdo.set_type(iit::advr::Ec_slave_pdo::RX_XT_MOTOR);
            // Motor_xt_tx_pdo
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_link_pos(_pdo_rx.link_pos);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_motor_pos(_pdo_rx.motor_pos);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_link_vel((float)_pdo_rx.link_vel/1000);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_motor_vel((float)_pdo_rx.motor_vel/1000);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_torque(_pdo_rx.torque);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_temperature(_pdo_rx.temperature);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_fault(_pdo_rx.fault);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_rtt(_pdo_rx.rtt);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_op_idx_ack(_pdo_rx.op_idx_ack);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_aux(_pdo_rx.aux);
            pb_rx_pdo.SerializeToString(pb_str);
        }


    int16_t Joint_robot_id;

    YAML::Node node_cfg;

    float   _offset;
    int     _sgn;

    stat_t  s_rtt;

    objd_t * SDOs;
    
    iit::advr::Ec_slave_pdo pb_rx_pdo;
            
    PDO_aux *   curr_pdo_aux;
    PDO_aux     pos_ref_fb_aux;
    PDO_aux     iq_ref_fb_aux;
    PDO_aux     iq_out_fb_aux;
    PDO_aux     torque_no_average_aux;
    PDO_aux     torque_no_calibrated_aux;
    PDO_aux     motor_temp_fb_aux;
    PDO_aux     board_temp_fb_aux;
    PDO_aux     i_batt_fb_aux;
    
    std::map<std::string,PDO_aux>           pdo_auxes_map;
    std::map<std::string,PDO_aux>::iterator pdo_aux_it;
};



}
}
}
#endif /* IIT_ECAT_ADVR_ESC_H_ */
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
