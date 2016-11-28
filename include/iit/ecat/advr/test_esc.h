/*
 *
 *  Created on: Dec, 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_TEST_ESC_H__
#define __IIT_ECAT_ADVR_TEST_ESC_H__

#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/log_esc.h>
#include <iit/ecat/advr/pipes.h>
#include <iit/ecat/utils.h>
#include <protobuf/ecat_pdo.pb.h>


#include <map>
#include <string>


namespace iit {
namespace ecat {
namespace advr {


struct TestEscPdoTypes {
    
    // TX  slave_input -- master output
    struct pdo_tx {
        float       pos_ref;    //link
        float       vel_ref;    //link
        float       tor_ref;    //link
        float       gain_0;     //kp_m  ImpPosP 
        float       gain_1;     //kp_l  ImpTorP
        float       gain_2;     //kd_m  ImpPosD
        float       gain_3;     //kd_l  ImpTorD
        float       gain_4;     //ki    ImpTorI
        uint16_t    fault_ack;
        uint16_t    ts;
        uint16_t    op_idx_aux;  // op [get/set] , idx
        float       aux;         // set value

        std::ostream& dump ( std::ostream& os, const std::string delim ) const {
            os << pos_ref << delim;
            os << vel_ref << delim;
            os << tor_ref << delim;
            os << gain_0 << delim;
            os << gain_1 << delim;
            os << gain_2 << delim;
            os << gain_3 << delim;
            os << gain_4 << delim;
            os << fault_ack << delim;
            os << ts << delim;
            //os << std::endl;
            return os;
        }
        void fprint ( FILE *fp ) {
            std::ostringstream oss;
            dump(oss,"\t");
            fprintf ( fp, "%s", oss.str().c_str() );
        }
        int sprint ( char *buff, size_t size ) {
            std::ostringstream oss;
            dump(oss,"\t");
            return snprintf ( buff, size, "%s", oss.str().c_str() );
        }

    }  __attribute__ ( ( __packed__ ) ); // 42 bytes

    // RX  slave_output -- master input
    struct pdo_rx {
        float       link_pos;           // rad
        float       motor_pos;          // rad
        float       link_vel;           // rad TBD on the firmware 
        float       motor_vel;          // rad/s
        float       torque;             // Nm
        uint16_t    temperature;        // C
        uint16_t    fault;
        uint16_t    rtt;                // us
        uint16_t    op_idx_ack;         // op [ack/nack] , idx
        float       aux;                // get value or nack erro code

        std::ostream& dump ( std::ostream& os, const std::string delim ) const {
            os << link_pos << delim;
            os << motor_pos << delim;
            os << link_vel << delim;
            os << motor_vel << delim;
            os << torque << delim;
            os << temperature << delim;
            os << fault << delim;
            os << rtt << delim;
            os << op_idx_ack << delim;
            os << aux << delim;
            //os << std::endl;
            return os;
        }
        void fprint ( FILE *fp ) {
            std::ostringstream oss;
            dump(oss,"\t");
            fprintf ( fp, "%s", oss.str().c_str() );
        }
        int sprint ( char *buff, size_t size ) {
            std::ostringstream oss;
            dump(oss,"\t");
            return snprintf ( buff, size, "%s", oss.str().c_str() );
        }
        void to_map ( jmap_t & jpdo ) {
            JPDO ( link_pos );
            JPDO ( motor_pos );
            JPDO ( link_vel );
            JPDO ( motor_vel );
            JPDO ( torque );
            JPDO ( temperature );
            JPDO ( fault );
            JPDO ( rtt );
            JPDO ( op_idx_ack );
            JPDO ( aux );
        }
        void pb_toString( std::string * pb_str ) {
            static iit::advr::Ec_slave_pdo pb_rx_pdo;
            static struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            // Header
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_sec(ts.tv_sec);
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_nsec(ts.tv_nsec);
            // Type
            pb_rx_pdo.set_type(iit::advr::Ec_slave_pdo::RX_XT_MOTOR);
            // Motor_xt_tx_pdo
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_link_pos(link_pos);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_motor_pos(motor_pos);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_link_vel(link_vel);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_motor_vel(motor_vel);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_torque(torque);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_temperature(temperature);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_fault(fault);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_rtt(rtt);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_op_idx_ack(op_idx_ack);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_aux(aux);
            pb_rx_pdo.SerializeToString(pb_str);
        }
    }  __attribute__ ( ( __packed__ ) ); // 32 bytes

}; // 74 bytes

/*
template <typename T>
inline std::ostream& operator<< (std::ostream& os, const T& obj ) {
    return obj.dump(os,"\t");
}
*/

inline std::ostream& operator<< (std::ostream& os, const TestEscPdoTypes::pdo_tx& tx_pdo ) {
    return tx_pdo.dump(os,"\t");
}

inline std::ostream& operator<< (std::ostream& os, const TestEscPdoTypes::pdo_rx& rx_pdo ) {
    return rx_pdo.dump(os,"\t");
}


struct TestEscSdoTypes {
    
    // 0x8000 flash param
    uint32_t    board_id;
    // 0x8001 ram param
    char        fw_ver[8];
    uint16_t    ctrl_status_cmd;
    uint16_t    ctrl_status_cmd_ack;
    uint16_t    flash_params_cmd;
    uint16_t    flash_params_cmd_ack;
    uint16_t    ack_board_faults;
    // 0x8002 aux param
    float       volt_ref;
    float       current;
    float       vout;
    float       pos_ref_fb;
    float       pwm_duty;
    
};

struct TestEscLogTypes {

    uint64_t                ts;           // ns
    TestEscPdoTypes::pdo_rx rx_pdo;

    void fprint ( FILE *fp ) {
        fprintf ( fp, "%lu\t", ts );
        rx_pdo.fprint ( fp );
    }
    int sprint ( char *buff, size_t size ) {
        int l = snprintf ( buff, size, "%lu\t", ts );
        return l + rx_pdo.sprint ( buff+l,size-l );
    }
};


class TestESC :
    public BasicEscWrapper<TestEscPdoTypes,TestEscSdoTypes>,
    public PDO_log<TestEscLogTypes>,
    public XDDP_pipe
{
public:
    typedef BasicEscWrapper<TestEscPdoTypes,TestEscSdoTypes>    Base;
    typedef PDO_log<TestEscLogTypes>                            Log;

    TestESC ( const ec_slavet& slave_descriptor ) :
        Base ( slave_descriptor ),
        Log ( std::string ( "/tmp/ESC_test_pos"+std::to_string ( position ) +"_log.txt" ),DEFAULT_LOG_SIZE ),
        XDDP_pipe ()
    {

    }

    virtual ~TestESC ( void ) {
        delete [] SDOs;
        DPRINTF ( "~%s %d\n", typeid ( this ).name(), position );
        print_stat ( s_rtt );
    }

    void print_info ( void ) {
//         DPRINTF ( "\tJoint id %d\tJoint robot id %d\n", sdo.Joint_number, sdo.Joint_robot_id );
//         DPRINTF ( "\tmin pos %f\tmax pos %f\tmax vel %f\n", sdo.Min_pos, sdo.Max_pos, sdo.Target_velocity );
//         DPRINTF ( "\tPosGainP: %f PosGainI: %f PosGainD: %f I lim: %f\n", sdo.PosGainP, sdo.PosGainI, sdo.PosGainD, sdo.Pos_I_lim );
//         DPRINTF ( "\tImpPosGainP :%f ImpPosGainD:%f\n", sdo.ImpedancePosGainP, sdo.ImpedancePosGainD );
//         DPRINTF ( "\tTorGainP:%f TorGainI:%f Tor_I_lim:%f\n", sdo.TorGainP, sdo.TorGainI, sdo.Tor_I_lim );
        DPRINTF ( "\tfw_ver %s\n", std::string((const char *)sdo.fw_ver,8).c_str() );
    }

    virtual void on_readPDO ( void ) {

        if ( rx_pdo.rtt ) {
            rx_pdo.rtt = ( uint16_t ) ( get_time_ns() / 1000 ) - rx_pdo.rtt;
            //DPRINTF(">> %s >> %d\n", __PRETTY_FUNCTION__, rx_pdo.rtt);
            s_rtt ( rx_pdo.rtt );
        }
        if ( rx_pdo.fault ) {
            ;//handle_fault();
        } else {
            // clean any previuos fault ack !!
            tx_pdo.fault_ack = 0;
        }

        ///////////////////////////////////////////////////
        // - pdo_aux 
        curr_pdo_aux = &pdo_aux_it->second;
        curr_pdo_aux->on_rx(rx_pdo);
        
        ///////////////////////////////////////////////////
        // - logging 
        if ( _start_log ) {
            Log::log_t log;
            log.ts = get_time_ns() - _start_log_ts ;
            log.rx_pdo = rx_pdo;
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
        tx_pdo.ts = get_time_ns() / 1000;

        ///////////////////////////////////////////////////
        // pdo_aux 
        if ( ++pdo_aux_it == pdo_auxes_map.end() ) { pdo_aux_it = pdo_auxes_map.begin(); }
        curr_pdo_aux = &pdo_aux_it->second;
        curr_pdo_aux->on_tx(tx_pdo);
    }

    virtual const objd_t * get_SDOs() {
        return SDOs;
    }
    virtual void init_SDOs ( void );
    
    virtual uint16_t get_ESC_type() {
        return EC_TEST;
    }

    virtual int init ( const YAML::Node & root_cfg ) {

        try {

            init_SDOs();
            init_sdo_lookup();
            
            pos_ref_fb_aux = PDO_aux(getSDObjd("pos_ref_fb"));
            volt_ref_fb_aux = PDO_aux(getSDObjd("volt_ref_fb"));
            vout_fb_aux = PDO_aux(getSDObjd("vout_fb"));
            current_fb_aux = PDO_aux(getSDObjd("current_fb"));
            pwm_duty_aux = PDO_aux(getSDObjd("pwm_duty"));
            // fill map, select which aux  
            pdo_auxes_map["pos_ref_fb"] = pos_ref_fb_aux;
            pdo_auxes_map["current_fb"] = current_fb_aux;
            pdo_auxes_map["volt_ref_fb"] = volt_ref_fb_aux;
            pdo_auxes_map["vout_fb"] = vout_fb_aux;
            pdo_auxes_map["pwm_duty"] = pwm_duty_aux;
            
            pdo_aux_it = pdo_auxes_map.begin();
            curr_pdo_aux = &pdo_aux_it->second; //&pos_ref_fb_aux;

        } catch ( EscWrpError &e ) {

            DPRINTF ( "Catch Exception %s ... %s\n", __FUNCTION__, e.what() );
            return EC_BOARD_INIT_SDO_FAIL;
        } catch ( std::exception &e ) {

            DPRINTF ( "Exception %s ... %s\n", __FUNCTION__, e.what() );
            return EC_WRP_NOK;
        }

        readSDO_byname ( "fw_ver" );
        
        // Should be better to start logging when enter OP ....
        start_log ( true );

        XDDP_pipe::init ( "Test_pos_"+std::to_string ( position ) );
        
        return EC_WRP_OK;

    }


private:
    
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


    objd_t * SDOs;

    stat_t  s_rtt;
    
    iit::advr::Ec_slave_pdo pb_rx_pdo;
            
    PDO_aux *   curr_pdo_aux;
    PDO_aux     pos_ref_fb_aux;
    PDO_aux     volt_ref_fb_aux;
    PDO_aux     vout_fb_aux;
    PDO_aux     current_fb_aux;
    PDO_aux     pwm_duty_aux;
    
    std::map<std::string,PDO_aux>           pdo_auxes_map;
    std::map<std::string,PDO_aux>::iterator pdo_aux_it;


};



}
}
}

#endif