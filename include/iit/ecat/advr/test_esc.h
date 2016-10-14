/*
 *
 *  Created on: Dec, 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_TEST_ESC_H__
#define __IIT_ECAT_ADVR_TEST_ESC_H__

#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/log_esc.h>
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
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_motor_vel(link_vel);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_torque(torque);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_temperature(temperature);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_fault(fault);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_rtt(rtt);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_op_idx_ack(op_idx_ack);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_aux(aux);
            pb_rx_pdo.SerializeToString(pb_str);
        }
    }  __attribute__ ( ( __packed__ ) ); // 32 bytes

}; // 56 bytes

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
    
    // 0x8001 flash param
    float   PosGainP;
    float   PosGainI;
    float   PosGainD;
    float   TorGainP;
    float   TorGainI;
    float   TorGainD;
    float   Pos_I_lim;
    float   Tor_I_lim;
    float   Min_pos;
    float   Max_pos;
    float   Max_tor;
    float   Max_cur;
    int16_t ConfigFlags;
    int16_t ConfigFlags2;
    float   ImpedancePosGainP;
    float   ImpedancePosGainD;
    int     MaxPWM;
    int16_t Joint_number;
    int16_t Joint_robot_id;
    float   Target_velocity;
    // 0x8002 ram param
    char fw_ver[8];
    unsigned int ack_board_faults;
    unsigned short ctrl_status_cmd;
    unsigned short ctrl_status_cmd_ack;
    float direct_ref;
    float abs_pos;
    float m_current;
    unsigned short flash_params_cmd;
    unsigned short flash_params_cmd_ack;
    // 0x8002 aux param
    float volt_ref;
    float current;
    float vout;
    float pos_ref_fb;
    
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
    public PDO_log<TestEscLogTypes> {

public:
    typedef BasicEscWrapper<TestEscPdoTypes,TestEscSdoTypes>    Base;
    typedef PDO_log<TestEscLogTypes>                            Log;

    TestESC ( const ec_slavet& slave_descriptor ) :
        Base ( slave_descriptor ),
        Log ( std::string ( "/tmp/ESC_test_pos"+std::to_string ( position ) +"_log.txt" ),DEFAULT_LOG_SIZE ) {

    }

    virtual ~TestESC ( void ) {
        delete [] SDOs;
        DPRINTF ( "~%s %d\n", typeid ( this ).name(), position );
        print_stat ( s_rtt );
    }

    void print_info ( void ) {
        DPRINTF ( "\tJoint id %d\tJoint robot id %d\n", sdo.Joint_number, sdo.Joint_robot_id );
        DPRINTF ( "\tmin pos %f\tmax pos %f\tmax vel %f\n", sdo.Min_pos, sdo.Max_pos, sdo.Target_velocity );
        DPRINTF ( "\tPosGainP: %f PosGainI: %f PosGainD: %f I lim: %f\n", sdo.PosGainP, sdo.PosGainI, sdo.PosGainD, sdo.Pos_I_lim );
        DPRINTF ( "\tImpPosGainP :%f ImpPosGainD:%f\n", sdo.ImpedancePosGainP, sdo.ImpedancePosGainD );
        DPRINTF ( "\tTorGainP:%f TorGainI:%f Tor_I_lim:%f\n", sdo.TorGainP, sdo.TorGainI, sdo.Tor_I_lim );
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

        if ( _start_log ) {
            Log::log_t log;
            log.ts = get_time_ns() - _start_log_ts ;
            log.rx_pdo = rx_pdo;
            push_back ( log );
        }

        //PDO_aux(getSDObjd("pos_ref_fb")).on_rx(rx_pdo);
        //PDO_aux(getSDObjd("volt_ref")).on_rx(rx_pdo);

    }

    virtual void on_writePDO ( void ) {
        tx_pdo.ts = get_time_ns() / 1000;
        //
        //PDO_aux(getSDObjd("pos_ref_fb")).on_tx(tx_pdo);
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

        } catch ( EscWrpError &e ) {

            DPRINTF ( "Catch Exception %s ... %s\n", __FUNCTION__, e.what() );
            return EC_BOARD_INIT_SDO_FAIL;
        } catch ( std::exception &e ) {

            DPRINTF ( "Exception %s ... %s\n", __FUNCTION__, e.what() );
            return EC_WRP_NOK;
        }

        readSDO_byname ( "fw_ver" );
        readSDO_byname ( "PosGainP");
        readSDO_byname ( "PosGainI");
        readSDO_byname ( "PosGainD");
        
#if 0
        readSDO_byname ( "Min_pos" );
        readSDO_byname ( "Max_pos" );
        readSDO_byname ( "link_pos" );
        readSDO_byname ( "Pos_I_lim");
        readSDO_byname ( "ImpPosGainP");
        readSDO_byname ( "ImpPosGainD");
        readSDO_byname ( "TorGainP");
        readSDO_byname ( "TorGainI");
        readSDO_byname ( "Tor_I_lim");
#endif
        // Should be better to start logging when enter OP ....
        start_log ( true );

        return EC_WRP_OK;

    }


private:

    objd_t * SDOs;

    stat_t  s_rtt;

};



}
}
}

#endif


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
