/*
 *
 *  Created on: Dec, 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_TEST_ESC_H__
#define __IIT_ECAT_ADVR_TEST_ESC_H__

#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/log_esc.h>
#include <protobuf/dummy.pb.h>

#include <map>
#include <string>


namespace iit {
namespace ecat {
namespace advr {


struct TestEscPdoTypes {
    // TX  slave_input -- master output
    struct pdo_tx {
        float       pos_ref;  //link
        int16_t     vel_ref;  //link
        int16_t     tor_ref;  //link
        uint16_t    gain_kp_m;
        uint16_t    gain_kp_l;
        uint16_t    gain_kd_m;
        uint16_t    gain_kd_l;
        uint16_t    gain_ki;
        uint16_t    fault_ack;
        uint16_t    ts;
        uint16_t    op_idx_aux;  // op [get/set] , idx
        float       aux;         // set value

        void fprint ( FILE *fp ) {
            fprintf ( fp, "%f\t%d\t%d\t%d\t%d\n", pos_ref,vel_ref,tor_ref,fault_ack,ts );
        }
        int sprint ( char *buff, size_t size ) {
            return snprintf ( buff, size, "%f\t%d\t%d\t%d\t%d\n", pos_ref,vel_ref,tor_ref,fault_ack,ts );
        }

    }  __attribute__ ( ( __packed__ ) ); // 12 bytes

    // RX  slave_output -- master input
    struct pdo_rx {
        float        link_pos;           // rad
        float        motor_pos;          // rad
        float        link_vel;           // rad TBD on the firmware 
        int16_t      motor_vel;          // rad/s
        int16_t      torque;             // Nm
        uint16_t     temperature;        // C
        uint16_t     fault;
        uint16_t     rtt;                // us
        uint16_t     op_idx_ack;         // op [ack/nack] , idx
        float        aux;                // get value or nack erro code


        void fprint ( FILE *fp ) {
            fprintf ( fp, "%f\t%f\t%f\t%d\t%d\t%d\t%d\t%d\n", link_pos,motor_pos,link_vel,motor_vel,torque,temperature,fault,rtt );
        }
        int sprint ( char *buff, size_t size ) {
            return snprintf ( buff, size, "%f\t%f\t%f\t%d\t%d\t%d\t%d\t%d\n", link_pos,motor_pos,link_vel,motor_vel,torque,temperature,fault,rtt );
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
        }
        void pb_toString( std::string * pb_str ) {
            iit::advr::Ec_slave_pdo pb_rx_pdo;
            // Type
            pb_rx_pdo.set_type(iit::advr::Ec_slave_pdo::RX_MOTOR);
            // Header
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_sec(0);
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_nsec(999);
            // Motor_rx_pdo
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_link_pos(link_pos);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_motor_pos(motor_pos);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_link_vel(link_vel);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_motor_vel(link_vel);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_torque(torque);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_temperature(temperature);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_fault(fault);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_rtt(rtt);
            pb_rx_pdo.SerializeToString(pb_str);
        }
    }  __attribute__ ( ( __packed__ ) ); // 20 bytes
};

inline std::ostream& operator<< (std::ostream& os, const TestEscPdoTypes::pdo_rx& rx_pdo ) {
    os << rx_pdo.link_pos << "\t";
    os << rx_pdo.motor_pos << "\t";
    os << rx_pdo.link_vel << "\t";
    os << rx_pdo.motor_vel << "\t";
    os << rx_pdo.torque << "\t";
    os << rx_pdo.temperature << "\t";
    os << rx_pdo.fault << "\t";
    os << rx_pdo.rtt << "\t";
    os << std::endl;
    return os;
}

struct TestEscSdoTypes {
    char fw_ver[8];
    unsigned int ack_board_faults;
    unsigned short ctrl_status_cmd;
    unsigned short ctrl_status_cmd_ack;
    float direct_ref;
    float abs_pos;
    float m_current;
    unsigned short flash_params_cmd;
    unsigned short flash_params_cmd_ack;
};

struct TestEscLogTypes {

    uint64_t    		ts;           // ns
    TestEscPdoTypes::pdo_rx	rx_pdo;

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
    typedef PDO_log<TestEscLogTypes>                    	Log;

    TestESC ( const ec_slavet& slave_descriptor ) :
        Base ( slave_descriptor ),
        Log ( std::string ( "/tmp/ESC_test_pos"+std::to_string ( position ) +"_log.txt" ),DEFAULT_LOG_SIZE ) {

    }

    virtual ~TestESC ( void ) {
        delete [] SDOs;
        DPRINTF ( "~%s %d\n", typeid ( this ).name(), position );
        print_stat ( s_rtt );
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

    }

    virtual void on_writePDO ( void ) {
        tx_pdo.ts = get_time_ns() / 1000;
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
