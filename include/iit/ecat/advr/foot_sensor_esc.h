/*
 *  ft6_esc.h
 *      
 *  Foot Sensor Board (grid 16 x 8)
 * 
 *  Created on: Apr 2016
 *      Author: Luca Muratore
 *      E-mail: luca.muratore@iit.it 
 */

#ifndef __IIT_ECAT_ADVR_FOOT_SENSOR_ESC_H__
#define __IIT_ECAT_ADVR_FOOT_SENSOR_ESC_H__

#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/log_esc.h>
#include <iit/ecat/utils.h>
#include <protobuf/ecat_pdo.pb.h>

#include <map>
#include <iostream>

#define SENSOR_NUMBER 128

namespace iit {
namespace ecat {
namespace advr {


struct FootSensorEscPdoTypes {
    // TX  slave_input -- master output
    struct pdo_tx {
        uint16_t    ts;
    }  __attribute__ ( ( __packed__ ) );

    // RX  slave_output -- master input
    struct pdo_rx {
        uint8_t    forceXY[SENSOR_NUMBER];
        uint16_t   fault;
        uint16_t   rtt;                
        
        std::ostream& dump ( std::ostream& os, const std::string delim ) const {
            for (int i=0; i<SENSOR_NUMBER; i++) {
                os << (unsigned)forceXY[i] << delim;
            }
            os << std::hex << fault << std::dec << delim;
            os << rtt << delim;
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
            for(int i = 0; i < SENSOR_NUMBER; i++) {
                jpdo["forceXY_"+std::to_string(i)] = std::to_string( forceXY[i] );
            }
            JPDO ( fault );
            JPDO ( rtt );
        }
        void pb_toString( std::string * pb_str ) {
            // !!! NO static declaration
            iit::advr::Ec_slave_pdo pb_rx_pdo;
            static struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            // Header
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_sec(ts.tv_sec);
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_nsec(ts.tv_nsec);
            // Type
            pb_rx_pdo.set_type(iit::advr::Ec_slave_pdo::RX_FOOT_SENS);
            // footWalkman_rx_pdo
            pb_rx_pdo.mutable_footwalkman_rx_pdo()->set_fault(fault);
            pb_rx_pdo.mutable_footwalkman_rx_pdo()->set_rtt(rtt);
            //for (int i=0; i<SENSOR_NUMBER; i++) { pb_rx_pdo.mutable_footwalkman_rx_pdo()->set_forcexy(i,(unsigned)forceXY[i]); }
            //pb_rx_pdo.mutable_footwalkman_rx_pdo()->clear_forcexy();
            for (int i=0; i<SENSOR_NUMBER; i++) { pb_rx_pdo.mutable_footwalkman_rx_pdo()->add_forcexy((unsigned)forceXY[i]); }
            //std::cout << "FootXY " << pb_rx_pdo.mutable_footwalkman_rx_pdo()->forcexy_size() << std::endl;
            pb_rx_pdo.SerializeToString(pb_str);
        }

    }  __attribute__ ( ( __packed__ ) );
};


struct FootSensorEscSdoTypes {

    // flash

    unsigned long Block_control;
    long NumAvSamples;

    unsigned long calibration_offset0;
    unsigned long calibration_offset1;
    unsigned long calibration_offset2;
    unsigned long calibration_offset3;
    unsigned long calibration_offset4;
    unsigned long calibration_offset5;

    float matrix_r1_c1;
    float matrix_r1_c2;
    float matrix_r1_c3;
    float matrix_r1_c4;
    float matrix_r1_c5;
    float matrix_r1_c6;

    int16_t sensor_number;
    int16_t sensor_robot_id;

    // ram

    char        firmware_version[8];
    uint16_t    ack_board_fault;
    float       matrix_rn_c1;
    float       matrix_rn_c2;
    float       matrix_rn_c3;
    float       matrix_rn_c4;
    float       matrix_rn_c5;
    float       matrix_rn_c6;
    uint16_t    flash_params_cmd;
    uint16_t    flash_params_cmd_ack;
};

/**
*
**/

class FootSensorESC :
    public BasicEscWrapper<FootSensorEscPdoTypes, FootSensorEscSdoTypes>,
    public PDO_log<FootSensorEscPdoTypes::pdo_rx> {
public:
    typedef BasicEscWrapper<FootSensorEscPdoTypes,FootSensorEscSdoTypes>       Base;
    typedef PDO_log<FootSensorEscPdoTypes::pdo_rx>                             Log;

public:
    FootSensorESC ( const ec_slavet& slave_descriptor ) :
        Base ( slave_descriptor ),
        Log ( std::string ( "/tmp/FootSensorESC_pos"+std::to_string ( position ) +"_log.txt" ),DEFAULT_LOG_SIZE )
    { }

    virtual ~FootSensorESC ( void ) {
        delete [] SDOs;
        DPRINTF ( "~%s pos %d\n", typeid ( this ).name(), position );
        print_stat ( s_rtt );
    }

    int set_cal_matrix ( std::vector<std::vector<float>> &cal_matrix );

    virtual const objd_t * get_SDOs() {
        return SDOs;
    }
    virtual void init_SDOs ( void );
    virtual uint16_t get_ESC_type() {
        return FOOT_SENSOR;
    }

    virtual void on_writePDO ( void ) {

        tx_pdo.ts = ( uint16_t ) ( get_time_ns() /1000 );
    }

    virtual void on_readPDO ( void ) {

        if ( rx_pdo.rtt ) {
            rx_pdo.rtt = ( uint16_t ) ( get_time_ns() /1000 ) - rx_pdo.rtt;
            s_rtt ( rx_pdo.rtt );
        }

        if ( rx_pdo.fault ) {
            handle_fault();
        } else {
            // clean any previuos fault ack !!
            //tx_pdo.fault_ack = 0;
        }

        if ( _start_log ) {
            Log::log_t log;
            memcpy(log.forceXY, rx_pdo.forceXY, sizeof(rx_pdo.forceXY));
            log.fault       = rx_pdo.fault;
            log.rtt         = rx_pdo.rtt;
            push_back ( log );
        }

    }

    int16_t get_robot_id() {
        //assert(sdo.Joint_robot_id != -1);
        return sdo.sensor_robot_id;
    }

    void print_info ( void ) {
        DPRINTF ( "\tSensor id %d\tSensor robot id %d\n", sdo.sensor_number, sdo.sensor_robot_id );
        DPRINTF ( "\tfw_ver %s\n", sdo.firmware_version );
    }

    virtual int init ( const YAML::Node & root_cfg ) {

        int16_t robot_id = -1;

        try {
            init_SDOs();
            init_sdo_lookup();
            readSDO_byname ( "Sensor_robot_id", robot_id );
            set_flash_cmd_X ( this, CTRL_REMOVE_TORQUE_OFFS );

        } catch ( EscWrpError &e ) {

            DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
            return EC_BOARD_INIT_SDO_FAIL;
        }

#if 0
        if ( robot_id > 0 ) {
            try {
                std::string esc_conf_key = std::string ( "FootSensorESC_"+std::to_string ( robot_id ) );
                const YAML::Node& esc_conf = root_cfg[esc_conf_key];
                if ( esc_conf.Type() != YAML::NodeType::Null ) {
                }
            } catch ( YAML::KeyNotFound &e ) {
                DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
                return EC_BOARD_KEY_NOT_FOUND;
            }
        }
#endif
        // set filename with robot_id
        log_filename = std::string ( "/tmp/FootSensorESC_"+std::to_string ( sdo.sensor_robot_id ) +"_log.txt" );

        // we log when receive PDOs
        start_log ( true );

        return EC_BOARD_OK;

    }

    virtual void start_log ( bool start ) {
        Log::start_log ( start );
    }

    virtual void handle_fault ( void ) {

        fault_t fault;
        fault.all = rx_pdo.fault;
        //fault.bit.
        //ack_faults_X(this, fault.all);

    }

private:
    stat_t  s_rtt;
    objd_t * SDOs;
};


typedef std::map<int, FootSensorESC*>  FootSensorSlavesMap;

inline int FootSensorESC::set_cal_matrix ( std::vector<std::vector<float>> &cal_matrix ) {
    int     res = 0;
    int16_t ack;
    int16_t flash_row_cmd = 0x00C7;

    for ( int r=0; r<6; r++ ) {

        // set row n param value
        writeSDO_byname ( "matrix_rn_c1",cal_matrix[r][0] );
        writeSDO_byname ( "matrix_rn_c2",cal_matrix[r][1] );
        writeSDO_byname ( "matrix_rn_c3",cal_matrix[r][2] );
        writeSDO_byname ( "matrix_rn_c4",cal_matrix[r][3] );
        writeSDO_byname ( "matrix_rn_c5",cal_matrix[r][4] );
        writeSDO_byname ( "matrix_rn_c6",cal_matrix[r][5] );

        if ( set_flash_cmd_X ( this, flash_row_cmd ) != EC_BOARD_OK ) {
            return EC_BOARD_FT6_CALIB_FAIL;
        }
        // next row cmd
        flash_row_cmd++;

    } // for rows

    return EC_BOARD_OK;
}



}
}
}
#endif /* __IIT_ECAT_ADVR_FOOT_SENSOR_ESC_H__ */
