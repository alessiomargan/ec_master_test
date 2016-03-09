/*
 *
 *  Created on: Dec, 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_TEST_ESC_H__
#define __IIT_ECAT_ADVR_TEST_ESC_H__

#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/log_esc.h>

#include <map>
#include <string>


namespace iit {
namespace ecat {
namespace advr {


struct TestEscPdoTypes {
    // TX  slave_input -- master output
    struct pdo_tx {
        uint16_t    _type;
        int32_t     _value;
        uint64_t    _ts;
    } __attribute__ ( ( __packed__ ) );

    // RX  slave_output -- master input
    struct pdo_rx {
        uint8_t     _bit_0:1;
        uint8_t     _bit_1:1;
        uint8_t     _bit_2:1;
        uint8_t     _bit_3:1;
        uint8_t     _bit_4:1;
        uint8_t     _bit_5:1;
        uint8_t     _bit_6:1;
        uint8_t     _bit_7:1;
        uint8_t     _bits;
        int8_t      _sint;
        uint8_t     _usint;
        int16_t     _int;
        uint16_t    _uint;
        int32_t     _dint;
        uint32_t    _udint;
        int64_t     _lint;
        uint64_t    _ulint;
        float       _real;
        int sprint ( char *buff, size_t size ) {
            return snprintf ( buff, size, "%ld\t%ld\t%f", _lint,_ulint,_real );
        }
        void fprint ( FILE *fp ) {
            fprintf ( fp, "%ld\t%ld\t%f\n", _lint,_ulint,_real );
        }
        void to_map ( jmap_t & jpdo ) {
            JPDO ( _int );
            JPDO ( _ulint );
            JPDO ( _real );
        }
    } __attribute__ ( ( __packed__ ) );
};


struct TestEscSdoTypes {
    // flash param
    int	    par_1;
    int	    par_2;

    // ram param
    //float		dummy;
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

        if ( rx_pdo._ulint ) {
            rx_pdo._ulint =  get_time_ns() - rx_pdo._ulint;
            s_rtt ( rx_pdo._ulint );
        }

        if ( _start_log ) {
            Log::log_t log;
            log.ts = get_time_ns() - _start_log_ts ;
            log.rx_pdo = rx_pdo;
            push_back ( log );
        }

    }

    virtual void on_writePDO ( void ) {
        tx_pdo._ts = get_time_ns();
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

            // !! sgn and offset must set before init_sdo_lookup !!
            init_SDOs();
            init_sdo_lookup();

            int32_t par_1, par_2;

            sdo.par_1 = 123;
            sdo.par_2 = 999;
            //getSDO_byname("par_1", par_1);
            readSDO_byname ( "par_1", par_1 );
            writeSDO_byname ( "par_2", par_2 );
            writeSDO_byname<int32_t> ( "par_2", 777 );
            int32_t test;
            readSDO_byname ( "par_2", test );
            assert ( test == 777 );

        } catch ( EscWrpError &e ) {

            DPRINTF ( "Catch Exception %s ... %s\n", __FUNCTION__, e.what() );
            return EC_WRP_NOK;
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
