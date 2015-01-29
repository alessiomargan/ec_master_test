/*
 *
 *  Created on: Dec, 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_TEST_ESC_H__
#define __IIT_ECAT_ADVR_TEST_ESC_H__

#include <iit/ecat/advr/esc.h>

#include <map>
#include <string>


namespace iit {
namespace ecat {
namespace advr {


struct TestEscPdoTypes {
    // TX  slave_input -- master output
    typedef struct {
        uint16_t    _type;
        int32_t     _value;
        uint64_t    _ts;
    } __attribute__((__packed__)) pdo_tx;
    // RX  slave_output -- master input
    typedef struct {
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
        void sprint(char *buff, size_t size) {
            snprintf(buff, size, "%ld\t%lu\n", _lint,_ulint);
        }
        void fprint(FILE *fp) {
            fprintf(fp, "%ld\t%lu\n", _lint,_ulint);
        }
    } __attribute__((__packed__)) pdo_rx;
};


struct TestEscSdoTypes {
    // flash param
    int	    par_1;
    int	    par_2;

    // ram param
    //float		dummy;
};

class TestESC :
public BasicEscWrapper<TestEscPdoTypes,TestEscSdoTypes>,
public PDO_log<TestEscPdoTypes>
{

public:
    typedef BasicEscWrapper<TestEscPdoTypes,TestEscSdoTypes>    Base;
    typedef PDO_log<TestEscPdoTypes>                            Log;

    TestESC(const ec_slavet& slave_descriptor) :
        Base(slave_descriptor), Log(std::string("/tmp/ESC_log.txt"),1000)
    {
        init_SDOs();
        init_sdo_lookup();
    }

    virtual ~TestESC(void) {
        delete [] SDOs;
        DPRINTF("~%s %d\n", typeid(this).name(), position);
    }

    virtual void on_readPDO(void) {
        push_back(rx_pdo);
    }
    virtual void on_writePDO(void) {
        tx_pdo._ts = get_time_ns();
    }
 
    virtual const objd_t * get_SDO_objd() { return SDOs; }
    virtual void init_SDOs(void);

private:
    objd_t * SDOs;

};



}
}
}

#endif


