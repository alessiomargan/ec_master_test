/*
 * motor controller.h
 *
 *  Created on: Jun 26, 2014
 *      Author: mfrigerio
 */

#ifndef __IIT_ECAT_ADVR_ESC_H__
#define __IIT_ECAT_ADVR_ESC_H__

#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/utils.h>

//#include <stdint.h>

namespace iit {
namespace ecat {
namespace advr {

typedef struct
{
    int index;
    int subindex;
    int datatype;
    int bitlength;
    int access;
    void * data;
    const char * name;
} objd_t;


struct TestESCTypes {
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
    } __attribute__((__packed__)) pdo_rx;
};

class TestESC : public BasicEscWrapper<TestESCTypes>
{
public:
    typedef BasicEscWrapper<TestESCTypes> Base;
public:
    TestESC(const ec_slavet& slave_descriptor) :
           Base(slave_descriptor)
       {}
    virtual ~TestESC(void) { DPRINTF("~%s %d\n", typeid(this).name(), position); }
};



} 
}
}

#endif /* IIT_ECAT_ADVR_ESC_H_ */
