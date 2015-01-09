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
#include <map>

#define DTYPE_INTEGER16         0x0001
#define DTYPE_INTEGER32         0x0002
#define DTYPE_UNSIGNED8         0x0003
#define DTYPE_UNSIGNED16        0x0004
#define DTYPE_REAL32            0x0005
#define DTYPE_VISIBLE_STRING    0x0006
#define DTYPE_UNSIGNED64        0x0007

#define ATYPE_RO 17
#define ATYPE_RW 18

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
    const char * name;
    void * data;
} objd_t;


struct McESCTypes {
    // TX  slave_input -- master output
    typedef struct {
        float	    	pos_ref;
        float		tor_offs;
        float		PosGainP;
        float		PosGainI;
        float		PosGainD;
        uint64_t	ts;

    }  __attribute__((__packed__)) pdo_tx;

    // RX  slave_output -- master input
    typedef struct {
        float	    	position;   		// rad
        float		velocity;   		// rad/s
        float		torque;     		// Nm
        float		max_temperature; 	// C
        uint16_t    	fault;
        uint64_t	rtt;        		// ns
    }  __attribute__((__packed__)) pdo_rx;
};


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



class McESC : public BasicEscWrapper<McESCTypes>
{

public:
    typedef BasicEscWrapper<McESCTypes> Base;
public:
    McESC(const ec_slavet& slave_descriptor) :
           Base(slave_descriptor) {
    }

    virtual ~McESC(void) { DPRINTF("~%s %d\n", typeid(this).name(), position); }

public:
    
    virtual const objd_t * get_SDOs() = 0;
    virtual const objd_t * get_SDOs6000() = 0;
    virtual const objd_t * get_SDOs7000() = 0;
    virtual const objd_t * get_SDOs8000() = 0;
    virtual const objd_t * get_SDOs8001() = 0;
    
    static McESCTypes::pdo_rx sdo_rx_pdo;
    static McESCTypes::pdo_tx sdo_tx_pdo;

};


// typedef std::shared_ptr<McESC>  McESCPtr;
typedef std::map<int, McESC*>  McSlavesMap;







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
