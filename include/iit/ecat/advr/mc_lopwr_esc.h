/*
 * mc_lopwr_esc.h
 * 
 *  LowPower Motor Controlleer 
 *  based on TI TM4C123AH6PM - Tiva C Series Microcontroller
 *  32-bit ARM Cortex-M4 80 MHz
 *  
 *  http://www.ti.com/product/tm4c123ah6pm
 *  
 *  Created on: Dec 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_MC_LOPWR_ESC_H__
#define __IIT_ECAT_ADVR_MC_LOPWR_ESC_H__

#include <iit/ecat/slave_wrapper.h>

namespace iit {
namespace ecat {
namespace advr {


/**
 *  
 **/ 

struct McTestESCTypes {
    // TX  slave_input -- master output
    typedef struct {
        uint16_t    _type;
        int32_t     _value;
        uint64_t    _ts;
    } __attribute__((__packed__)) pdo_tx;


    // RX  slave_output -- master input
    typedef struct {
        // 2x qei 
        uint32_t    rel_enc[2];
        // spi 3 chip 
        uint32_t    abs_enc[2];
        // spi 2 chip : torque + 3 spare channel
        uint16_t    ain[4];
        // adc : temperature + 3 phase current
        uint32_t    adc[4];
        // 3 digital input
        uint16_t    hall;
        // pwm
        uint32_t    pwm[6];
        // round trip time
        uint64_t    rtt;
        // temperature
        float       temp;  

    } __attribute__((__packed__)) pdo_rx;
};

class McTestESC : public BasicEscWrapper<McTestESCTypes>
{
public:
    typedef BasicEscWrapper<McTestESCTypes> Base;
public:
    McTestESC(const ec_slavet& slave_descriptor) :
           Base(slave_descriptor)
       {}
};


/**
 *  
 **/ 

struct McESCTypes {
    // TX  slave_input -- master output
    typedef struct {
        float	    pos_ref;
        float		tor_ref;
        float		direct_ref;
        uint64_t	ts;
    }  __attribute__((__packed__)) pdo_tx;

    // RX  slave_output -- master input
    typedef struct {
        float	    position;   // rad
        float		velocity;   // rad/s
        float		torque;     // Nm
        float		torque_D;   // Nm/s
        float		direct_out; // A in AC or V in DC
        uint16_t    fault;
        uint64_t	rtt;        // ns 
    }  __attribute__((__packed__)) pdo_rx;
};

class McESC : public BasicEscWrapper<McESCTypes>
{
public:
    typedef BasicEscWrapper<McESCTypes> Base;
public:
    McESC(const ec_slavet& slave_descriptor) :
           Base(slave_descriptor)
       {}
};




} 
}
}

#endif /* IIT_ECAT_ADVR_ESC_H_ */
