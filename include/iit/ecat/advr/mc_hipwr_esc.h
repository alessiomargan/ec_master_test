/*
 * mc_hipwr_esc.h
 * 
 *  HiPower Motor Controlleer 
 *  based on TI TMS320F28335 - Delfino Microcontroller
 *  High-Performance 32-Bit CPU 150 Mhz
 *  
 *  http://www.ti.com/product/tms320f28335
 *  
 *  Created on: Dec 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_MC_HIPWR_ESC_H__
#define __IIT_ECAT_ADVR_MC_HIPWR_ESC_H__

#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/advr/esc.h>
#include <iit/mc_tm4c/types.h>
#include <map>
#include <pwd.h>

namespace iit {
namespace ecat {
namespace advr {

/**
 *  
 **/ 

class McESC : public BasicEscWrapper<McESCTypes>
{
public:
    typedef BasicEscWrapper<McESCTypes> Base;
public:
    McESC(const ec_slavet& slave_descriptor) :
           Base(slave_descriptor) {
    }

    virtual ~McESC(void) {
    }

private:
};



} 
}
}

#endif /* IIT_ECAT_ADVR_ESC_H_ */
