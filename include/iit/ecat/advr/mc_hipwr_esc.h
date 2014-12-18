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

#include <iit/ecat/advr/pipes.h>
#include <json_serialization.hpp>
#include <iit/ecat/advr/types.h>
#include <json/json.h>
#include <map>

namespace iit {
namespace ecat {
namespace advr {

typedef std::map<std::string, std::string> pdo_map_t;

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

        std::string pipe_name = "from_esc_" + std::to_string(position);
        xddp_wr = new Write_XDDP_pipe(pipe_name, 16384);
        pipe_name = "to_esc_" + std::to_string(position);
        xddp_rd = new Read_XDDP_pipe(pipe_name, 16384);

    }

    int write_pdo_to_pipe();
    int read_pdo_from_pipe();

    Write_XDDP_pipe *   xddp_wr;
    Read_XDDP_pipe *    xddp_rd;

private:
    json_serializer serializer;
    pdo_map_t      mc_pdo_map;


};



} 
}
}

#endif /* IIT_ECAT_ADVR_ESC_H_ */
