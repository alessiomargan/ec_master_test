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

inline int McESC::write_pdo_to_pipe() {

    json_object * jObj = json_object_new_object();
    rx_pdo.serializeToJson(jObj);
    std::string json_str;
    json_str = json_object_to_json_string(jObj);
    json_str += "\n";
    // write to pipe or cross domain socket ALL bc data
    //ssize_t nbytes = write(fd_data, (void*)&_ts_bc_data, sizeof(_ts_bc_data));
    // using (rt)xddp or (nrt)fifo could raise different errors 
    // !!!! if no one is reading got error and also other rtpipes do not works !!!!
    // SOLVED with setsockopt XDDP_POOLSZ   
    // (rt) errno ENOMEM 12 --> Cannot allocate memory ...
    // (nrt)errno        11 --> Resource temporarily unavailable
    ssize_t nbytes = xddp_wr->write((void*)json_str.c_str(), json_str.length());
    if (nbytes <= 0 && errno != 11 && errno != 12) { DPRINTF("%d ", errno); perror(">> write to xddp/pipe fail"); }

}

inline int McESC::read_pdo_from_pipe() {

    char buffer[4096]; 
    int i = 0;
    ssize_t nbytes = 0;
    
    advr::McESC::pdo_tx_t tmp_tx;

    memset(buffer, 0, sizeof(buffer));

    // NON-BLOCKING : read binary data
    nbytes = xddp_rd->read((void*)buffer, sizeof(buffer));


    if (nbytes > 0) {
        DPRINTF("read  %d %s\n", nbytes, buffer);

        json_object * jObj = json_tokener_parse(buffer);
    
        tx_pdo.deSerializeToJson(jObj);
               
        DPRINTF( "JSON parse : %s\n", json_object_to_json_string(jObj));
    }
}


} 
}
}

#endif /* IIT_ECAT_ADVR_ESC_H_ */
