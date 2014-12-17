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

#include <json/json.h>

namespace iit {
namespace ecat {
namespace advr {

#define MAP_ITEM(x) [#x] = std::to_string(x)

typedef std::map<std::string, std::string> pdo_map_t;

#define params(x) #x,x

inline void add_object(std::string name, float value, json_object* jobj)
{
    json_object *jfloat  = json_object_new_double(value);
    json_object_object_add(jobj,name.c_str(),jfloat);
}

inline void add_object(std::string name, uint64_t value, json_object* jobj)
{
    json_object *jn  = json_object_new_int64(value);
    json_object_object_add(jobj,name.c_str(),jn);
}

inline void add_object(std::string name, uint16_t value, json_object* jobj)
{
    json_object *jn  = json_object_new_int(value);
    json_object_object_add(jobj,name.c_str(),jn);
}

inline void add_object(std::string name, long value, json_object* jobj)
{
    json_object *jn  = json_object_new_int(value);
    json_object_object_add(jobj,name.c_str(),jn);
}


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

        void serializeToJson(json_object* jObj);
        void deSerializeToJson(json_object *jObj);        

    }  /*__attribute__((__packed__))*/ pdo_tx;

    // RX  slave_output -- master input
    typedef struct {
        float	    position;   // rad
        float		velocity;   // rad/s
        float		torque;     // Nm
        float		torque_D;   // Nm/s
        float		direct_out; // A in AC or V in DC
        uint16_t    fault;
        uint64_t	rtt;        // ns 

        void serializeToJson(json_object* jObj);
        void deSerializeToJson(json_object *jObj);  

    }  /*__attribute__((__packed__))*/ pdo_rx;
};



inline void McESCTypes::pdo_tx::serializeToJson(json_object* jObj)
{
    /*
    float        pos_ref;
    float        tor_ref;
    float        direct_ref;
    uint64_t    ts;
    */

    add_object(params(pos_ref),jObj);
    add_object(params(tor_ref),jObj);
    add_object(params(direct_ref),jObj);
    add_object(params(ts),jObj);
    
}

inline void McESCTypes::pdo_rx::serializeToJson(json_object* jObj)
{
    /*
    float        position;   // rad
    float        velocity;   // rad/s
    float        torque;     // Nm
    float        torque_D;   // Nm/s
    float        direct_out; // A in AC or V in DC
    uint16_t    fault;
    uint64_t    rtt;        // ns 
    */

    add_object(params(position),jObj);
    add_object(params(velocity),jObj);
    add_object(params(torque),jObj);
    add_object(params(torque_D),jObj);
    add_object(params(direct_out),jObj);
    add_object(params(fault),jObj);
    add_object(params(rtt),jObj);
    
}

inline void get_object(std::string name, float &value, json_object* jobj)
{
    json_object *jfloat  = json_object_object_get(jobj,name.c_str());
    value = json_object_get_double(jfloat);
}

inline void get_object(std::string name, uint64_t& value, json_object* jobj)
{
    json_object *jn  = json_object_object_get(jobj,name.c_str());
    value = json_object_get_int64(jn);
}

inline void get_object(std::string name, uint16_t& value, json_object* jobj)
{
    json_object *jn  = json_object_object_get(jobj,name.c_str());
    value = json_object_get_int(jn);
}

inline void get_object(std::string name, long& value, json_object* jobj)
{
    json_object *jn  = json_object_object_get(jobj,name.c_str());
    value = json_object_get_int(jn);
}
  
inline void McESCTypes::pdo_tx::deSerializeToJson(json_object* jObj)
{
    /*
    float        pos_ref;
    float        tor_ref;
    float        direct_ref;
    uint64_t    ts;
    */

    get_object(params(pos_ref),jObj);
    get_object(params(tor_ref),jObj);
    get_object(params(direct_ref),jObj);
    get_object(params(ts),jObj);
}

inline void McESCTypes::pdo_rx::deSerializeToJson(json_object* jObj)
{
    /*
    float        position;   // rad
    float        velocity;   // rad/s
    float        torque;     // Nm
    float        torque_D;   // Nm/s
    float        direct_out; // A in AC or V in DC
    uint16_t    fault;
    uint64_t    rtt;        // ns 
    */

    get_object(params(position),jObj);
    get_object(params(velocity),jObj);
    get_object(params(torque),jObj);
    get_object(params(torque_D),jObj);
    get_object(params(direct_out),jObj);
    get_object(params(fault),jObj);
    get_object(params(rtt),jObj);
}



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
