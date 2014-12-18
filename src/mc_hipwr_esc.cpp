#include <string.h>

#include "iit/ecat/advr/mc_hipwr_esc.h"
#include "iit/ecat/utils.h"
#include "json_serialization.hpp"


using namespace iit::ecat::advr;

int McESC::write_pdo_to_pipe() {
    json_object * jObj = json_object_new_object();
    serializer.serializeToJson(rx_pdo,jObj);
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


int McESC::read_pdo_from_pipe() {
    char buffer[4096]; 
    int i = 0;
    ssize_t nbytes = 0;

    memset(buffer, 0, sizeof(buffer));

    // NON-BLOCKING : read binary data
    nbytes = xddp_rd->read((void*)buffer, sizeof(buffer));

    if (nbytes > 0) {
        DPRINTF("read  %ld %s\n", nbytes, buffer);

        json_object * jObj = json_tokener_parse(buffer);
        serializer.deSerializeToJson(tx_pdo,jObj);

        DPRINTF( "JSON parse : %s\n", json_object_to_json_string(jObj));
    }
}
