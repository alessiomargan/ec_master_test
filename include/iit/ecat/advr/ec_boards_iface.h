/*
   ec_boards_iface.h

   Copyright (C) 2014 Italian Institute of Technology

   Developer:
       Alessio Margan (2014-, alessio.margan@iit.it)

*/


#ifndef __EC_BOARDS_IFACE_H__
#define __EC_BOARDS_IFACE_H__

#include <iit/ecat/ec_master_iface.h>
#include <iit/ecat/slave_wrapper.h>

#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/mc_hipwr_esc.h>

#include <string>
#include <mutex>

namespace iit {
namespace ecat {
namespace advr {


struct info_item
{
    int index;
    int sub_index;
    int size;
};

/**
 * @class Ec_Boards_ctrl
 * @brief Boards_ctrl class
 */

class Ec_Boards_ctrl {

public:
    Ec_Boards_ctrl(const char * config);
    ~Ec_Boards_ctrl();

    int init(void);

    void configure_boards(void);

    int set_operative();

    const McESCTypes::pdo_rx& getRxPDO(int slave_index);
    void setTxPDO(int slave_index, McESCTypes::pdo_tx pdo);
    
    int recv_from_slaves(void);
    int send_to_slaves(void);
    
    uint64_t mailbox_recv_from_slaves_as_int(int slave_index, std::string token);
    std::string mailbox_recv_from_slaves_as_string(int slave_index, std::string token);
    float mailbox_recv_from_slaves_as_float(int slave_index, std::string token);
    
    int mailbox_recv_from_slaves(int slave_index, std::string token, void* data);
    
    int mailbox_send_to_slaves(int slave_index, std::string token, void* data);

    inline int get_number_of_boards(){return slaves.size();};
protected:

    void factory_board(void);

    int set_param(int slave_pos, int index, int subindex, int size, void *data);
    int get_param(int slave_pos, int index, int subindex, int *size, void *data);

    SlavesMap slaves;

private:

    int             expected_wkc;
    ec_timing_t     timing;

    std::string     eth_if;

    uint64_t    sync_cycle_time_ns;
    uint64_t    sync_cycle_offset_ns;
    std::map<int,McESCTypes::pdo_rx> RxPDO_map;
    std::map<int,McESCTypes::pdo_tx> TxPDO_map;
    std::map<std::string, info_item> info_map;
    std::mutex rd_mtx, wr_mtx;
    
    bool get_info(std::string token,int& main_index,int& sub_index, int& size);
    void set_info_table();
    
};


} 
}
}

#endif
