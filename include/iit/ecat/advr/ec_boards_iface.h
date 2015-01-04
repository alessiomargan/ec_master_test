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

/*
CTRL_SET_DIRECT_MODE correnti nulle
CTRL_POWER_MOD_ON 
tx_pdo.pos_ref = rx_pdo.position 
CTRL_SET_POS_MODE / CTRL_SET_IMPED_MODE
*/

// Control commands
#define CTRL_POWER_MOD_ON		0x00A5
#define CTRL_POWER_MOD_OFF		0x005A
#define CTRL_SET_IMPED_MODE		0x00D4
#define CTRL_SET_POS_MODE		0x003B
#define CTRL_SET_DIRECT_MODE	0x004F
#define CTRL_FAN_ON				0x0026
#define CTRL_FAN_OFF			0x0062
#define CTRL_LED_ON				0x0019
#define CTRL_LED_OFF			0x0091
#define CTRL_ALIGN_ENCODERS		0x00B2
#define CTRL_SET_ZERO_POSITION	0x00AB
#define CTRL_REMOVE_TORQUE_OFFS	0x00CD

#define CTRL_CMD_DONE			0x7800
#define CTRL_CMD_ERROR			0xAA00


struct info_item
{
    int index;
    int sub_index;
    int size;
};



/**
 * TODO .... The Facade Pattern provides a unified interface to 
 * a set of interfaces in a subsystem. Facade defines a 
 * higher-level interface that makes the subsystem easier to 
 * use. 
 *  
 * @class Ec_Boards_ctrl
 *  
 * @brief Boards_ctrl 
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

    inline int get_number_of_boards() {return slaves.size(); };

    int set_ctrl_status(uint16_t sPos, uint16_t cmd);

    int check_sanity();

protected:

    void factory_board(void);

    int set_SDO(int slave_pos, int index, int subindex, int size, void *data);
    int set_SDO(int slave_pos, const objd_t *sdo);

    int get_SDO(int slave_pos, int index, int subindex, int *size, void *data);
    int get_SDO(int slave_pos, const objd_t *sdo);

    SlavesMap   slaves;
    McSlavesMap mcSlaves;

private:

    int             slave_cnt;
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
