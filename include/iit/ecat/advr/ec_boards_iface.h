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
    const objd_t* sdo_ptr;
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

    /**
     * @brief Initializes the ethercat driver and creates the boards pointers
     * 
     * @return 1 on success, 0 on failure
     */
    int init(void);

    /**
     * @brief reads and sets SDO, configure boards parameters
     * 
     * @return void
     */
    void configure_boards(void);

    /**
     * @brief Starts the communication of the ethercat slaves (moves from starting to operative)
     * 
     * @return int the number of boards that acknowledged the operative request
     */
    int set_operative();

    /**
     * @brief returns the PDO of the slave @p slave_index
     * @note This will not receive anything, it will just return a copy of the last PDO received!
     * @param slave_index id of the slave
     * @return const iit::ecat::advr::McESCTypes::pdo_rx&
     */
    const McESCTypes::pdo_rx& getRxPDO(int slave_index);
    /**
     * @brief Sends a PDO to a slave
     * @note This will not send anything, it will just copy the PDO so that send_to_slaves() can send it     * 
     * @param slave_index id of the slave
     * @param pdo data to write
     * @return void
     */
    void setTxPDO(int slave_index, McESCTypes::pdo_tx pdo);
    
    /**
     * @brief This will receive a running train from all the slaves, and will fill the RxPDO_map returned from getRxPDO()
     * 
     * @return int the number of boards that returned a PDO
     */
    int recv_from_slaves(void);
    /**
     * @brief This will send a running train to all the slaves, using the TxPDO_map set with setTxPDO()
     * 
     * @return int the number of boards that received the PDO
     */
    int send_to_slaves(void);
    
    /**
     * @brief Receives a specific parameter from the board @p slave_index
     * @note This method send a request packet and receive a return packet, the frequency of the calls to this method should not be higher than 5 hz
     * @param slave_index id of the slave
     * @param token name of the parameter to receive
     * @return uint64_t
     */
    uint64_t mailbox_recv_from_slaves_as_int(int slave_index, std::string token);
    /**
     * @brief @see mailbox_recv_from_slaves_as_int
     * 
     * @param slave_index id of the slave
     * @param token 
     * @return std::string
     */
    std::string mailbox_recv_from_slaves_as_string(int slave_index, std::string token);
    /**
     * @brief @see mailbox_recv_from_slaves_as_int
     * 
     * @param slave_index id of the slave
     * @param token 
     * @return float
     */
    float mailbox_recv_from_slaves_as_float(int slave_index, std::string token);
    
    /**
     * @brief Receives the parameter @p token from the slave @p slave_index, putting it into @p data
     * 
     * @param slave_index id of the slave
     * @param token name of the parameter to receive
     * @param data[out] value of the parameter
     * @return int number of slaves that provided the parameter (should be 1)
     */
    int mailbox_recv_from_slaves(int slave_index, std::string token, void* data);
    
    /**
     * @brief Sends the parameter @p token with value @p data to the slave @p slave_index
     * 
     * @param slave_index id of the slave
     * @param token name of the parameter to send
     * @param data value of the parameter
     * @return int number of slaves that received the parameter (should be 1)
     */
    int mailbox_send_to_slaves(int slave_index, std::string token, void* data);

    /**
     * @brief 
     * 
     * @return int number of slaves
     */
    inline int get_number_of_boards() {return slaves.size(); };

    /**
     * @brief 
     * 
     * @param sPos 
     * @param cmd 
     * @return int
     */
    int set_ctrl_status(uint16_t sPos, uint16_t cmd);

    /**
     * @brief Checks if temperature and currents in the boards are fine
     * 
     * @return int
     */
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
