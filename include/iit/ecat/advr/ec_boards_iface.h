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
#include <iit/ecat/advr/test_esc.h>
#include <iit/ecat/advr/mc_hipwr_esc.h>
#include <iit/ecat/advr/mc_lowpwr_esc.h>
#include <iit/ecat/advr/ft6_esc.h>

#include <string>
#include <mutex>

#include <boost/variant.hpp>

namespace iit {
namespace ecat {
namespace advr {


struct info_item
{
    const objd_t* sdo_ptr;
    int index;
    int sub_index;
    int size;
};


enum class Board_type 
{ 
    HIGH_POWER,
    LOW_POWER,
    FT6
}; 


enum class Robot_IDs : std::int32_t 
{ 
    RL_H_Y = 41,
    RL_H_R,
    RL_H_P,
    RL_K,
    RL_A_P,
    RL_A_R,

    LL_H_Y = 51,
    LL_H_R,
    LL_H_P,
    LL_K,
    LL_A_P,
    LL_A_R,


}; 


typedef boost::variant<HpESC*, LpESC*, bool> McEscVar;
typedef std::map<int, McEscVar>  McSlavesMap;

typedef std::map<int, int>  Rid2PosMap;


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
    int configure_boards(void);

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
    void getRxPDO(int slave_index, McEscPdoTypes::pdo_rx &pdo);
    void getRxPDO(int slave_index, Ft6EscPdoTypes::pdo_rx &pdo);

    /**
     * @brief Sends a PDO to a slave
     * @note This will not send anything, it will just copy the PDO so that send_to_slaves() can send it     * 
     * @param slave_index id of the slave
     * @param pdo data to write
     * @return void
     */
    void setTxPDO(int slave_index, McEscPdoTypes::pdo_tx pdo);
    void setTxPDO(int slave_index, Ft6EscPdoTypes::pdo_tx pdo);

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
    
#if 0
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

#endif
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
    int set_flash_cmd(uint16_t sPos, uint16_t cmd);
    int set_cal_matrix(uint16_t sPos, std::vector<std::vector<float>> &cal_matrix);

    /**
     * @brief Checks if temperature and currents in the boards are fine
     * 
     * @return int
     */
    int check_sanity(uint16_t sPos);

    /**
     * @brief update slave firmware using FOE
     * 
     * @return int
     */
    int update_board_firmware(uint16_t slave_pos, std::string firmware, uint32_t passwd_firm);

    HpESC *  slave_as_HP(uint16_t sPos) { return(slaves.find(sPos) != slaves.end()) ? dynamic_cast<HpESC*>(slaves[sPos].get()) : NULL;}
    //MpESC *  slave_as_MP(uint16_t sPos) { return(slaves.find(sPos) != slaves.end()) ? dynamic_cast<MpESC*>(slaves[sPos].get()) : NULL;}
    LpESC *  slave_as_LP(uint16_t sPos) { return(slaves.find(sPos) != slaves.end()) ? dynamic_cast<LpESC*>(slaves[sPos].get()) : NULL;}
    Ft6ESC * slave_as_FT(uint16_t sPos) { return(slaves.find(sPos) != slaves.end()) ? dynamic_cast<Ft6ESC*>(slaves[sPos].get()) : NULL;}

    void rd_LOCK(void);
    void rd_UNLOCK(void);
    void wr_LOCK(void);
    void wr_UNLOCK(void);

protected:

    void factory_board(void);

    SlavesMap   slaves;
    //McSlavesMap mcSlaves;
    FtSlavesMap ftSlaves;
    
private:

    int             slave_cnt;
    int             expected_wkc;
    ec_timing_t     timing;

    std::string     eth_if;

    uint64_t    sync_cycle_time_ns;
    uint64_t    sync_cycle_offset_ns;

    std::map<int,Ft6EscPdoTypes::pdo_rx> FtRxPDO_map;
    std::map<int,Ft6EscPdoTypes::pdo_tx> FtTxPDO_map;

    std::map<int,McEscPdoTypes::pdo_rx> McRxPDO_map;
    std::map<int,McEscPdoTypes::pdo_tx> McTxPDO_map;

#ifdef __XENO__
    pthread_mutex_t rd_mtx, wr_mtx;
#else
    std::mutex      rd_mtx, wr_mtx;
#endif    


};


inline void Ec_Boards_ctrl::rd_LOCK(void)
{
#ifdef __XENO__
    pthread_mutex_lock(&rd_mtx);
#else
    std::unique_lock<std::mutex> (rd_mtx);
#endif
}

inline void Ec_Boards_ctrl::rd_UNLOCK(void)
{
#ifdef __XENO__
    pthread_mutex_unlock(&rd_mtx);
#endif
}

inline void Ec_Boards_ctrl::wr_LOCK(void)
{
#ifdef __XENO__
    pthread_mutex_lock(&rd_mtx);
#else
    std::unique_lock<std::mutex> (wr_mtx);
#endif
}

inline void Ec_Boards_ctrl::wr_UNLOCK(void)
{
#ifdef __XENO__
    pthread_mutex_unlock(&wr_mtx);
#endif
}

} 
}
}

#endif
