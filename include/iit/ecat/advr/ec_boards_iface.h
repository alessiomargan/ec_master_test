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
#include <iit/ecat/advr/hub_esc.h>
#include <iit/ecat/advr/power_board.h>
#include <iit/ecat/advr/power_coman_board.h>

#include <string>
#include <mutex>
#include <algorithm>

#include <yaml-cpp/yaml.h>

// [ECat_master] o: 410   i: 788 = 1198

namespace iit {
namespace ecat {
namespace advr {


enum Robot_IDs 
{ 
    // neck
    HEAD_R = 1,
    HEAD_P,
    
    // right arm
    RA_SH_1 = 11,
    RA_SH_2,
    RA_SH_3,
    RA_EL,
    RA_WR_1,
    RA_WR_2,
    RA_WR_3,
    RA_FT,
    RA_HA,

    // left arm
    LA_SH_1 = 21,
    LA_SH_2,
    LA_SH_3,
    LA_EL,
    LA_WR_1,
    LA_WR_2,
    LA_WR_3,
    LA_FT,
    LA_HA,
    
    // waist
    WAIST_Y = 31,
    WAIST_R,
    WAIST_P,
    
    // right leg
    RL_H_R = 41,
    RL_H_Y,
    RL_H_P,
    RL_K,
    RL_A_P,
    RL_A_R,
    RL_FT,
    
    // left leg
    LL_H_R = 51,
    LL_H_Y,
    LL_H_P,
    LL_K,
    LL_A_P,
    LL_A_R,
    LL_FT,
}; 



typedef std::map<int, int>  Rid2PosMap;
typedef std::map<int, int>  Pos2RidMap;


/**
 * @class Ec_Boards_ctrl
 *  
 * @brief Boards_ctrl 
 */

class Ec_Boards_ctrl {

public:
    Ec_Boards_ctrl(std::string config);
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
     * @brief get RxPDO of the slave @p slave_index
     * @note This will not receive anything, it will just return 
     *       copy of the last RxPDO received!
     * @param slave_index id of the slave
     * @param iit::ecat::advr::McESCTypes::pdo_rx&
     */
    int getRxPDO(int slave_index, McEscPdoTypes::pdo_rx &pdo);
    int getRxPDO(int slave_index, Ft6EscPdoTypes::pdo_rx &pdo);

    /**
     * @brief set TxPDO of the slave @p slave_index
     * @note This will not send anything, it will just copy the 
     *       TxPDO so that send_to_slaves() can send it
     * @param slave_index id of the slave
     * @param iit::ecat::advr::McESCTypes::pdo_rx&
     * @return void
     */
    int setTxPDO(int slave_index, McEscPdoTypes::pdo_tx pdo);
    int setTxPDO(int slave_index, Ft6EscPdoTypes::pdo_tx pdo);
    /**
     * @brief returns the TxPDO of the slave @p slave_index
     * @note Just return a copy of the last TxPDO sent!
     * @param slave_index id of the slave
     * @param iit::ecat::advr::McESCTypes::pdo_rx&
     */
    int getTxPDO(int slave_index, McEscPdoTypes::pdo_tx &pdo);
    int getTxPDO(int slave_index, Ft6EscPdoTypes::pdo_tx &pdo);

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
    int set_ctrl_status(uint16_t sPos, int16_t cmd);
    int set_flash_cmd(uint16_t sPos, int16_t cmd);
    int set_cal_matrix(uint16_t sPos, std::vector<std::vector<float>> &cal_matrix);

    /**
     * @brief Checks if temperature and currents in the boards are fine
     * 
     * @return int
     */
    int check_sanity(uint16_t sPos);
    void check_DataLayer(void);
    
    void start_motors(int);
    void stop_motors(void);
    /**
     * @brief update slave firmware using FOE
     * 
     * @return int
     */
    int update_board_firmware(uint16_t slave_pos, std::string firmware, uint32_t passwd_firm);

    EscWrapper * slave_as_EscWrapper(uint16_t sPos) { return(slaves.find(sPos) != slaves.end()) ? slaves[sPos].get() : NULL;}
    EscWrapper * slave_as_Zombie(uint16_t sPos) { return(zombies.find(sPos) != zombies.end()) ? zombies[sPos].get() : NULL;}

    Motor * slave_as_Motor(uint16_t sPos) { return(slaves.find(sPos) != slaves.end()) ? dynamic_cast<Motor*>(slaves[sPos].get()) : NULL;}
    //Motor *  slave_as_Motor(uint16_t sPos) { return dynamic_cast<Motor*>(slaves.at(sPos).get()); }

    HpESC *  slave_as_HP(uint16_t sPos) { return(slaves.find(sPos) != slaves.end()) ? dynamic_cast<HpESC*>(slaves[sPos].get()) : NULL;}
    LpESC *  slave_as_LP(uint16_t sPos) { return(slaves.find(sPos) != slaves.end()) ? dynamic_cast<LpESC*>(slaves[sPos].get()) : NULL;}
    Ft6ESC * slave_as_FT(uint16_t sPos) { return(slaves.find(sPos) != slaves.end()) ? dynamic_cast<Ft6ESC*>(slaves[sPos].get()) : NULL;}

    const YAML::Node & get_config_YAML_Node(void) { return root_cfg; }

    const Rid2PosMap & get_Rid2PosMap(void) { return rid2pos; }
    int rid2Pos(int rId) { return rid2pos.find(rId) != rid2pos.end() ? rid2pos[rId] : 0; }

    const Pos2RidMap & get_Pos2RidMap(void) { return pos2rid; }
    int pos2Rid(int pos) { return pos2rid.find(pos) != pos2rid.end() ? pos2rid[pos] : 0; }

    //template <typename T, typename A>
    //int get_esc_bytype(uint16_t ESC_type, std::vector<T,A> &esc_bytype) {
    template <typename T>
    int get_esc_map_bytype(uint16_t ESC_type, std::map<int, T> &esc_bytype) {

        esc_bytype.clear();
        for (auto it = slaves.begin(); it != slaves.end(); it++ ) {
            EscWrapper * esc = it->second.get();
            if ( esc->get_ESC_type() == ESC_type ) {
                //esc_bytype.push_back( dynamic_cast<T>(it->second.get()) );
                esc_bytype[it->first] = dynamic_cast<T>(it->second.get());
            }
        }
        return esc_bytype.size();
    }

    template <typename T>
    int get_esc_map_byclass(std::map<int, T> &esc_bytype) {

	EscWrapper * esc;
	T t;
        esc_bytype.clear();
        for (auto it = slaves.begin(); it != slaves.end(); it++ ) {
            EscWrapper * esc = it->second.get();
            t = dynamic_cast<T>(esc);
	    if ( t ) {
                esc_bytype[it->first] = t;
            }
        }
        return esc_bytype.size();
    }

    template <typename T>
    int get_esc_map_byclass(std::map<int, T> &esc_bytype, std::vector<int> rId_vect) {

	EscWrapper * esc;
	T t;
	// N log N
	std::sort(rId_vect.begin(), rId_vect.end());
        esc_bytype.clear();
        for ( auto const& item : slaves ) {
            EscWrapper * esc = item.second.get();
            t = dynamic_cast<T>(esc);
	    // log N + O(1)
	    if ( t && std::binary_search(rId_vect.begin(), rId_vect.end(), pos2Rid(item.first)) ) {
                esc_bytype[item.first] = t;
            }
        }
        return esc_bytype.size();
    }

    template <typename T>
    int get_zombie_map_bytype(uint16_t ESC_type, std::map<int, T> &esc_bytype) {

        esc_bytype.clear();
        for (auto it = zombies.begin(); it != zombies.end(); it++ ) {
            EscWrapper * esc = it->second.get();
            if ( esc->get_ESC_type() == ESC_type ) {
                //esc_bytype.push_back( dynamic_cast<T>(it->second.get()) );
                esc_bytype[it->first] = dynamic_cast<T>(it->second.get());
            }
        }
        return esc_bytype.size();
    }
     
    void rd_LOCK(void);
    void rd_UNLOCK(void);
    void wr_LOCK(void);
    void wr_UNLOCK(void);

protected:

    void factory_board(void);

    SlavesMap   slaves;
    SlavesMap   zombies;
    
    Rid2PosMap  rid2pos;
    Pos2RidMap  pos2rid;
    YAML::Node  root_cfg;

private:


    int             slave_cnt;
    int             expected_wkc;
    ec_timing_t     timing;

    std::string     eth_if;

    uint64_t    sync_cycle_time_ns;
    uint64_t    sync_cycle_offset_ns;


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
