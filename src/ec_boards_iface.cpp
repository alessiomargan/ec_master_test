#include <iit/ecat/advr/ec_boards_iface.h>
#include <iit/ecat/advr/esc.h>

using namespace iit::ecat::advr;

Rid2PosMap  rid2pos;


template <class C>
inline int ack_faults_X(C *c, int32_t faults)
{
    int32_t xor_faults = 0xFFFFFFFF;
    //xor_faults ^= xor_faults;
    return c->template set_SDO_byname("ack_board_fault_all", xor_faults);

}


template <class C>
inline int set_ctrl_status_X(C *c, int16_t cmd)
{
    int16_t ack;

    cmd = cmd & 0x00FF;
    c->template set_SDO_byname("ctrl_status_cmd", cmd);
    c->template get_SDO_byname("ctrl_status_cmd_ack", ack);

    // check 
    DPRINTF("set_ctrl_status ");
    return check_cmd_ack(cmd, ack);
}

template <class C>
inline int set_flash_cmd_X(C *c, uint16_t cmd)
{
    int16_t ack;

    cmd = cmd & 0x00FF;
    c->template set_SDO_byname("flash_params_cmd", cmd);
    c->template get_SDO_byname("flash_params_cmd_ack", ack);

    // check 
    DPRINTF("flash_params_cmd ");
    return check_cmd_ack(cmd, ack);

}

///////////////////////////////////////////////////////////////////////////////


Ec_Boards_ctrl::Ec_Boards_ctrl(const char * config_file) {

#ifdef __XENO__
    rd_mtx = PTHREAD_MUTEX_INITIALIZER;
    wr_mtx = PTHREAD_MUTEX_INITIALIZER;
#endif
    // read conf file .....
    eth_if = std::string(config_file);

    sync_cycle_time_ns = 1e6;     //   1ms
    //sync_cycle_time_ns = 10e6;      //  10ms
    //sync_cycle_time_ns = 0;         //   no dc 
    sync_cycle_offset_ns = 500e6;   // 500ms

}

Ec_Boards_ctrl::~Ec_Boards_ctrl() {

    // std::shared_ptr slaves 

#ifndef KEEP_POWER_ON
    iit::ecat::power_off();
#endif
    iit::ecat::finalize();

}

int Ec_Boards_ctrl::init(void) {

    if ( iit::ecat::initialize(eth_if.c_str()) <= 0 ) {
        return 0;
    }

    factory_board();

    return 1;

}

void Ec_Boards_ctrl::factory_board(void) {

    int i;
    int16_t rid;
    slave_cnt = ec_slavecount;

    for ( i=1; i<=slave_cnt; i++ ) {

        // XL Motor
        if ( ec_slave[i].eep_id == 0x10 ) {

            HpESC * mc_slave = new HpESC(ec_slave[i]);
            slaves[i] = iit::ecat::ESCPtr(mc_slave);
            rid = mc_slave->get_joint_robot_id();
            if (rid != -1) {
                rid2pos[rid] = i;
                //mcSlaves[rid] = mc_slave;
            }

        // MD Motor
        } else if ( ec_slave[i].eep_id == 0x11 ) {

            HpESC * mc_slave = new HpESC(ec_slave[i]);
            slaves[i] = iit::ecat::ESCPtr(mc_slave);
            rid = mc_slave->get_joint_robot_id();
            if (rid != -1) {
                rid2pos[rid] = i;
                //mcSlaves[rid] = mc_slave;
            }
        // LP Motor
        } else if ( ec_slave[i].eep_id == 0x12 ) {

            LpESC * mc_slave = new LpESC(ec_slave[i]);
            //HpESC * mc_slave = new HpESC(ec_slave[i]);
            slaves[i] = iit::ecat::ESCPtr(mc_slave);
            rid = mc_slave->get_joint_robot_id();
            if (rid != -1) {
                rid2pos[rid] = i;
                //mcSlaves[rid] = mc_slave;
            }
        // FT6 Sensor
        } else if ( ec_slave[i].eep_id == 0x20 ) {

            Ft6ESC * ft_slave = new Ft6ESC(ec_slave[i]);
            slaves[i] = iit::ecat::ESCPtr(ft_slave);
            //ftSlaves[i] = ft_slave;
#if 1
        } else if ( ec_slave[i].eep_id == 1234 ) {

            TestESC * test_slave = new TestESC(ec_slave[i]);
            slaves[i] = iit::ecat::ESCPtr(test_slave);
            test_slave->getSDO_ptr().par_1 = 123;
            test_slave->getSDO_ptr().par_2 = 999;
            test_slave->set_SDO_byname("par_1");
            test_slave->set_SDO_byname("par_2");
            test_slave->set_SDO_byname<int32_t>("par_2", 777);
            int32_t test;
            test_slave->get_SDO_byname<int32_t>("par_2", test);
            assert(test == 777);
#endif       
        } else if ( ec_slave[i].eep_id == 0x100 ) {

            // ECAT HUB 

        } else {

            DPRINTF("Warning %d not handled !!!\n", ec_slave[i].eep_id);
        }

    }

    iit::ecat::setExpectedSlaves(slaves);


}

int Ec_Boards_ctrl::configure_boards(void) {

    for ( auto it = slaves.begin(); it != slaves.end(); it++ ) {

        HpESC * hp = slave_as_HP(it->first);
        if ( hp ) { hp->print_info();; continue; }
    }
#if 0

    const objd_t * sdo = 0;

    for ( auto it = slaves.begin(); it != slaves.end(); it++ ) {
        DPRINTF("Get SDOs from %d\n", it->first);

        sdo = it->second.get()->get_SDO_objd();

        while ( sdo && sdo->index ) {

            //get_SDO(it->first, sdo);
            sdo ++;
        }
    }
    DPRINTF("End GET all SDOs\n");

    for ( auto it = slaves.begin(); it != slaves.end(); it++ ) {
        DPRINTF("Set SDOs from %d\n", it->first);
        if ( (mc = dynamic_cast<McESC*>(it->second.get())) ) {
            sdo = mc->get_SDOs();
        } else if ( dynamic_cast<TestESC*>(it->second.get()) ) {
            sdo = 0;
        }
        while ( sdo && sdo->index ) {
            int wkc = 0;
            if ( sdo->access == ATYPE_RW ) {
                wkc = set_SDO(it->first, sdo);
                if ( wkc <= 0 ) {
                    DPRINTF("Error SETTING SDO on board %d\n", it->first);
                }
            }
            sdo ++;
        }
    }
    DPRINTF("End SET all SDOs\n");
#endif

    return slaves.size();

}

/** 
 *  TODO: change to McESC objects !!!! 
 */
int Ec_Boards_ctrl::set_ctrl_status(uint16_t sPos, uint16_t cmd) {

    HpESC * hp = slave_as_HP(sPos);
    if (hp) { return set_ctrl_status_X(hp, cmd); }

    //MpESC * mp = slave_as_MP(sPos);
    //if (mp) { return set_ctrl_status_X(mp, cmd); }

    LpESC * lp = slave_as_LP(sPos);
    if (lp) { return set_ctrl_status_X(lp, cmd); }

    return -1;
 
#if 0
    const objd_t * sdo = 0;
    HpESC * hp;
    LpESC * lp;

    McEscVar var = mcSlaves[sPos];    //TODO has to be generic
    switch (var.which()) {
        case 0:
            hp = boost::get<HpESC*>(var);
            set_ctrl_status_X(hp, cmd);
            break;
        case 1:
            //lp = boost::get<LpESC*>(var);
            ///set_ctrl_status_X(lp, cmd);
            break;
    }

    return 0;
#endif
}

/** 
 *  TODO: change to McESC objects !!!! 
 */
int Ec_Boards_ctrl::set_flash_cmd(uint16_t sPos, uint16_t cmd) {

    HpESC * hp = slave_as_HP(sPos);
    if (hp) { return set_flash_cmd_X(hp, cmd); }

    LpESC * lp = slave_as_LP(sPos);
    if (lp) { return set_flash_cmd_X(lp, cmd); }

    Ft6ESC * ft = slave_as_FT(sPos);
    if (ft) { return set_flash_cmd_X(ft, cmd); }
    
    return 0; 

}

/** 
 *  TODO: change to McESC objects !!!! 
 */
int Ec_Boards_ctrl::ack_faults(uint16_t sPos, int32_t faults) {

    HpESC * hp = slave_as_HP(sPos);
    if (hp) { return ack_faults_X(hp, faults); }

    LpESC * lp = slave_as_LP(sPos);
    if (lp) { return ack_faults_X(lp, faults); }

    Ft6ESC * ft = slave_as_FT(sPos);
    if (ft) { return ack_faults_X(ft, faults); }
    
    return 0; 

}


int Ec_Boards_ctrl::set_cal_matrix(uint16_t sPos, std::vector<std::vector<float>> &cal_matrix) {

    Ft6ESC * ft = slave_as_FT(sPos);
    if (ft) { return ft->set_cal_matrix(cal_matrix); }

    return 0; 
}



/**
 *  TODO: change to McESC objects !!!!
 */
int Ec_Boards_ctrl::check_sanity(uint16_t sPos) {

    float iq_ref;
    HpESC * hp = slave_as_HP(sPos);
    if (hp) {
        hp->get_SDO_byname("iq_ref", iq_ref);
        DPRINTF("iq_ref %f\n", iq_ref);
    }

#if 0
    const objd_t * sdo = 0;
    HpESC * mc = 0;     //TODO has to be generic
    //McESC * mc = slave_as_MC(sPos);

    // V_batt_filt_100ms , Board_Temperature , T_mot1_filt_100ms
    std::vector<const objd_t*> offsets;// = {3,4,5};
    offsets.push_back(info_map[Board_type::HIGH_POWER]["V_batt_filt_100ms"].sdo_ptr); //TODO has to be generic
    offsets.push_back(info_map[Board_type::HIGH_POWER]["Board_Temperature"].sdo_ptr); //TODO has to be generic
    offsets.push_back(info_map[Board_type::HIGH_POWER]["T_mot1_filt_100ms"].sdo_ptr); //TODO has to be generic

    for ( auto sl_it = mcSlaves.begin(); sl_it != mcSlaves.end(); sl_it++ ) {
        mc = (HPESC*)sl_it->second; //TODO has to be generic
        for ( auto it = offsets.begin(); it != offsets.end(); it++ ) {
            if ( *it ) {
                // get value from slave
                if ( get_SDO(sl_it->first, *it) <= 0 ) {
                    ; // error
                }
            }
        }
    }
    //TODO return something!!
#endif
    return 0;
}


int Ec_Boards_ctrl::set_operative() {

    expected_wkc = iit::ecat::operational(&sync_cycle_time_ns, &sync_cycle_offset_ns);
    return expected_wkc;
}

void Ec_Boards_ctrl::getRxPDO(int slave_index, McEscPdoTypes::pdo_rx &pdo)
{
    rd_LOCK();
    memcpy((void*)&pdo, (void*)&McRxPDO_map[slave_index], sizeof(pdo));
    rd_UNLOCK();
}
void Ec_Boards_ctrl::getRxPDO(int slave_index, Ft6EscPdoTypes::pdo_rx &pdo)
{
    rd_LOCK();
    memcpy((void*)&pdo, (void*)&FtRxPDO_map[slave_index], sizeof(pdo));
    rd_UNLOCK();
}

void Ec_Boards_ctrl::setTxPDO(int slave_index, McEscPdoTypes::pdo_tx pdo)
{
    wr_LOCK();
    McTxPDO_map[slave_index] = pdo;
    wr_UNLOCK();
}
void Ec_Boards_ctrl::setTxPDO(int slave_index, Ft6EscPdoTypes::pdo_tx pdo)
{
    wr_LOCK();
    FtTxPDO_map[slave_index] = pdo;
    wr_UNLOCK();
}
void Ec_Boards_ctrl::getTxPDO(int slave_index, McEscPdoTypes::pdo_tx &pdo)
{
    wr_LOCK();
    memcpy((void*)&pdo, (void*)&McTxPDO_map[slave_index], sizeof(pdo));
    wr_UNLOCK();
}
void Ec_Boards_ctrl::getTxPDO(int slave_index, Ft6EscPdoTypes::pdo_tx &pdo)
{
    wr_LOCK();
    memcpy((void*)&pdo, (void*)&FtTxPDO_map[slave_index], sizeof(pdo));
    wr_UNLOCK();
}


int Ec_Boards_ctrl::recv_from_slaves() {

    TestESC * test;
    /////////////////////////////////////////////
    // wait for cond_signal 
    // ecat_thread sync with DC
    int ret = iit::ecat::recv_from_slaves(&timing);
    if ( ret != 0 ) {
        DPRINTF("fail recv_from_slaves\n");
        return 0;
    }

    rd_LOCK();
    for ( auto it = slaves.begin(); it != slaves.end(); it++ ) {

        HpESC * hp = slave_as_HP(it->first);
        if ( hp ) { McRxPDO_map[it->first] = hp->getRxPDO(); continue; }

        //MpESC * mp = slave_as_MP(it->first);
        //if ( mp ) { McRxPDO_map[it->first] = mp->getRxPDO(); continue; }

        LpESC * lp = slave_as_LP(it->first);
        if ( lp ) { McRxPDO_map[it->first] = lp->getRxPDO(); continue; }

        Ft6ESC * ft = slave_as_FT(it->first);
        if ( ft ) { FtRxPDO_map[it->first] = ft->getRxPDO(); continue; }

        //if ( (test = dynamic_cast<TestESC*>(it->second.get())) ) {
        //    test->push_back(test->getRxPDO());
        //}
    }
    rd_UNLOCK();

    return 1;
}

int Ec_Boards_ctrl::send_to_slaves() {

    int wkc, retry = 2;

    { // begin std::mutex scope   
        wr_LOCK();

        for ( auto it = slaves.begin(); it != slaves.end(); it++ ) {

            HpESC * hp = slave_as_HP(it->first);
            if ( hp ) { hp->setTxPDO(McTxPDO_map[it->first]); continue; }

            //MpESC * mp = slave_as_MP(it->first);
            //if ( mp ) { hp->setTxPDO(McTxPDO_map[it->first]); continue; }

            LpESC * lp = slave_as_LP(it->first);
            if ( lp ) { lp->setTxPDO(McTxPDO_map[it->first]); continue; }

            Ft6ESC * ft = slave_as_FT(it->first);
            if ( ft ) { ft->setTxPDO(FtTxPDO_map[it->first]); continue;}
        }
        wr_UNLOCK();
    } // end mutex scope

    wkc = iit::ecat::send_to_slaves();

    while ( wkc < expected_wkc && retry-- ) {
        DPRINTF("## wkc %d\n", wkc);
        wkc = iit::ecat::send_to_slaves();
    }

    return retry;
}


static int esc_gpio_ll_wr(uint16_t configadr, uint16_t gpio_val) {

    int wc = ec_FPWR(configadr, 0x0F10, sizeof(gpio_val), &gpio_val, EC_TIMEOUTRET3);
    if ( wc <= 0 ) {
        DPRINTF("ERROR FPWR(%x, 0x0F10, %d)\n", configadr, gpio_val);
    }
    return wc;
}

int Ec_Boards_ctrl::update_board_firmware(uint16_t slave_pos, std::string firmware, uint32_t passwd_firm) {

    int wc, ret = 0;
    bool go_ahead = true;
    uint16_t configadr = slaves[slave_pos]->get_configadr();

    // all slaves in INIT state 
    req_state_check(0, EC_STATE_INIT);

    // check slave type ... HiPwr uses ET1100 GPIO to force/release bootloader 
    HpESC * s = dynamic_cast<HpESC*>(slaves[slave_pos].get());
    if ( s ) {
        // pre-update
        // POWER OFF
        if ( esc_gpio_ll_wr(configadr, 0x0) <= 0 ) {
            return 0;
        }
        sleep(1);
        // POWER ON and BOOT
        if ( esc_gpio_ll_wr(configadr, 0x5) <= 0 ) {
            return 0;
        }
        sleep(1);

    } else {
        DPRINTF("Slave %d is NOT a HpESC\n", slave_pos);
    }

    // first boot state request is handled by application that jump to bootloader
    // we do NOT have a state change in the slave
    req_state_check(slave_pos, EC_STATE_BOOT);

    // second boot state request is handled by bootloader
    // now the slave should go in BOOT state
    if ( ! req_state_check(slave_pos, EC_STATE_BOOT) ) {
        DPRINTF("Slave %d not changed to BOOT state.\n", slave_pos);
        return 0;
    }

    if ( s ) {
        // erase flash
        uint16_t flash_cmd = 0x00EE;
        uint16_t flash_cmd_ack;
        int size;
        // write sdo flash_cmd
        DPRINTF("erasing flash ...\n");
        wc = ec_SDOwrite(slave_pos, 0x8000, 0x1, false, sizeof(flash_cmd), &flash_cmd, EC_TIMEOUTRXM * 20); // 14 secs
        if ( wc <= 0 ) {
            DPRINTF("ERROR writing flash_cmd\n");
            go_ahead = false;
        } else {
            // read flash_cmd_ack
            wc = ec_SDOread(slave_pos, 0x8000, 0x2, false, &size, &flash_cmd_ack, EC_TIMEOUTRXM *3);
            DPRINTF("Slave %d wc %d flash_cmd_ack 0x%04X\n", slave_pos, wc, flash_cmd_ack);
            if ( wc <= 0 ) {
                DPRINTF("ERROR reading flash_cmd_ack\n");
                go_ahead = false;
            } else if ( flash_cmd_ack != CTRL_CMD_DONE ) {
                DPRINTF("ERROR erasing flash\n");
                go_ahead = false;
            }
        }

    }

    if ( go_ahead ) {
        ret = send_file(slave_pos, firmware, passwd_firm);
    }

    if ( s ) {
        // post-update ... restore
        // POWER ON and RESET
        //if ( esc_gpio_ll_wr(configadr, 0x3) <= 0) { return 0; }
        //sleep(1);
        // power ON
        if ( esc_gpio_ll_wr(configadr, 0x1) <= 0 ) {
            return 0;
        }
    }

    //INIT state request is handled by bootloader that jump to application that start from INIT
    req_state_check(slave_pos, EC_STATE_INIT);

    return go_ahead && (ret > 0) ;
}

