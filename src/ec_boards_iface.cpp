#include <iit/ecat/advr/ec_boards_iface.h>

#include <iostream>
#include <fstream>

using namespace iit::ecat::advr;


///////////////////////////////////////////////////////////////////////////////


Ec_Boards_ctrl::Ec_Boards_ctrl(std::string config_file) {

#ifdef __XENO__
    rd_mtx = PTHREAD_MUTEX_INITIALIZER;
    wr_mtx = PTHREAD_MUTEX_INITIALIZER;
#endif
    
    std::ifstream fin(config_file);
    if ( fin.fail() ) {
        DPRINTF("Can not open %s\n", config_file.c_str());
        assert(0);
    }

    root_cfg = YAML::LoadFile(config_file);

    const YAML::Node& board_ctrl = root_cfg["ec_board_ctrl"];

    eth_if = board_ctrl["eth_iface"].as<std::string>();
    sync_cycle_time_ns = board_ctrl["sync_cycle_time_ns"].as<uint64>();
    sync_cycle_offset_ns = board_ctrl["sync_cycle_offset_ns"].as<uint64>();

}

Ec_Boards_ctrl::~Ec_Boards_ctrl() {

    // std::shared_ptr slaves 
    if ( root_cfg["ec_board_ctrl"]["power_off_boards"].as<bool>() == true ) {
        iit::ecat::power_off();
    }
    iit::ecat::finalize();

}

int Ec_Boards_ctrl::init(void) {
        
    if ( iit::ecat::initialize(eth_if.c_str()) > 0 ) {
        factory_board();
        return EC_BOARD_OK;
    }

    return EC_BOARD_NOK;

}

void Ec_Boards_ctrl::factory_board(void) {

    int i;
    int16_t rid;
    slave_cnt = ec_slavecount;


    for ( i=1; i<=slave_cnt; i++ ) {

        ///////////////////////////////////////////////////
        // BigMotor and MediumMotor
        if ( ec_slave[i].eep_id == HI_PWR_AC_MC ||
             ec_slave[i].eep_id == HI_PWR_DC_MC) {

            HpESC * mc_slave = new HpESC(ec_slave[i]);
            if ( mc_slave->init(root_cfg) != EC_BOARD_OK ) {
                // skip this slave
                zombies[i] = iit::ecat::ESCPtr(mc_slave);
                continue;
            }
            slaves[i] = iit::ecat::ESCPtr(mc_slave);
            rid = mc_slave->get_robot_id();
            mc_slave->print_info();
            if (rid != -1) {
                rid2pos[rid] = i;
            }
        }
        ///////////////////////////////////////////////////
        // LP Motor
        else if ( ec_slave[i].eep_id == LO_PWR_DC_MC ) {

            LpESC * mc_slave = new LpESC(ec_slave[i]);
            if ( mc_slave->init(root_cfg) != EC_BOARD_OK ) {
                // skip this slave
                zombies[i] = iit::ecat::ESCPtr(mc_slave);
                continue;
            }
            slaves[i] = iit::ecat::ESCPtr(mc_slave);
            rid = mc_slave->get_robot_id();
            mc_slave->print_info();
            if (rid != -1) {
                rid2pos[rid] = i;
            }
        }
        ///////////////////////////////////////////////////
        // FT6 Sensor
        else if ( ec_slave[i].eep_id == FT6 ) {

            Ft6ESC * ft_slave = new Ft6ESC(ec_slave[i]);
            if ( ft_slave->init(root_cfg) != EC_BOARD_OK ) {
                // skip this slave
                zombies[i] = iit::ecat::ESCPtr(ft_slave);
                continue;
            }
            slaves[i] = iit::ecat::ESCPtr(ft_slave);
            rid = ft_slave->get_robot_id();
            ft_slave->print_info();
            if (rid != -1) {
                rid2pos[rid] = i;
            }
        }
        ///////////////////////////////////////////////////
        // Pow board    
        else if ( ec_slave[i].eep_id == POW_BOARD ) {

            PowESC * pow = new PowESC(ec_slave[i]);
            if ( pow->init(root_cfg) != EC_BOARD_OK ) {
                // skip this slave
                zombies[i] = iit::ecat::ESCPtr(pow);
                continue;
            }
            slaves[i] = iit::ecat::ESCPtr(pow);            
        }
        ///////////////////////////////////////////////////
        // Hubs
        else if ( ec_slave[i].eep_id == HUB ) {

            HubESC * hub = new HubESC(ec_slave[i]);
            slaves[i] = iit::ecat::ESCPtr(hub); 

        } else if ( ec_slave[i].eep_id == HUB_IO ) {

            HubIoESC * hub = new HubIoESC(ec_slave[i]);
            slaves[i] = iit::ecat::ESCPtr(hub); 
        }
        ///////////////////////////////////////////////////
        // Test    
        else if ( ec_slave[i].eep_id == EC_TEST ) {

            TestESC * test_slave = new TestESC(ec_slave[i]);
            if ( test_slave->init(root_cfg) != EC_BOARD_OK ) {
                // skip this slave
                zombies[i] = iit::ecat::ESCPtr(test_slave);
                continue;
            }
            slaves[i] = iit::ecat::ESCPtr(test_slave);            
        }
        ///////////////////////////////////////////////////
        else {

            DPRINTF("Warning product code %d not handled !!!\n", ec_slave[i].eep_id);
        }

    }

    iit::ecat::setExpectedSlaves(slaves);

    if ( zombies.size() > 0 ) {
        DPRINTF("Warning you got %lud zombies !!!\n", zombies.size());
        for (auto it = zombies.begin(); it != zombies.end(); it++) {
            DPRINTF("\tpos %d ", it->first);
        }
        DPRINTF("\n");
    }
}

   
int Ec_Boards_ctrl::configure_boards(void) {

    return slaves.size();

}


void Ec_Boards_ctrl::start_motors(int control_type)
{
    for (auto it = rid2pos.begin(); it != rid2pos.end(); it++ ) {
        Motor * moto = slave_as_Motor(it->second);
        if (moto) {
            moto->start(control_type);
        }
    }
}

void Ec_Boards_ctrl::stop_motors(void)
{
    for (auto it = rid2pos.begin(); it != rid2pos.end(); it++ ) {
    
        Motor * moto = slave_as_Motor(it->second);
        if (moto) {
            moto->stop();
        }
    }
}

/** 
 *  TODO: change to McESC objects !!!! 
 */
int Ec_Boards_ctrl::set_ctrl_status(uint16_t sPos, int16_t cmd) {

    HpESC * hp = slave_as_HP(sPos);
    if (hp) { return set_ctrl_status_X(hp, cmd); }

    LpESC * lp = slave_as_LP(sPos);
    if (lp) { return set_ctrl_status_X(lp, cmd); }

    return EC_WRP_NOK;
}

/** 
 *  TODO: change to McESC objects !!!! 
 */
int Ec_Boards_ctrl::set_flash_cmd(uint16_t sPos, int16_t cmd) {

    HpESC * hp = slave_as_HP(sPos);
    if (hp) { return set_flash_cmd_X(hp, cmd); }

    LpESC * lp = slave_as_LP(sPos);
    if (lp) { return set_flash_cmd_X(lp, cmd); }
    
    Ft6ESC * ft = slave_as_FT(sPos);
    if ( ft ) { return set_flash_cmd_X(ft, cmd); }
    
    return EC_WRP_NOK;
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
        hp->readSDO_byname("iq_ref", iq_ref);
        DPRINTF("iq_ref %f\n", iq_ref);
    }

    return 0;
}

void Ec_Boards_ctrl::check_DataLayer(void)
{
    for (auto it = slaves.begin(); it != slaves.end(); it++) {
            it->second.get()->readErrReg();
        }
}
    

int Ec_Boards_ctrl::set_operative() {

    expected_wkc = iit::ecat::operational(&sync_cycle_time_ns, &sync_cycle_offset_ns);
    return expected_wkc;
}

int Ec_Boards_ctrl::getRxPDO(int slave_index, McEscPdoTypes::pdo_rx &pdo)
{
    Motor * m = slave_as_Motor(slave_index);
    if ( !m ) { return EC_BOARD_NOK; }
    rd_LOCK();
    pdo = m->getRxPDO();
    rd_UNLOCK();
    return EC_BOARD_OK;
}
int Ec_Boards_ctrl::setTxPDO(int slave_index, McEscPdoTypes::pdo_tx pdo)
{
    Motor * m = slave_as_Motor(slave_index);
    if ( !m ) { return EC_BOARD_NOK; }
    wr_LOCK();
    m->setTxPDO(pdo);
    wr_UNLOCK();
    return EC_BOARD_OK;
}
int Ec_Boards_ctrl::getTxPDO(int slave_index, McEscPdoTypes::pdo_tx &pdo)
{
    Motor * m = slave_as_Motor(slave_index);
    if ( !m ) { return EC_BOARD_NOK; }
    wr_LOCK();
    pdo = m->getTxPDO();
    wr_UNLOCK();
    return EC_BOARD_OK;
}


int Ec_Boards_ctrl::getRxPDO(int slave_index, Ft6EscPdoTypes::pdo_rx &pdo)
{
    Ft6ESC * ft = slave_as_FT(slave_index);
    if ( !ft ) { return EC_BOARD_NOK; }
    rd_LOCK();
    pdo = ft->getRxPDO();
    rd_UNLOCK();
    return EC_BOARD_OK;
}
int Ec_Boards_ctrl::setTxPDO(int slave_index, Ft6EscPdoTypes::pdo_tx pdo)
{
    Ft6ESC * ft = slave_as_FT(slave_index);
    if ( !ft ) { return EC_BOARD_NOK; }
    wr_LOCK();
    ft->setTxPDO(pdo);
    wr_UNLOCK();
    return EC_BOARD_OK;
}
int Ec_Boards_ctrl::getTxPDO(int slave_index, Ft6EscPdoTypes::pdo_tx &pdo)
{
    Ft6ESC * ft = slave_as_FT(slave_index);
    if ( !ft ) { return EC_BOARD_NOK; }
    wr_LOCK();
    pdo = ft->getTxPDO();
    wr_UNLOCK();
    return EC_BOARD_OK;
}


int Ec_Boards_ctrl::recv_from_slaves() {

    /////////////////////////////////////////////
    // wait for cond_signal 
    // ecat_thread sync with DC
    rd_LOCK();
    int ret = iit::ecat::recv_from_slaves(&timing);
    rd_UNLOCK();
    if ( ret < 0 ) {
        DPRINTF("fail recv_from_slaves\n");
        return EC_BOARD_RECV_FAIL;
    }
    
    return EC_BOARD_OK;
}

int Ec_Boards_ctrl::send_to_slaves() {

    int wkc, retry = 3;

    wr_LOCK();
    wkc = iit::ecat::send_to_slaves();
    wr_UNLOCK();
    while ( wkc <= 0  && retry ) {
        DPRINTF("iit::ecat::send_to_slaves wkc %d retry %d\n", wkc, retry);
        wr_LOCK();
        wkc = iit::ecat::send_to_slaves();
        wr_UNLOCK();
        retry--;
    }

    return wkc;
}


static int esc_gpio_ll_wr(uint16_t configadr, uint16_t gpio_val) {

    int wc = ec_FPWR(configadr, 0x0F10, sizeof(gpio_val), &gpio_val, EC_TIMEOUTRET3);
    if ( wc <= 0 ) {
        DPRINTF("ERROR FPWR(%x, 0x0F10, %d)\n", configadr, gpio_val);
    }
    return wc;
}

/**
 * bit0 power_on
 * bit1 reset
 * bit2 boot
 */
int Ec_Boards_ctrl::update_board_firmware(uint16_t slave_pos, std::string firmware, uint32_t passwd_firm) {

    int wc, ret = 0;
    char * ec_err_string;
    bool go_ahead = true;
    uint16_t configadr; 
    uint16_t flash_cmd;
    uint16_t flash_cmd_ack = 0x0;
    int size;
    int tries;
    char firm_ver[16];

    // all slaves in INIT state 
    req_state_check(0, EC_STATE_INIT);

    EscWrapper * s = slave_as_EscWrapper(slave_pos);
    if ( ! s ) {
        s = slave_as_Zombie(slave_pos);
        if ( ! s ) {
            return 0;
        }
    }

    configadr = s->get_configadr();

    // check slave type ... HiPwr uses ET1100 GPIO to force/release bootloader 
    if ( s ) {
        if ( (s->get_ESC_type() == POW_BOARD) ||
             (s->get_ESC_type() == HI_PWR_AC_MC) || 
             (s->get_ESC_type() == HI_PWR_DC_MC)) {
            // pre-update
            // POW_OFF+RESET 0x2
            if ( esc_gpio_ll_wr(configadr, 0x2) <= 0 ) {
                return 0;
            }
            sleep(1);
            // todo POW_ON+BOOT 0x5
            if ( esc_gpio_ll_wr(configadr, 0x5) <= 0 ) {
                return 0;
            }
            sleep(3);
    
        } else {
            DPRINTF("Slave %d is NOT a XL or a MD motor\n", slave_pos);
        }
    }

    // first boot state request is handled by application that jump to bootloader
    // we do NOT have a state change in the slave
    req_state_check(slave_pos, EC_STATE_BOOT);

    sleep(3);

    // second boot state request is handled by bootloader
    // now the slave should go in BOOT state
    if ( req_state_check(slave_pos, EC_STATE_BOOT) != EC_STATE_BOOT ) {
        DPRINTF("Slave %d not changed to BOOT state.\n", slave_pos);
        return 0;
    }
                   
    
    if ( s) {
        if ( (s->get_ESC_type() == POW_BOARD) ||
             (s->get_ESC_type() == HI_PWR_AC_MC) || 
             (s->get_ESC_type() == HI_PWR_DC_MC)) {
            // erase flash
            flash_cmd = 0x00EE;
            flash_cmd_ack = 0x0;
            tries = 30;

            memset((void*)firm_ver, sizeof(firm_ver), 0);
            wc = ec_SDOread(slave_pos, 0x8000, 0x4, false, &size, &firm_ver, EC_TIMEOUTRXM * 30);
            DPRINTF("Slave %d bl fw %s\n", slave_pos, firm_ver);

        
            // write sdo flash_cmd
            DPRINTF("erasing flash ...\n");
            wc = ec_SDOwrite(slave_pos, 0x8000, 0x1, false, sizeof(flash_cmd), &flash_cmd, EC_TIMEOUTRXM * 30); // 21 secs
            if ( wc <= 0 ) {
                DPRINTF("ERROR writing flash_cmd\n");
                ec_err_string =  ec_elist2string();
                DPRINTF("Ec_error : %s\n", ec_err_string);
                go_ahead = false;
            } else {
                
                while ( tries -- ) {
                    sleep(1);
                    // read flash_cmd_ack
                    wc = ec_SDOread(slave_pos, 0x8000, 0x2, false, &size, &flash_cmd_ack, EC_TIMEOUTRXM * 30);
                    DPRINTF("Slave %d wc %d flash_cmd_ack 0x%04X\n", slave_pos, wc, flash_cmd_ack);
                    if ( wc <= 0 ) {
                        DPRINTF("ERROR reading flash_cmd_ack\n");
                        ec_err_string =  ec_elist2string();
                        DPRINTF("Ec_error : %s\n", ec_err_string);
                        go_ahead = false;
                    } else if ( flash_cmd_ack != CTRL_CMD_DONE ) {
                        DPRINTF("ERROR erasing flash\n");
                        go_ahead = false;
                    } else {
                        // 
                        break;
                    }
                }
            }
        }
    }

    if ( go_ahead ) {
        ret = send_file(slave_pos, firmware, passwd_firm);
    }

    if ( s ) {
        // post-update ... restore
        // POW_OFF+RESET 0x2
        if ( esc_gpio_ll_wr(configadr, 0x2) <= 0 ) {
            return 0;
        }
        sleep(3);
        // POW_ON 0x1
        if ( esc_gpio_ll_wr(configadr, 0x1) <= 0 ) {
            return 0;
        }
    }

    //INIT state request is handled by bootloader that jump to application that start from INIT
    req_state_check(slave_pos, EC_STATE_INIT);

    return go_ahead && (ret > 0) ;
}

