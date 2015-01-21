#include <iit/ecat/advr/ec_boards_iface.h>
#include <iit/ecat/advr/esc.h>

using namespace iit::ecat::advr;

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

    set_info_tables();
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
    slave_cnt = ec_slavecount;

    for ( i=1; i<=slave_cnt; i++ ) {

        if ( ec_slave[i].eep_id == 0x10 ) {

            HPESC * mc_slave = new HPESC(ec_slave[i]);
            slaves[i] = iit::ecat::ESCPtr(mc_slave);
            mcSlaves[i] = mc_slave;

        } else if ( ec_slave[i].eep_id == 0x12 ) {

            //LPESC * mc_slave = new LPESC(ec_slave[i]);
            HPESC * mc_slave = new HPESC(ec_slave[i]);
            slaves[i] = iit::ecat::ESCPtr(mc_slave);
            mcSlaves[i] = mc_slave;

        } else if ( ec_slave[i].eep_id == 0x20 ) {

            FtESC * ft_slave = new FtESC(ec_slave[i]);
            slaves[i] = iit::ecat::ESCPtr(ft_slave);
            ftSlaves[i] = ft_slave;

        } else if ( ec_slave[i].eep_id == 1234 ) {

            TestESC * test_slave = new TestESC(ec_slave[i]);
            slaves[i] = iit::ecat::ESCPtr(test_slave);
            test_slave->getFlashParam().par_1 = 123;
            test_slave->getFlashParam().par_2 = 999;
            test_slave->set_SDO_byname("par_1");
            test_slave->set_SDO_byname("par_2");

        } else if ( ec_slave[i].eep_id == 0x100 ) {

            // ECAT HUB 

        } else {

            DPRINTF("Warning %d not handled !!!\n", ec_slave[i].eep_id);
        }

    }

    iit::ecat::setExpectedSlaves(slaves);


}

int Ec_Boards_ctrl::set_SDO(int slave_pos, int index, int subindex, int size, void *data) {

    return ec_SDOwrite(slave_pos, index, subindex, false, size, data, EC_TIMEOUTRXM);
}

int Ec_Boards_ctrl::set_SDO(int slave_pos, const objd_t *sdo) {

    //DPRINTF("set_SDO %s [%d.0x%04X.0x%02X]\n", sdo->name, slave_pos, sdo->index, sdo->subindex);
    char * err;
    ec_errort   ec_error;
    int final_size = sdo->bitlength/8;
    int wkc = set_SDO(slave_pos, sdo->index, sdo->subindex, final_size, sdo->data);
    if ( wkc <= 0 || final_size!=sdo->bitlength/8 ) {
        DPRINTF("Slave %d >> ", slave_pos);
        if ( wkc <= 0 ) {
            err =  ec_elist2string();
            DPRINTF("Ec_error : %s\n", err);
        } else {
            DPRINTF("SDO write fail : %d != %d\n", final_size, sdo->bitlength/8);
        }
    } else {
        // OK ....
    }

    return wkc; 
}

int Ec_Boards_ctrl::get_SDO(int slave_pos, int index, int subindex, int *size, void *data) {

    return ec_SDOread(slave_pos, index, subindex, FALSE, size, data, EC_TIMEOUTRXM);
}

int Ec_Boards_ctrl::get_SDO(int slave_pos, const objd_t *sdo) {

    //DPRINTF("get_SDO %s [%d.0x%04X.0x%02X]\n", sdo->name, slave_pos, sdo->index, sdo->subindex);

    char * err;
    ec_errort ec_error;
    int final_size = sdo->bitlength/8;
    int wkc = get_SDO(slave_pos, sdo->index, sdo->subindex, &final_size, sdo->data);
    if ( wkc <= 0 || final_size!=sdo->bitlength/8 ) {
        DPRINTF("Slave %d >> ", slave_pos);
        if ( wkc <= 0 ) {
            err =  ec_elist2string();
            DPRINTF("Ec_error : %s\n", err);
        } else {
            DPRINTF("SDO read fail : %d != %d\n", final_size, sdo->bitlength/8);
        }
    } else {
        // OK ....
    }

    return wkc; 
}

int Ec_Boards_ctrl::configure_boards(void) {

    McESC  * mc;
    const objd_t * sdo = 0;

    for ( auto it = slaves.begin(); it != slaves.end(); it++ ) {
        DPRINTF("Get SDOs from %d\n", it->first);
        if ( (mc = dynamic_cast<McESC*>(it->second.get())) ) {
            sdo = mc->get_SDOs();
        } else if ( dynamic_cast<TestESC*>(it->second.get()) ) {
            sdo = 0;

        }
        while ( sdo && sdo->index ) {

            get_SDO(it->first, sdo);
            sdo ++;
        }
    }
    DPRINTF("End GET all SDOs\n");

    /**
     * set params ....
     */
    for ( auto it = mcSlaves.begin(); it != mcSlaves.end(); it++ ) {

        mc = it->second;
        //mc->param. 
    }
    /**
     * 
     */
#if 0
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
#else
    /*
    mcSlaves[2]->param.ctrl_status_cmd = 2;
    sdo = mcSlaves[2]->SDOs;
    while ( sdo && strcmp(sdo->name,"pos_ref") ) { sdo ++; }
    DPRINTF("SDOs idx %d %s\n", sdo - s->SDOs, sdo->name);
    while ( sdo && strcmp(sdo->name,"Sensor_type") ) { sdo ++; }
    DPRINTF("SDOs idx %d %s\n", sdo - s->SDOs, sdo->name);
    while ( sdo && strcmp(sdo->name,"firmware_version") ) { sdo ++; }
    DPRINTF("SDOs idx %d %s\n", sdo - s->SDOs, sdo->name);
    while ( sdo && strcmp(sdo->name,"ctrl_status_cmd") ) { sdo ++; }
    for (auto it = mcSlaves.begin(); it != mcSlaves.end(); it++) {

        mc = it->second;
        mc->param.ctrl_status_cmd = 2;
        sdo = mc->SDOs8001 + 6;
        if (sdo) {
            DPRINTF("SDOs idx %d %s\n", sdo - mc->SDOs, sdo->name);
            set_SDO(it->first, sdo);
        }
    } 
    */

#endif
    return slaves.size();

}

/** 
 *  TODO: change to McESC objects !!!! 
 */
int Ec_Boards_ctrl::set_ctrl_status(uint16_t sPos, uint16_t cmd) {

    const objd_t * sdo = 0;
    HPESC * mc = (HPESC*)mcSlaves[sPos];    //TODO has to be generic
    //McESC * mc = slave_as_MC(sPos);

    if ( !mc ) {
        return 0;
    }
    // set ctrl_status_cmd value
    mc->param.ctrl_status_cmd = cmd & 0x00FF;
    // get ctrl_status_cmd sdo pointer
    sdo = mc->get_SDOs8001() + 6;
    if ( sdo ) {
        DPRINTF("SDOs idx %ld %s\n", sdo - mc->get_SDOs(), sdo->name);
        // set value to slave
        set_SDO(sPos, sdo);
    }
    // get ctrl_status_cmd_Ack sdo pointer
    sdo = mc->get_SDOs8001() + 7;
    if ( sdo ) {
        DPRINTF("SDOs idx %ld %s\n", sdo - mc->get_SDOs(), sdo->name);
        // get value from slave
        get_SDO(sPos, sdo);
    }

    // check 
    DPRINTF("set_ctrl_status ");
    if ( mc->param.ctrl_status_cmd_ack == ((cmd & 0x00FF) | CTRL_CMD_DONE) ) {
        DPRINTF("DONE 0x%04X\n", cmd);
    } else if ( mc->param.ctrl_status_cmd_ack == ((cmd & 0x00FF) | CTRL_CMD_ERROR) ) {
        DPRINTF("FAIL 0x%04X\n", cmd);
    } else {
        DPRINTF("PROTOCOL FAILURE !!!\n");
    }
    //TODO return something!!
}

/** 
 *  TODO: change to McESC objects !!!! 
 */
int Ec_Boards_ctrl::set_flash_cmd(uint16_t sPos, uint16_t cmd) {

    const objd_t * sdo = 0;
    //HPESC * mc = (HPESC*)mcSlaves[sPos];	//TODO has to be generic
    //McESC * mc = slave_as_MC(sPos);
    FtESC * ft = slave_as_FT(sPos);

    if ( !ft ) {
        return 0;
    }
    // set param value
    ft->param.flash_params_cmd = cmd & 0x00FF;
    // get parameter sdo pointer
    sdo = info_map[Board_type::FT6]["flash_parameters_command"].sdo_ptr;
    if ( sdo ) {
        DPRINTF("SDOs idx %ld %s\n", sdo - ft->get_SDOs(), sdo->name);
        // set value to slave
        set_SDO(sPos, sdo);
    }
    // get parameter sdo pointer
    sdo = info_map[Board_type::FT6]["flash_parameters_command_ack"].sdo_ptr;
    if ( sdo ) {
        DPRINTF("SDOs idx %ld %s\n", sdo - ft->get_SDOs(), sdo->name);
        // get value from slave
        get_SDO(sPos, sdo);
    }

    // check 
    DPRINTF("set_ctrl_status ");
    if ( ft->param.flash_params_cmd_ack == ((cmd & 0x00FF) | CTRL_CMD_DONE) ) {
        DPRINTF("DONE\n");
    } else if ( ft->param.flash_params_cmd_ack == ((cmd & 0x00FF) | CTRL_CMD_ERROR) ) {
        DPRINTF("FAIL\n");
    } else {
        DPRINTF("PROTOCOL FAILURE !!!\n");
    }
    //TODO return something!!
}

int Ec_Boards_ctrl::set_cal_matrix(uint16_t sPos, std::vector<std::vector<float>> &cal_matrix) {

    const objd_t * sdo = 0;
    FtESC * ft = slave_as_FT(sPos);

    if ( !ft ) {
        return 0;
    }

    uint32_t flash_row_cmd = 0x00C7;

    for (int r=0; r<6; r++ ) {

        // set row n param value
        ft->param.matrix_rn_c1 = cal_matrix[r][0];
        ft->param.matrix_rn_c2 = cal_matrix[r][1];
        ft->param.matrix_rn_c3 = cal_matrix[r][2];
        ft->param.matrix_rn_c4 = cal_matrix[r][3];
        ft->param.matrix_rn_c5 = cal_matrix[r][4];
        ft->param.matrix_rn_c6 = cal_matrix[r][5];

        // get first parameter sdo pointer
        sdo = info_map[Board_type::FT6]["Matrix c1"].sdo_ptr;

        for ( int i=0; i<6; i++) {
            if ( sdo ) {
                DPRINTF("SDOs idx %ld %s\n", sdo - ft->get_SDOs(), sdo->name);
                // set value to slave
                set_SDO(sPos, sdo);
            }
            sdo ++;
        }

        // set cmd param value
        ft->param.flash_params_cmd = flash_row_cmd & 0x00FF;
        // get parameter sdo pointer
        sdo = info_map[Board_type::FT6]["flash_parameters_command"].sdo_ptr;
        if ( sdo ) {
            DPRINTF("SDOs idx %ld %s\n", sdo - ft->get_SDOs(), sdo->name);
            // set value to slave
            set_SDO(sPos, sdo);
        }

        // get parameter sdo pointer
        sdo = info_map[Board_type::FT6]["flash_parameters_command_ack"].sdo_ptr;
        if ( sdo ) {
            DPRINTF("SDOs idx %ld %s\n", sdo - ft->get_SDOs(), sdo->name);
            // get value from slave
            get_SDO(sPos, sdo);
        }

        // check 
        DPRINTF("flash_parameters_command ");
        if ( ft->param.flash_params_cmd_ack == ((flash_row_cmd & 0x00FF) | CTRL_CMD_DONE) ) {
            DPRINTF("DONE\n");
        } else if ( ft->param.flash_params_cmd_ack == ((flash_row_cmd & 0x00FF) | CTRL_CMD_ERROR) ) {
            DPRINTF("FAIL\n");
        } else {
            DPRINTF("PROTOCOL FAILURE !!!\n");
        }

        // next row cmd
        flash_row_cmd++;

    } // for rows

    //TODO return something!!
}


/**
 *  TODO: change to McESC objects !!!!
 */
int Ec_Boards_ctrl::check_sanity(void) {

    const objd_t * sdo = 0;
    HPESC * mc = 0;     //TODO has to be generic
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
}


int Ec_Boards_ctrl::set_operative() {

    expected_wkc = iit::ecat::operational(&sync_cycle_time_ns, &sync_cycle_offset_ns);
    return expected_wkc;
}

void Ec_Boards_ctrl::getRxPDO(int slave_index, McESCTypes::pdo_rx &pdo)
{
    rd_LOCK();
    memcpy((void*)&pdo, (void*)&McRxPDO_map[slave_index], sizeof(pdo));
    //pdo = FtRxPDO_map[slave_index];
    rd_UNLOCK();
}
void Ec_Boards_ctrl::getRxPDO(int slave_index, FtESCTypes::pdo_rx &pdo)
{
    rd_LOCK();
    memcpy((void*)&pdo, (void*)&FtRxPDO_map[slave_index], sizeof(pdo));
    //pdo = FtRxPDO_map[slave_index];
    rd_UNLOCK();
}

void Ec_Boards_ctrl::setTxPDO(int slave_index, McESCTypes::pdo_tx pdo)
{
    wr_LOCK();
    McTxPDO_map[slave_index] = pdo;
    wr_UNLOCK();
}
void Ec_Boards_ctrl::setTxPDO(int slave_index, FtESCTypes::pdo_tx pdo)
{
    wr_LOCK();
    FtTxPDO_map[slave_index] = pdo;
    wr_UNLOCK();
}


int Ec_Boards_ctrl::recv_from_slaves() {

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

        McESC * mc = slave_as_MC(it->first);
        if ( mc ) {
            McRxPDO_map[it->first] = mc->getRxPDO();
        }

        FtESC * ft = slave_as_FT(it->first);
        if ( ft ) {
            FtRxPDO_map[it->first] = ft->getRxPDO();
        }
    }
    rd_UNLOCK();

    return 1;
}

int Ec_Boards_ctrl::send_to_slaves() {

    int wkc, retry = 2;

    { // begin std::mutex scope   
        wr_LOCK();

        for ( auto it = slaves.begin(); it != slaves.end(); it++ ) {

            McESC * mc = slave_as_MC(it->first);
            if ( mc ) {
                mc->setTxPDO(McTxPDO_map[it->first]);
            }

            FtESC * ft = slave_as_FT(it->first);
            if ( ft ) {
                ft->setTxPDO(FtTxPDO_map[it->first]);
            }
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

inline float Ec_Boards_ctrl::mailbox_recv_from_slaves_as_float(int slave_index,std::string token)
{
    float temp;
    mailbox_recv_from_slaves(slave_index,token,(void*)&temp);
    return temp;
}

inline std::string Ec_Boards_ctrl::mailbox_recv_from_slaves_as_string(int slave_index,std::string token)
{
    char temp[9];
    mailbox_recv_from_slaves(slave_index,token,(void*)&temp);
    temp[8]='\n';
    return temp;
}

inline uint64_t Ec_Boards_ctrl::mailbox_recv_from_slaves_as_int(int slave_index,std::string token)
{
    uint64_t temp;
    mailbox_recv_from_slaves(slave_index,token,(void*)&temp);
    return temp;
}


int Ec_Boards_ctrl::mailbox_recv_from_slaves(int slave_index,std::string token, void* data){
    int sub_index=0;
    int size=0;
    int main_index=0;
    get_info(Board_type::HIGH_POWER, token,main_index,sub_index,size); //TODO has to be generic
    int final_size=size;
    int wkc = get_SDO(slave_index, main_index, sub_index, &final_size,data);
    //     std::string temp; temp.resize(9); temp[4]='\0';temp[8]='\0';
    //     temp[0]=((char*)data)[0];temp[1]=((char*)data)[1];temp[2]=((char*)data)[2];temp[3]=((char*)data)[3];
    //     if (final_size>4) temp[4]=((char*)data)[4];temp[5]=((char*)data)[5];temp[6]=((char*)data)[6];temp[7]=((char*)data)[7];
    //     std::cout<<token<<" "<<main_index<<":"<<sub_index<<" =char "<<temp<<" =int "<<*(int*)data<<" =float"<<*(float*)data<<std::endl;


    if ( wkc <= 0 || final_size!=size ) {
        DPRINTF("fail sdo read\n");
    }
    return wkc;
}

int Ec_Boards_ctrl::mailbox_send_to_slaves(int slave_index,std::string token, void* data){
    int sub_index=0;
    int size=0;
    int main_index=0;
    get_info(Board_type::HIGH_POWER, token,main_index,sub_index,size); //TODO has to be generic
    int wkc = set_SDO(slave_index, main_index, sub_index, size, data);
    if ( wkc <= 0 ) {
        DPRINTF("fail sdo read\n");
    }

//     std::cout<<"-- sending token: "<<token<<"<--"<<std::endl;

    return wkc;
}

void Ec_Boards_ctrl::set_info_table(Board_type board_type, const objd_t *it)
{
    while ( it != NULL && !(is_objd_t_zeros(*it)) ) {
        std::string temp=(char*)it->name;
        info_map[board_type][temp].sdo_ptr=it;
        info_map[board_type][temp].index=it->index;
        info_map[board_type][temp].sub_index=it->subindex;
        info_map[board_type][temp].size=it->bitlength/8;
        //std::cout<<temp<<" at "<<it->subindex<<" size:"<<it->bitlength<<std::endl;
        it++;
    }
}

void Ec_Boards_ctrl::set_info_tables()
{
    const objd_t *it;

    // HIGH POWER boards
    it = &HPESC::SDOs[0];
    set_info_table(Board_type::HIGH_POWER, it);
    // LOW POWER boards
    it = &LPESC::SDOs[0];
    set_info_table(Board_type::LOW_POWER, it);
    // FORCE-TORQUE boards
    it = &FtESC::SDOs[0];
    set_info_table(Board_type::FT6, it);

}


bool Ec_Boards_ctrl::get_info(Board_type board_type, std::string token,int& main_index,int& sub_index, int& size)
{
    if ( !info_map.count(board_type) && !info_map[board_type].count(token) ) return false;
    sub_index=info_map[board_type][token].sub_index;
    size=info_map[board_type][token].size;
    main_index=info_map[board_type][token].index;
    return true;
}

bool Ec_Boards_ctrl::is_objd_t_zeros(const objd_t& objd)
{
    return(objd.index == 0) &&
    (objd.subindex == 0) &&
    (objd.datatype == 0) &&
    (objd.bitlength == 0) &&
    (objd.access == 0) &&
    (objd.data == 0) &&
    (objd.name == 0);
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
    HPESC * s = dynamic_cast<HPESC*>(slaves[slave_pos].get());
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

