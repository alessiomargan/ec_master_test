#include <iit/ecat/advr/ec_boards_iface.h>
#include "iit/mc_tm4c/objectlist.h"

#include <math.h>
#include <pwd.h>
#include "iit/mc_tm4c/types.h"

using namespace iit::ecat::advr;



Ec_Boards_ctrl::Ec_Boards_ctrl(const char * config_file) {

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

    if ( iit::ecat::initialize(eth_if.c_str()) <= 0) {
        return 0;
    }

    factory_board();

    return 1;

}

void Ec_Boards_ctrl::factory_board(void) {
    
    int i;
    slave_cnt = ec_slavecount;

    for (i=1; i<=slave_cnt; i++) {
        
        if ( ec_slave[i].eep_id == 6 ) {
            McESC * mc_slave = new McESC(ec_slave[i]);
            slaves[i] = ESCPtr(mc_slave);
            mcSlaves[i] = mc_slave;
        }
        if ( ec_slave[i].eep_id == 1234 ) {
            TestESC * test_slave = new TestESC(ec_slave[i]);
            slaves[i] = ESCPtr(test_slave);
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
    if (wkc <= 0 || final_size!=sdo->bitlength/8 ) {
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
    if (wkc <= 0 || final_size!=sdo->bitlength/8 ) {
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

void Ec_Boards_ctrl::configure_boards(void) {

    const McESC  * mc;
    const objd_t * sdo = 0;

    for (auto it = slaves.begin(); it != slaves.end(); it++) {
        DPRINTF("Get SDOs from %d\n", it->first);
        if ( (mc = dynamic_cast<McESC*>(it->second.get())) ) {
            sdo = mc->SDOs;
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
    for (auto it = mcSlaves.begin(); it != mcSlaves.end(); it++) {

        mc = it->second;
        //mc->param. 
    }
    /**
     * 
     */
#if 1
    for (auto it = slaves.begin(); it != slaves.end(); it++) {
        DPRINTF("Set SDOs from %d\n", it->first);
        if ( (mc = dynamic_cast<McESC*>(it->second.get())) ) {
            sdo = mc->SDOs;
        } else if ( dynamic_cast<TestESC*>(it->second.get()) ) {
            sdo = 0;
        }
        while ( sdo && sdo->index ) {
            if (sdo->access == ATYPE_RW) {
                set_SDO(it->first, sdo);
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
    */
    for (auto it = mcSlaves.begin(); it != mcSlaves.end(); it++) {

        mc = it->second;
        mc->param.ctrl_status_cmd = 2;
        sdo = mc->SDOs8001 + 6;
        if (sdo) {
            DPRINTF("SDOs idx %d %s\n", sdo - mc->SDOs, sdo->name);
            set_SDO(it->first, sdo);
        }
    }
#endif


}
/**
 *  McESC objects !!!!
 * 
 * @param sPos 
 * @param cmd 
 * 
 * @return int 
 */
int Ec_Boards_ctrl::set_ctrl_status(uint16_t sPos, uint16_t cmd) {

    const objd_t * sdo = 0;
    const McESC * mc = mcSlaves[sPos];

    if (!mc) {
        return 0;
    }
    // set ctrl_status_cmd value
    mc->param.ctrl_status_cmd = cmd & 0x00FF;
    // get ctrl_status_cmd sdo pointer
    sdo = mc->SDOs8001 + 6;
    if (sdo) {
        DPRINTF("SDOs idx %d %s\n", sdo - mc->SDOs, sdo->name);
        // set value to slave
        set_SDO(sPos, sdo);
    }
    // get ctrl_status_cmd_Ack sdo pointer
    sdo = mc->SDOs8001 + 7;
    if (sdo) {
        DPRINTF("SDOs idx %d %s\n", sdo - mc->SDOs, sdo->name);
        // get value from slave
        get_SDO(sPos, sdo);
    }

    // check 
    DPRINTF("set_ctrl_status ");
    if ( mc->param.ctrl_status_cmd_ack == ((cmd & 0x00FF) | CTRL_CMD_DONE) ) {
        DPRINTF("DONE\n");
    } else if ( mc->param.ctrl_status_cmd_ack == ((cmd & 0x00FF) | CTRL_CMD_ERROR) ){
        DPRINTF("FAIL\n");
    } else {
        DPRINTF("PROTOCOL FAILURE !!!\n");
    }
    
}



int Ec_Boards_ctrl::check_sanity(void) {

    const objd_t * sdo = 0;
    const McESC * mc = 0;

    // V_batt_filt_100ms , Board_Temperature , T_mot1_filt_100ms
    const std::vector<int> x8001_offsets = {3,4,5};

    for (auto sl_it = mcSlaves.begin(); sl_it != mcSlaves.end(); sl_it++) {
        mc = sl_it->second;
        for (auto it = x8001_offsets.begin(); it != x8001_offsets.end(); it++  ) {
            sdo = mc->SDOs8001 + *it;
            if (sdo) {
                // get value from slave
                if ( get_SDO(sl_it->first, sdo) <= 0 ) {
                    ; // error
                }
            }
        }
    }

}


int Ec_Boards_ctrl::set_operative() {

    expected_wkc = iit::ecat::operational(&sync_cycle_time_ns, &sync_cycle_offset_ns);
    return expected_wkc;
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

    ///////////////////////////////////////////////////////////////////////////
    for (auto it = slaves.begin(); it != slaves.end(); it++) {
        
        McESC * s = dynamic_cast<McESC*>(it->second.get());
        if (s) {
            RxPDO_map[it->first] = s->getRxPDO();
        }
    }

    return 1;
}

int Ec_Boards_ctrl::send_to_slaves() {

    int wkc, retry = 2;

#if 1
    //////////////
    static double time;
    TxPDO_map[2].ts = get_time_ns();
    TxPDO_map[2].pos_ref = 3000 * sinf(2*M_PI*time);
    time += 0.001;
    ////////////////////
#endif

    
    for (auto it = slaves.begin(); it != slaves.end(); it++) {

        McESC * s = dynamic_cast<McESC*>(it->second.get());
        if (s) {
            s->setTxPDO(TxPDO_map[it->first]);
        }
    }

    wkc = iit::ecat::send_to_slaves();

    while ( wkc < expected_wkc && retry--) {
        DPRINTF("## wkc %d\n", wkc);
        wkc = iit::ecat::send_to_slaves();
    }

    return retry;
}



/**
 *  
 *  
 *  
 */

int Ec_Boards_ctrl::mailbox_recv_from_slaves(int slave_index,std::string token, void* data){
    int sub_index=0;
    int size=0;
    int main_index=0;
    get_info(token,main_index,sub_index,size);
    int final_size=size;
    int wkc = get_SDO(slave_index, main_index, sub_index, &final_size,data);
    if (wkc <= 0 || final_size!=size) { DPRINTF("fail sdo read\n"); }
    return wkc;
}

int Ec_Boards_ctrl::mailbox_send_to_slaves(int slave_index,std::string token, void* data){
    int sub_index=0;
    int size=0;
    int main_index=0;
    get_info(token,main_index,sub_index,size);
    int wkc = set_SDO(slave_index, main_index, sub_index, size, data);
    if (wkc <= 0 ) { DPRINTF("fail sdo read\n"); }
    return wkc;
}

void Ec_Boards_ctrl::set_info_table()
{
    const objd * lookup_table = SDO8000;
    const objd * lookup_table1 = SDO8001;
    
    int table_size = lookup_table[0].value;
    for (int i=1;i<table_size;i++)
    {
        std::string temp=(char*)lookup_table[i].data;
        info_map[temp].index=0x8000;
        info_map[temp].sub_index=lookup_table[i].subindex;
        info_map[temp].size=lookup_table[i].bitlength/8;
        std::cout<<temp<<" at "<<lookup_table[i].subindex<<" size:"<<lookup_table[i].bitlength<<std::endl;
    }
    table_size = lookup_table1[0].value;
    for (int i=1;i<table_size;i++)
    {
        std::string temp=(char*)lookup_table1[i].data;
        info_map[temp].index=0x8000;
        info_map[temp].sub_index=lookup_table1[i].subindex;
        info_map[temp].size=lookup_table1[i].bitlength/8;
        std::cout<<temp<<" at "<<lookup_table1[i].subindex<<" size:"<<lookup_table1[i].bitlength<<std::endl;
    }
}

bool Ec_Boards_ctrl::get_info(std::string token,int& main_index,int& sub_index, int& size)
{
    if (!info_map.count(token)) return false;
    sub_index=info_map[token].sub_index;
    size=info_map[token].size;
    main_index=info_map[token].index;
    return true;
}