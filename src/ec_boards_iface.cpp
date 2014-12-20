#include <iit/ecat/advr/ec_boards_iface.h>
#include "iit/mc_tm4c/objectlist.h"

#include <math.h>
#include <pwd.h>
#include "iit/mc_tm4c/types.h"

using namespace iit::ecat::advr;


Ec_Boards_ctrl::Ec_Boards_ctrl(const char * config_file) {

    // read conf file .....
    eth_if = std::string(config_file);

    //sync_cycle_time_ns = 1e6;     //   1ms
    //sync_cycle_time_ns = 100e6;     //   100ms
    sync_cycle_time_ns = 0;         //   no dc 
    sync_cycle_offset_ns = 500e6;   // 500ms
    
    set_info_table();
}

Ec_Boards_ctrl::~Ec_Boards_ctrl() {

    iit::ecat::finalize();

    for (auto it = slaves.begin(); it != slaves.end(); it++) {
        delete it->second.get();
    }
}

int Ec_Boards_ctrl::init(void) {

    if ( iit::ecat::initialize(eth_if.c_str()) <= 0) {
        return 0;
    }

    factory_board();

    return 1;

}

void Ec_Boards_ctrl::factory_board(void) {

    //advr::TestESCTypes::pdo_tx  test_slave_1_tx_pdo;
    //advr::TestESC *             test_slave_1 = new advr::TestESC(ec_slave[1]);
    //slaves[1] = ESCPtr(test_slave_1);
    
    advr::McESC *mc_slave = new advr::McESC(ec_slave[1]);
    slaves[1] = ESCPtr(mc_slave);

    TxPDO_map[1];
    //advr::McESC *               mc2_slave = new advr::McESC(ec_slave[3]);
    //slaves[3] = ESCPtr(mc2_slave);

    iit::ecat::setExpectedSlaves(slaves);


}

int Ec_Boards_ctrl::set_param(int slave_pos, int index, int subindex, int size, void *data) {

    return ec_SDOwrite(slave_pos, index, subindex, FALSE, size, data, EC_TIMEOUTRXM);
}

int Ec_Boards_ctrl::get_param(int slave_pos, int index, int subindex, int *size, void *data) {

    return ec_SDOread(slave_pos, index, subindex, FALSE, size, data, EC_TIMEOUTRXM);
}

void Ec_Boards_ctrl::configure_boards(void) {
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
            RxPDO_map[it->first]=s->getRxPDO();
        }
    }

    return 1;
}

int Ec_Boards_ctrl::send_to_slaves() {

    int wkc, retry = 2;

    for (auto it = slaves.begin(); it != slaves.end(); it++) {
        
        McESC * s = dynamic_cast<McESC*>(it->second.get());
        if (s) {
            s->setTxPDO(TxPDO_map[it->first]);
        }
    }
    //////////////
    //McESC * s = dynamic_cast<McESC*>(slaves[2].get());
    //advr::McESCTypes::pdo_tx    mc_tx_pdo;
    //static double time;
    //mc_tx_pdo.ts = get_time_ns();
    //mc_tx_pdo.direct_ref = 3000 * sinf(2*M_PI*time);
    //time += 0.001;
    //s->setTxPDO(mc_tx_pdo);
    ////////////////////

    wkc = iit::ecat::send_to_slaves();

    while ( wkc < expected_wkc && retry--) {
        DPRINTF("## wkc %d\n", wkc);
        wkc = iit::ecat::send_to_slaves();
    }

    return retry;
}

int Ec_Boards_ctrl::mailbox_recv_from_slaves(int slave_index,std::string token, void* data){
    int sub_index=0;
    int size=0;
    int main_index=0;
    get_info(token,main_index,sub_index,size);
    int final_size=size;
    int wkc = get_param(slave_index, main_index, sub_index, &final_size,data);
    if (wkc <= 0 || final_size!=size) { DPRINTF("fail sdo read\n"); }
    
//     std::string temp; temp.resize(9); temp[4]='\0';temp[8]='\0';
//     temp[0]=((char*)data)[0];temp[1]=((char*)data)[1];temp[2]=((char*)data)[2];temp[3]=((char*)data)[3];
//     if (final_size>4) temp[4]=((char*)data)[4];temp[5]=((char*)data)[5];temp[6]=((char*)data)[6];temp[7]=((char*)data)[7];
//     std::cout<<token<<" "<<main_index<<":"<<sub_index<<" =char "<<temp<<" =int "<<*(int*)data<<" =float"<<*(float*)data<<std::endl;
  
    return wkc;
}

int Ec_Boards_ctrl::mailbox_send_to_slaves(int slave_index,std::string token, void* data){
    int sub_index=0;
    int size=0;
    int main_index=0;
    get_info(token,main_index,sub_index,size);
    int wkc = set_param(slave_index, main_index, sub_index, size, data);
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
        info_map[temp].index=0x8001;
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