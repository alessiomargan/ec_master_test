#include <iit/ecat/advr/ec_boards_iface.h>

#include <math.h>


using namespace iit::ecat::advr;


Ec_Boards_ctrl::Ec_Boards_ctrl(const char * config_file) {

    // read conf file .....

    eth_if = std::string(config_file);

    sync_cycle_time_ns = 1e6;     //   1ms
    //sync_cycle_time_ns = 0;         //   no dc 
    sync_cycle_offset_ns = 500e6;   // 500ms

    std::string pipe_name = "get_param";
    get_param_pipe = new Write_XDDP_pipe(pipe_name, 16384);
    pipe_name = "set_param";
    set_param_pipe = new Read_XDDP_pipe(pipe_name, 16384);

}

Ec_Boards_ctrl::~Ec_Boards_ctrl() {

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

    advr::TestESCTypes::pdo_tx  test_slave_1_tx_pdo;
    advr::TestESC *             test_slave_1 = new advr::TestESC(ec_slave[1]);
    slaves[1] = ESCPtr(test_slave_1);
    
    advr::McESC *               mc_slave = new advr::McESC(ec_slave[2]);
    slaves[2] = ESCPtr(mc_slave);

    advr::McESC *               mc2_slave = new advr::McESC(ec_slave[3]);
    slaves[3] = ESCPtr(mc2_slave);

    iit::ecat::setExpectedSlaves(slaves);


}

int Ec_Boards_ctrl::set_param(int slave_pos, int index, int subindex, int size, void *data) {

    return ec_SDOwrite(slave_pos, index, subindex, FALSE, size, data, EC_TIMEOUTRXM);
}

int Ec_Boards_ctrl::get_param(int slave_pos, int index, int subindex, int *size, void *data) {

    return ec_SDOread(slave_pos, index, subindex, FALSE, size, data, EC_TIMEOUTRXM);
}

void Ec_Boards_ctrl::configure_boards(void) {

    for (auto it = slaves.begin(); it != slaves.end(); it++) {
        
        //it->second-> 
    }

    short int controller_status = 2;
    //int wkc = ec_SDOwrite(2, 0x8001, 0x3, FALSE, sizeof(controller_status), &controller_status, EC_TIMEOUTRXM);
    int wkc = set_param(2, 0x8001, 0x3, sizeof(controller_status), &controller_status);
    if (wkc <= 0 ) {
        DPRINTF("fail sdo write\n");
    }
    char fw_ver[16];
    int size = 8;
    wkc = get_param(2, 0x8001, 0x1, &size, fw_ver);
    if (wkc <= 0 ) {
        DPRINTF("fail sdo write\n");
    }
    DPRINTF("FW ver %s\n", fw_ver);

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
            s->write_pdo_to_pipe();
        }
    }

    ///////////////////////////////////////////////////////////////////////////


    return 1;
}

int Ec_Boards_ctrl::send_to_slaves() {

    int wkc, retry = 2;

    for (auto it = slaves.begin(); it != slaves.end(); it++) {
        
        McESC * s = dynamic_cast<McESC*>(it->second.get());
        if (s) {
            s->read_pdo_from_pipe();
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

