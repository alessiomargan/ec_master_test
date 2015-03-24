#include <signal.h>
#include <sys/mman.h>
#include <execinfo.h>
#include <stdio.h>
#include <stdint.h>
#include <yaml-cpp/yaml.h>

#ifdef __XENO__
    #include <rtdk.h>
#endif

#include <memory>

#include <iit/ecat/advr/ec_boards_iface.h>

using namespace iit::ecat::advr;
using namespace iit::ecat;

extern int run_loop;
extern int main_common(void);

Ec_Boards_ctrl * ec_boards_ctrl; 


int main(int argc, char **argv)
{
    main_common();

    if ( argc != 2) {
    printf("Usage: %s config.yaml\n", argv[0]);
        return 0;
    }


    Ec_Boards_ctrl * ec_boards_ctrl;

    ///////////////////////////////////////////////////////////////////////////

    ec_boards_ctrl = new Ec_Boards_ctrl(argv[1]); 

    if ( ec_boards_ctrl->init() != EC_BOARD_OK) {
        std::cout << "Error in boards init()... cannot proceed!" << std::endl;		
        delete ec_boards_ctrl;
        return 0;
    }

    //ec_boards_ctrl->configure_boards();

    Rid2PosMap  rid2pos = ec_boards_ctrl->get_Rid2PosMap();

    ec_boards_ctrl->start_motors(CTRL_SET_DIRECT_MODE);
     
    //Motor * moto = ec_boards_ctrl->slave_as_Motor(4);
    //moto->writeSDO<float>("Direct_ref", 5.0);
        
#if 0
    Motor * moto = ec_boards_ctrl->slave_as_Motor(2);
    assert(moto);
    moto->set_off_sgn(0,1);
    float home;
    moto->readSDO("position", home);
    DPRINTF(">>>>>>>>>>> home %f\n", home);
    moto->writeSDO<float>("tor_offs", 0.0);


    if ( moto->start(CTRL_SET_POS_LNK_ERR, 13.0, 0.0035, 3.0) != EC_BOARD_OK ) {
    //if ( moto->start(CTRL_SET_MIX_POS_MODE_2) != EC_BOARD_OK ) {
        DPRINTF("Motor not started\n");
        delete ec_boards_ctrl;
        return 0;
    }

    if ( ec_boards_ctrl->set_operative() <= 0 ) {
        std::cout << "Error in boards set_operative()... cannot proceed!" << std::endl; 
        delete ec_boards_ctrl;
        return 0;
    }

    sleep(3);

    moto->start_log(true);

    uint64_t dt;
    double time = 0.0;
    McEscPdoTypes::pdo_rx mc_pdo_rx;
    McEscPdoTypes::pdo_tx mc_pdo_tx;

    mc_pdo_tx.PosGainP = 13.0;
    mc_pdo_tx.PosGainI = 0.0035; 
    mc_pdo_tx.PosGainD = 3.0;
    mc_pdo_tx.tor_offs = 0.0;
    mc_pdo_tx.pos_ref = 0;

    moto->setTxPDO(mc_pdo_tx);


    ec_boards_ctrl->send_to_slaves();

    while (run_loop) {

        if ( ! ec_boards_ctrl->recv_from_slaves() ) {
            break;
        }

        time += 0.0003;
        mc_pdo_rx = moto->getRxPDO();
        mc_pdo_tx.pos_ref = home + 1 * sinf(2*M_PI*time);
        //ec_boards_ctrl->setTxPDO(1, mc_pdo_tx);
        moto->set_posRef(mc_pdo_tx.pos_ref);

        //DPRINTF("GO %f\n", mc_pdo_tx.pos_ref);

        ec_boards_ctrl->send_to_slaves();
    }

    moto->stop();

#else

#if 0
    HubIoESC * hub_io = dynamic_cast<HubIoESC*>(ec_boards_ctrl->slave_as_EscWrapper(2));
    uint16_t reg_0x1000;

    while (run_loop) {

        hub_io->read_io_reg(reg_0x1000);
        DPRINTF("%d\n",reg_0x1000);
        sleep(1);

    }
#endif

    if ( ec_boards_ctrl->set_operative() <= 0 ) {
        std::cout << "Error in boards set_operative()... cannot proceed!" << std::endl; 
        delete ec_boards_ctrl;
        return 0;
    }

    uint64_t start_time = get_time_ns();
    uint64_t tNow, tPre = start_time;
    stat_t  s_loop;
    int fails = 0;
    int cnt = 0;
    try {

        while (run_loop) {
            
            tNow = get_time_ns();
            s_loop(tNow - tPre);
            tPre = tNow;
            
            if ( ec_boards_ctrl->recv_from_slaves() != EC_BOARD_OK ) {
                //break;
                fails++;
                if (fails > 3) {
                    break;
                }
            }
            ec_boards_ctrl->send_to_slaves();
            
            //if ((cnt++) % 10000 == 0){
            //    ec_boards_ctrl->check_DataLayer();
            //}
        }

    } catch (EscWrpError &e) {
            std::cout << e.what() << std::endl;
    }

    DPRINTF("elapsed secs %d\n", (get_time_ns() - start_time)/1000000000L);
    print_stat(s_loop);
#endif

    ec_boards_ctrl->stop_motors();

    delete ec_boards_ctrl;

    return 1;
}
