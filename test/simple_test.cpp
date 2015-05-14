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
    
    std::vector<PowESC*> pow_boards;
    PowEscPdoTypes::pdo_rx pow_pdo_rx;
    
#if 0
    ///////////////////////////////////////////////////////////////////////////

    ec_boards_ctrl = new Ec_Boards_ctrl(argv[1]); 

    if ( ec_boards_ctrl->init() != EC_BOARD_OK) {
        std::cout << "Error in boards init()... cannot proceed!" << std::endl;      
        delete ec_boards_ctrl;
        return 0;
    }

    
    if ( ec_boards_ctrl->get_esc_bytype(POW_BOARD, pow_boards) == 1 ) {
        
        while ( ! pow_boards[0]->power_on_ok() ) {
            osal_usleep(1000000);
            pow_boards[0]->readSDO_byname("status");
            pow_boards[0]->handle_status();
        }
    }

    delete ec_boards_ctrl;

    ///////////////////////////////////////////////////////////////////////////
    sleep(6);
    ///////////////////////////////////////////////////////////////////////////
#endif
    
    
    ec_boards_ctrl = new Ec_Boards_ctrl(argv[1]); 

    if ( ec_boards_ctrl->init() != EC_BOARD_OK) {
        std::cout << "Error in boards init()... cannot proceed!" << std::endl;		
        delete ec_boards_ctrl;
        return 0;
    }

    //ec_boards_ctrl->configure_boards();

    Rid2PosMap  rid2pos = ec_boards_ctrl->get_Rid2PosMap();

    //ec_boards_ctrl->start_motors(CTRL_SET_DIRECT_MODE);
     

    /////////////////////////////////////////////
    // set state OP
    /////////////////////////////////////////////
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

#if 0
            if ( pow_boards.size() ) {
                // check emergency wireless btn
                pow_pdo_rx = pow_boards[0]->getRxPDO();
                if ( pow_pdo_rx.status.bit.vsc_status ) {
                    break;
                }
            }
#endif
            ec_boards_ctrl->send_to_slaves();
            
        }

    } catch (EscWrpError &e) {
            std::cout << e.what() << std::endl;
    }

    DPRINTF("elapsed secs %d\n", (int)((get_time_ns() - start_time)/1000000000L));
    print_stat(s_loop);

    ec_boards_ctrl->stop_motors();

    delete ec_boards_ctrl;

    return 1;
}
