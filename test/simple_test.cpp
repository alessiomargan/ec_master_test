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
            ec_boards_ctrl->send_to_slaves();
            
            //if ((cnt++) % 10000 == 0){
            //    ec_boards_ctrl->check_DataLayer();
            //}
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
