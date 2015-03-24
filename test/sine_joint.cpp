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

#define MID_POS(m,M)    (m+(M-m)/2)

using namespace iit::ecat::advr;
using namespace iit::ecat;

extern int run_loop;
extern int main_common(void);

Ec_Boards_ctrl * ec_boards_ctrl; 

std::vector<int> right_arm = {

    Robot_IDs::RA_SH_1,
    Robot_IDs::RA_SH_2,
    Robot_IDs::RA_SH_3,
    Robot_IDs::RA_EL,
    Robot_IDs::RA_WR_1,
    Robot_IDs::RA_WR_2,
    Robot_IDs::RA_WR_3,
    Robot_IDs::RA_FT,
    Robot_IDs::RA_HA,

};

std::vector<int> left_arm = {

    Robot_IDs::LA_SH_1,
    Robot_IDs::LA_SH_2,
    Robot_IDs::LA_SH_3,
    Robot_IDs::LA_EL,
    Robot_IDs::LA_WR_1,
    Robot_IDs::LA_WR_2,
    Robot_IDs::LA_WR_3,
    Robot_IDs::LA_FT,
    Robot_IDs::LA_HA, 
};

std::map<int,float> home;
    
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

    /////////////////////////////////////////////
    // start motor
    /////////////////////////////////////////////
    float min_pos, max_pos;
    for ( auto it = left_arm.begin(); it != left_arm.end(); it++ ) {
        Motor * moto = ec_boards_ctrl->slave_as_Motor(rid2pos[*it]);
        if (moto) {
            moto->start(CTRL_SET_MIX_POS_MODE, 40.0, 0.0, 1.0);
            moto->getSDO("Min_pos", min_pos);
            moto->getSDO("Max_pos", max_pos);
            home[*it] = MID_POS(min_pos,max_pos);
        }
    }

    

    /////////////////////////////////////////////
    // set state OP
    /////////////////////////////////////////////
    if ( ec_boards_ctrl->set_operative() <= 0 ) {
        std::cout << "Error in boards set_operative()... cannot proceed!" << std::endl; 
        delete ec_boards_ctrl;
        return 0;
    }
    
    /////////////////////////////////////////////
    // bring to home pos
    /////////////////////////////////////////////
    int sentinel = 0;
    try {

        while (run_loop) {
            
            if ( ec_boards_ctrl->recv_from_slaves() != EC_BOARD_OK ) { break; }
            sentinel = 0;
            for ( auto it = left_arm.begin(); it != left_arm.end(); it++ ) {
                Motor * moto = ec_boards_ctrl->slave_as_Motor(rid2pos[*it]);
                if (moto) {
                    sentinel += moto->move_to(home[*it], 0.0005);
                }
            }
    
            ec_boards_ctrl->send_to_slaves();
            
            if ( sentinel == left_arm.size() ) { break; }
        }

    } catch (EscWrpError &e) {
            std::cout << e.what() << std::endl;
    }
    
    /////////////////////////////////////////////
    //
    /////////////////////////////////////////////
    uint64_t start_time = get_time_ns();
    uint64_t tNow, tPre = start_time;
    stat_t  s_loop;
    int fails = 0;
    int cnt = 0;
    double time = 0;
    try {

        while (run_loop) {
            
            tNow = get_time_ns();
            s_loop(tNow - tPre);
            tPre = tNow;
            time += 0.0002;
            
            if ( ec_boards_ctrl->recv_from_slaves() != EC_BOARD_OK ) { break; }
    
            for ( auto it = left_arm.begin(); it != left_arm.end(); it++ ) {
                Motor * moto = ec_boards_ctrl->slave_as_Motor(rid2pos[*it]);
                if (moto) {
                    moto->set_posRef(home[*it] + 0.2 * sinf(2*M_PI*time));
                }
            }
    
            ec_boards_ctrl->send_to_slaves();
            
        }

    } catch (EscWrpError &e) {
            std::cout << e.what() << std::endl;
    }

    DPRINTF("elapsed secs %d\n", (get_time_ns() - start_time)/1000000000L);
    print_stat(s_loop);

    ec_boards_ctrl->stop_motors();

    delete ec_boards_ctrl;

    return 1;
}
