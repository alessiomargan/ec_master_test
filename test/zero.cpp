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


std::vector<int> body = {

    Robot_IDs::HEAD_R,
    Robot_IDs::HEAD_P,
   
    Robot_IDs::RA_SH_2,
    Robot_IDs::RA_SH_1,
    Robot_IDs::RA_SH_3,
    Robot_IDs::RA_EL,
    Robot_IDs::RA_WR_1,
    Robot_IDs::RA_WR_2,
    Robot_IDs::RA_WR_3,
    //Robot_IDs::RA_FT,
    //Robot_IDs::RA_HA,
    
    Robot_IDs::LA_SH_2,
    Robot_IDs::LA_SH_1,
    Robot_IDs::LA_SH_3,
    Robot_IDs::LA_EL,
    Robot_IDs::LA_WR_1,
    Robot_IDs::LA_WR_2,
    Robot_IDs::LA_WR_3,
    //Robot_IDs::LA_FT,
    //Robot_IDs::LA_HA, 
    
    Robot_IDs::WAIST_Y,
    Robot_IDs::WAIST_P,
    Robot_IDs::WAIST_R,
    
    Robot_IDs::RL_H_R,
    Robot_IDs::LL_H_R,

    Robot_IDs::RL_H_Y,
    Robot_IDs::LL_H_Y,    
    
    Robot_IDs::RL_H_P,
    Robot_IDs::LL_H_P,
    
    Robot_IDs::RL_K,
    Robot_IDs::LL_K,
    
    Robot_IDs::RL_A_P,
    Robot_IDs::LL_A_P,
    
    Robot_IDs::RL_A_R,
    Robot_IDs::LL_A_R,

    //Robot_IDs::RL_FT,
    //Robot_IDs::LL_FT,

    
};


//std::vector<int> motors = body; 
std::vector<int> motors = { 1000 }; 

std::map<int,float> home;
std::map<int,float> start_pos;
    
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
    //
    /////////////////////////////////////////////
    for ( auto it = motors.begin(); it != motors.end(); it++ ) {
        HpESC * hp = ec_boards_ctrl->slave_as_HP(rid2pos[*it]);
        if (hp) {
            if ( *it == Robot_IDs::WAIST_P || *it == Robot_IDs::LL_A_P ) {
                // 160 deg - 2.79 rad 
                hp->set_zero_position(2.7925);
            } else if ( *it == Robot_IDs::RL_A_P ) {
                // 200 deg - 3.4906 rad 
                hp->set_zero_position(3.4906);
            } else {
                // almost all motor has pi
                hp->set_zero_position(M_PI);
            }
        } else {
            DPRINTF("SET_ZERO_POSITION only for AC and DC MOTOR\n");
        }
    }

    
    delete ec_boards_ctrl;

    return 1;
}
