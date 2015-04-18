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
    //Robot_IDs::RA_FT,
    //Robot_IDs::RA_HA,

};

std::vector<int> left_arm = {

    Robot_IDs::LA_SH_1,
    Robot_IDs::LA_SH_2,
    Robot_IDs::LA_SH_3,
    Robot_IDs::LA_EL,
    Robot_IDs::LA_WR_1,
    Robot_IDs::LA_WR_2,
    Robot_IDs::LA_WR_3,
    //Robot_IDs::LA_FT,
    //Robot_IDs::LA_HA, 
};

std::vector<int> left_leg = {

    //Robot_IDs::LL_H_Y,
    Robot_IDs::LL_H_R,
    Robot_IDs::LL_H_P,
    Robot_IDs::LL_K,
    Robot_IDs::LL_A_P,
    Robot_IDs::LL_A_R,
    //Robot_IDs::LL_FT,
};

std::vector<int> head = {

    Robot_IDs::HEAD_R,
    Robot_IDs::HEAD_P,
};    

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
    
//     Robot_IDs::WAIST_Y,
//     Robot_IDs::WAIST_R,
//     Robot_IDs::WAIST_P,
    
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


std::vector<int> motors = body; 
//std::vector<int> motors = { 1000 }; 

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


    McEscPdoTypes::pdo_rx mc_pdo_rx;
    McEscPdoTypes::pdo_tx mc_pdo_tx;
    char buffer[1024];
    
    /////////////////////////////////////////////
    // start motor
    /////////////////////////////////////////////
    float min_pos, max_pos, fault;
    for ( auto it = motors.begin(); it != motors.end(); it++ ) {
        EscWrapper * esc = ec_boards_ctrl->slave_as_EscWrapper(rid2pos[*it]);
        Motor * moto = ec_boards_ctrl->slave_as_Motor(rid2pos[*it]);
        if (moto) {
            if ( esc->get_ESC_type() == HI_PWR_AC_MC ) {
                moto->start(CTRL_SET_MIX_POS_MODE, 1000.0, 0.0, 40.0);
            } else if ( esc->get_ESC_type() == HI_PWR_DC_MC ) {
                moto->start(CTRL_SET_MIX_POS_MODE, 250.0, 0.0, 8.0);
            } else {
                moto->start(CTRL_SET_MIX_POS_MODE, 0.0, 0.0, 0.0);
            }
            //moto->start(CTRL_SET_POS_MODE, 200.0, 0.0, 0.0);
            moto->getSDO("Min_pos", min_pos);
            moto->getSDO("Max_pos", max_pos);
            moto->getSDO("position", start_pos[*it]); 
            // set home to mid pos
            home[*it] = MID_POS(min_pos,max_pos);
            // special case home
            if (*it == 12 ) { home[*it] = start_pos[*it] + 0.4; }
            if (*it == 22 ) { home[*it] = start_pos[*it] - 0.4; }
            if (*it == Robot_IDs::WAIST_Y ) { home[*it] = 0; }
            if (*it == Robot_IDs::RL_H_R ) { home[*it] = 0; }
            if (*it == Robot_IDs::LL_H_R ) { home[*it] = 0; }
            if (*it == Robot_IDs::RL_H_Y ) { home[*it] = 0; }
            if (*it == Robot_IDs::LL_H_Y ) { home[*it] = 0; }
            
            DPRINTF("%d : home pos %f\n", *it, home[*it]);
                        
        } else {
            DPRINTF("NOT a MOTOR %d\n", rid2pos[*it]);
        }
    }

    
    for ( auto it = motors.begin(); it != motors.end(); it++ ) {
        Motor * moto = ec_boards_ctrl->slave_as_Motor(rid2pos[*it]); 
        if (! moto) { continue; }
        while ( ! moto->move_to(home[*it], 0.002) && run_loop ) {
            osal_usleep(2000);    
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

    ec_boards_ctrl->recv_from_slaves();
    
    for ( auto it = motors.begin(); it != motors.end(); it++ ) {
        
        Motor * moto = ec_boards_ctrl->slave_as_Motor(rid2pos[*it]);
        if (moto) {        
            ec_boards_ctrl->getRxPDO(rid2pos[*it], mc_pdo_rx);
            mc_pdo_rx.sprint(buffer, sizeof(buffer));
            DPRINTF("%d\t %s\n",*it, buffer);
            ec_boards_ctrl->getTxPDO(rid2pos[*it], mc_pdo_tx);
            //mc_pdo_tx.tor_offs = 0.0;
            mc_pdo_tx.sprint(buffer, sizeof(buffer));
            DPRINTF("%d\t %s\n",*it, buffer);
            ec_boards_ctrl->setTxPDO(rid2pos[*it], mc_pdo_tx);
        }
    }
    
    ec_boards_ctrl->send_to_slaves();
    
    
#if 0
    for ( auto it = arm.begin(); it != arm.end(); it++ ) {
        
        Motor * moto = ec_boards_ctrl->slave_as_Motor(rid2pos[*it]);
        
        while (run_loop) {
        
            if ( ec_boards_ctrl->recv_from_slaves() != EC_BOARD_OK ) {
                break;
            }
            if ( moto->move_to(home[*it], 0.002) ) {
                break;
            }
            ec_boards_ctrl->send_to_slaves();
    
            osal_usleep(5000);
        
        }    
    }
#endif
    
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

        DPRINTF("STARTING\n");
        
        while (run_loop) {
            
            tNow = get_time_ns();
            s_loop(tNow - tPre);
            tPre = tNow;
            time += 0.001;      // dc sync 2 ms
            //time += 0.0005;   // dc sync 1 ms
            
            if ( ec_boards_ctrl->recv_from_slaves() != EC_BOARD_OK ) { break; }
    
            for ( auto it = motors.begin(); it != motors.end(); it++ ) {
                Motor * moto = ec_boards_ctrl->slave_as_Motor(rid2pos[*it]);
                if (moto) {
                    if (*it == Robot_IDs::RL_H_R || *it == Robot_IDs::LL_H_R ||
                        *it == Robot_IDs::RL_H_Y || *it == Robot_IDs::LL_H_Y) {
                        //moto->set_posRef(home[*it]);
                        moto->set_posRef(home[*it] + 0.1 * sinf(2*M_PI*time));
                    } else if ( *it == 11 || *it == 21 || *it == Robot_IDs::WAIST_Y ) {
                        //moto->set_posRef(home[*it]);
                        moto->set_posRef(home[*it] + 0.4 * sinf(2*M_PI*time));
                    } else {
                        //moto->set_posRef(home[*it]);
                        moto->set_posRef(home[*it] + 0.2 * sinf(2*M_PI*time));
                    }
                }
                ec_boards_ctrl->getTxPDO(rid2pos[*it], mc_pdo_tx);
                mc_pdo_tx.sprint(buffer, sizeof(buffer));
                //DPRINTF("%d\t %s",*it, buffer);
            }
    
            ec_boards_ctrl->send_to_slaves();
            
            
        }

    } catch (EscWrpError &e) {
            std::cout << e.what() << std::endl;
    }

    //
    req_state_check(0, EC_STATE_PRE_OP);
    
    for ( auto it = motors.begin(); it != motors.end(); it++ ) {
        Motor * moto = ec_boards_ctrl->slave_as_Motor(rid2pos[*it]); 
        if (! moto) { continue; }
        while ( ! moto->move_to(home[*it], 0.002) ) {
            osal_usleep(2000);    
        }
    }

    
    DPRINTF("elapsed secs %d\n", (int)(get_time_ns() - start_time/1000000000L));
    print_stat(s_loop);

    ec_boards_ctrl->stop_motors();

    delete ec_boards_ctrl;

    return 1;
}
