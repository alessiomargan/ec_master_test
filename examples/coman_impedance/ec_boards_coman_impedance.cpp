#include <ec_boards_coman_impedance.h>
#include <iit/advr/coman_robot_id.h>

#define MID_POS(m,M)    (m+(M-m)/2)

Ec_Boards_coman_impedance::Ec_Boards_coman_impedance ( const char* config_yaml ) : Ec_Thread_Boards_base ( config_yaml ) {

    name = "Ec_Boards_coman_impedance";
    // non periodic
    period.period = {0,1};

#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max ( schedpolicy );
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    // open pipe ... xeno xddp or fifo
    inXddp.init ( "EC_board_input" );

}

Ec_Boards_coman_impedance::~Ec_Boards_coman_impedance() {

}

void Ec_Boards_coman_impedance::init_preOP ( void ) {

    std::map<int, iit::ecat::advr::Motor*>  motors_ctrl_imp;
    std::map<int, iit::ecat::advr::Motor*>  motors_ctrl_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::LpESC * lp_moto;
    int slave_pos;
    float min_pos, max_pos, pos_ref_fb;
    int16_t torque;

    std::vector<int> pos_rid = std::initializer_list<int> {

        // waist
        iit::ecat::advr::coman::WAIST_Y,
        iit::ecat::advr::coman::WAIST_P,
        iit::ecat::advr::coman::WAIST_R,
        // right leg
        iit::ecat::advr::coman::RL_H_P,
        iit::ecat::advr::coman::RL_H_R,
        iit::ecat::advr::coman::RL_H_Y,
        iit::ecat::advr::coman::RL_K,
        iit::ecat::advr::coman::RL_A_P,
        iit::ecat::advr::coman::RL_A_R,
        iit::ecat::advr::coman::RL_FT,
        // left leg
        iit::ecat::advr::coman::LL_H_P,
        iit::ecat::advr::coman::LL_H_R,
        iit::ecat::advr::coman::LL_H_Y,
        iit::ecat::advr::coman::LL_K,
        iit::ecat::advr::coman::LL_A_P,
        iit::ecat::advr::coman::LL_A_R,
        iit::ecat::advr::coman::LL_FT,
    };

    get_esc_map_byclass ( motors_ctrl_pos,  pos_rid );

    for ( auto const& item : motors_ctrl_pos ) {

        slave_pos = item.first;
        moto = item.second;
        //////////////////////////////////////////////////
        // start controller :
        moto->start ( CTRL_SET_POS_MODE);
    }
    
    std::vector<int> imp_rid = std::initializer_list<int> {
        
        //iit::ecat::advr::coman::RL_K,
        //iit::ecat::advr::coman::LL_K,
        
        // right arm
        iit::ecat::advr::coman::RA_SH_1,
        iit::ecat::advr::coman::RA_SH_2,
        iit::ecat::advr::coman::RA_SH_3,
        iit::ecat::advr::coman::RA_EL,
        iit::ecat::advr::coman::RA_WR_1,
        //iit::ecat::advr::coman::RA_WR_2,
        //iit::ecat::advr::coman::RA_WR_3,
        // left arm
        iit::ecat::advr::coman::LA_SH_1,
        iit::ecat::advr::coman::LA_SH_2,
        iit::ecat::advr::coman::LA_SH_3,
        iit::ecat::advr::coman::LA_EL,
        iit::ecat::advr::coman::LA_WR_1,
        //iit::ecat::advr::coman::LA_WR_2,
        //iit::ecat::advr::coman::LA_WR_3,
    };

    get_esc_map_byclass ( motors_ctrl_imp,  imp_rid );

    for ( auto const& item : motors_ctrl_imp ) {

        slave_pos = item.first;
        moto = item.second;
        //moto->readSDO("torque", torque);    
        //DPRINTF ( ">>> before cmd torque %d\n", torque );
        // reset torque offset
        set_flash_cmd(slave_pos, 0x00CD);
        //moto->readSDO("torque", torque);    
        //DPRINTF ( ">>> after cmd torque %d\n", torque );
        ///////////////////////////////////////////////////
        // start controller :
        // - read actual joint position and set as pos_ref
        //moto->start ( CTRL_SET_IMPED_MODE, 10000, 0, 5000 );
        // use default impedance gains P D
        moto->start ( CTRL_SET_IMPED_MODE);
    }

    //DPRINTF ( ">>> wait xddp terminal ....\n" );
    //char c; while ( termInXddp.xddp_read ( c ) <= 0 ) { osal_usleep(100); }
}

void Ec_Boards_coman_impedance::init_OP ( void ) {

}

template<class C>
int Ec_Boards_coman_impedance::user_input ( C &user_cmd ) {

    static int  bytes_cnt;
    int     bytes;
    char    cmd;

    if ( ( bytes = inXddp.xddp_read ( cmd ) ) <= 0 ) {
        return bytes;
    }

    bytes_cnt += bytes;
    DPRINTF ( ">> %d %d\n",bytes, bytes_cnt );

    return bytes;
}

int Ec_Boards_coman_impedance::user_loop ( void ) {

    int what;
    user_input ( what );
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
