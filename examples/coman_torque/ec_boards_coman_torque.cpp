#include <ec_boards_coman_torque.h>
#include <iit/advr/coman_robot_id.h>

#define MID_POS(m,M)    (m+(M-m)/2)

Ec_Boards_coman_torque::Ec_Boards_coman_torque ( const char* config_yaml ) : Ec_Thread_Boards_base ( config_yaml ) {

    name = "Ec_Boards_coman_torque";
    // non periodic
    period.period = {0,1};

#ifdef __COBALT__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max ( schedpolicy );
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    // open pipe ... xeno xddp or fifo
    inXddp.init ( "EC_board_input" );

}

Ec_Boards_coman_torque::~Ec_Boards_coman_torque() {

}

void Ec_Boards_coman_torque::init_preOP ( void ) {

    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::LpESC * lp_moto;
    
    std::vector<double> Ys;
    static const std::vector<double> Xt_5s = std::initializer_list<double> { 0, 5 };

    int slave_pos;
    float min_pos, max_pos;
    int16_t torque;

    std::vector<int> pos_rid = iit::ecat::advr::coman::robot_mcs_ids;
    std::vector<int> no_control = std::initializer_list<int> {
        iit::ecat::advr::coman::RA_HA,
        iit::ecat::advr::coman::LA_HA,
    };
    std::vector<int> tor_rid = std::initializer_list<int> {
        // right arm
        iit::ecat::advr::coman::RA_SH_1,
        iit::ecat::advr::coman::RA_SH_2,
        //iit::ecat::advr::coman::RA_EL,
        // left arm
        iit::ecat::advr::coman::LA_SH_1,
        iit::ecat::advr::coman::LA_SH_2,
        //iit::ecat::advr::coman::LA_EL,
        
        //iit::ecat::advr::coman::RL_H_P,
        //iit::ecat::advr::coman::LL_H_P,
        
    };

    remove_rids_intersection(pos_rid, no_control);
    remove_rids_intersection(pos_rid, tor_rid);

    // start first torque controlled joints
    // put manually this set of joint to a zero torque configuration 
    get_esc_map_byclass ( motors_ctrl_tor,  tor_rid );
    for ( auto const& item : motors_ctrl_tor ) {

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
        // - impedance with position gains to zero --> torque
        moto->start ( CTRL_SET_IMPED_MODE, 0.0, 0.0, 0.0 );
    }

    get_esc_map_byclass ( motors_ctrl_pos,  pos_rid );
    for ( auto const& item : motors_ctrl_pos ) {

        slave_pos = item.first;
        moto = item.second;
        moto->readSDO ( "link_pos", start_pos[slave_pos] );
        home[slave_pos] = DEG2RAD ( iit::ecat::advr::coman::robot_ids_home_pos_deg[pos2Rid(slave_pos)] );
        Ys =  std::initializer_list<double> { start_pos[slave_pos], home[slave_pos] };
        spline_start2home[slave_pos].set_points ( Xt_5s, Ys );

        //////////////////////////////////////////////////
        // start controller :
        moto->start ( CTRL_SET_POS_MODE);
    }

    //DPRINTF ( ">>> wait xddp terminal ....\n" );
    //char c; while ( termInXddp.xddp_read ( c ) <= 0 ) { osal_usleep(100); }
    
    q_spln.push ( &spline_start2home );
}

void Ec_Boards_coman_torque::init_OP ( void ) {

    if ( ! q_spln.empty() ) {
        running_spline = q_spln.front();
        last_run_spline = running_spline;
        advr::reset_trj ( *running_spline );
    }
    
    DPRINTF ( "End Init_OP\n" );

}

int Ec_Boards_coman_torque::user_loop ( void ) {

    static const float spline_error = 0.07;

    int what;
    user_input ( what );
  
    if ( ! q_spln.empty() ) {

        running_spline = q_spln.front();
        if ( running_spline ) {
            // !@#%@$#%^^# ... tune error
            if ( go_there ( motors_ctrl_pos, *running_spline, spline_error, false) ) {
                // running spline has finish !!
                last_run_spline = running_spline;
                q_spln.pop();
                if ( ! q_spln.empty() ) {
                    running_spline = q_spln.front();
                    smooth_splines_trj ( motors_ctrl_pos, *running_spline, *last_run_spline );
                    advr::reset_trj ( *running_spline );
                }
            }
        } else {
            DPRINTF ( "Error NULL running spline ... pop it\n" );
            q_spln.pop();
        }
        
    } else 
    //////////////////////////////////////////////////////
    // trajectory
    //////////////////////////////////////////////////////
    {
        static uint64_t start_time_sine;
        start_time_sine = start_time_sine ? start_time_sine : iit::ecat::get_time_ns();
        uint64_t tNow = iit::ecat::get_time_ns();
        float dt = ( tNow - start_time_sine ) / 1e9;
        // !!!!! if too fast adjust this
        float freq = 0.2;
        float mN;
        iit::ecat::advr::Motor * moto;
        int slave_pos;
        for ( auto const& item : motors_ctrl_tor ) {
            slave_pos = item.first;
            moto = item.second;
            if ( pos2Rid(slave_pos) == iit::ecat::advr::coman::LA_SH_1 )
            {
                mN = -2000; 
                moto->set_torRef ( mN * (1 + cosf ( 2*M_PI*freq*dt ) ) );
            }
            if ( pos2Rid(slave_pos) == iit::ecat::advr::coman::RA_SH_1 )
            {
                mN = -2000; 
                moto->set_torRef ( mN * (1 + cosf ( 2*M_PI*freq*dt ) ) );
            }
            if ( pos2Rid(slave_pos) == iit::ecat::advr::coman::LA_SH_2 )
            {
                mN = 1500; 
                moto->set_torRef ( mN * (1 + sinf ( 2*M_PI*freq*dt ) ) );
            }
            if ( pos2Rid(slave_pos) == iit::ecat::advr::coman::RA_SH_2 )
            {
                mN = -1500; 
                moto->set_torRef ( mN * (1 + sinf ( 2*M_PI*freq*dt ) ) );
            }
//             if ( pos2Rid(slave_pos) == iit::ecat::advr::coman::RL_H_P ||
//                  pos2Rid(slave_pos) == iit::ecat::advr::coman::LL_H_P)
//             {
//                 mN = 0; 
//                 moto->set_torRef ( mN );
//             }
             
        }
    }
    //////////////////////////////////////////////////////
    // sine trajectory
    //////////////////////////////////////////////////////
}

template<class C>
int Ec_Boards_coman_torque::user_input ( C &user_cmd ) {

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


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
