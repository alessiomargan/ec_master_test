/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/
#include <iit/advr/ec_boards_base.h>

Ec_Thread_Boards_base::Ec_Thread_Boards_base ( std::string config_yaml ) : Ec_Boards_ctrl ( config_yaml ) {
    
    emergencyInXddp.init ( "emergency" );
    termInXddp.init ( "terminal" );
    debugOutXddp.init ( "debugOut" );

}

Ec_Thread_Boards_base::~Ec_Thread_Boards_base() {

    std::cout << "~" << typeid ( this ).name() << std::endl;
    stop_motors();
    iit::ecat::print_stat ( s_loop );
    
}

void Ec_Thread_Boards_base::th_init ( void * ) {

    const YAML::Node config = get_config_YAML_Node();

    // init Ec_Boards_ctrl
    if ( Ec_Boards_ctrl::init() != iit::ecat::advr::EC_BOARD_OK ) {
        throw iit::ecat::advr::EcBoardsError(1,"something wrong in Ec_Boards_ctrl::init()");
    }

    pthread_barrier_wait(&threads_barrier);
    // >>> actual ECAT state is PREOP ...
    
    get_esc_map_byclass ( pows );
    DPRINTF ( "found %lu pows\n", pows.size() );
    get_esc_map_byclass ( powF28M36s );
    DPRINTF ( "found %lu powF28M36s\n", powF28M36s.size() );
    get_esc_map_byclass ( powCmns );
    DPRINTF ( "found %lu powCmns\n", powCmns.size() );
    
    // walkman power/battery board turn on all ESCs
    if ( pows.size() == 1 && slaves.size() == 1 ) {
        
        while ( ! pows[1]->power_on_ok() ) {
            osal_usleep(1000000);
            pows[1]->readSDO_byname("status");
            pows[1]->handle_status();
        }
        // power on
        Ec_Boards_ctrl::shutdown(false);
        // wait boards boot up
        sleep(6);
        // init Ec_Boards_ctrl
        if ( Ec_Boards_ctrl::init() != iit::ecat::advr::EC_BOARD_OK ) {
            throw iit::ecat::advr::EcBoardsError(2,"something wrong in Ec_Boards_ctrl::init()");
        }

    }

    get_esc_map_byclass ( motors );
    DPRINTF ( "found %lu motors\n", motors.size() );
    get_esc_map_byclass ( fts );
    DPRINTF ( "found %lu fts\n", fts.size() );
    get_esc_map_byclass ( foot_10x5 );
    DPRINTF ( "found %lu foot_10x5\n", foot_10x5.size() );
    get_esc_map_byclass ( skin_8x3 );
    DPRINTF ( "found %lu skin_8x3\n", skin_8x3.size() );
    get_esc_map_byclass ( tests );
    DPRINTF ( "found %lu tests\n", tests.size() );

//     for ( auto const& item : fts ) {
//         DPRINTF ("pos %d == %d rid2Pos() rid %d ==  %d pos2Rid()\n",
//                  item.first, rid2Pos(item.second->get_robot_id()),
//                  item.second->get_robot_id(), pos2Rid(item.first) );
//         assert( item.first == rid2Pos(item.second->get_robot_id()) && item.second->get_robot_id() == pos2Rid(item.first) );
//     }

    
    init_preOP();

    if ( Ec_Boards_ctrl::set_operative() <= 0 ) {
        throw iit::ecat::advr::EcBoardsError(3,"something wrong in Ec_Boards_ctrl::set_operative()");;
    }

    start_time = iit::ecat::get_time_ns();
    tNow, tPre = start_time;

    init_OP();
    
    emergency_active = 0;

}

void Ec_Thread_Boards_base::th_loop ( void * ) {

    uint32_t emg = 0;

    tNow = iit::ecat::get_time_ns();
    s_loop ( tNow - tPre );
    tPre = tNow;

    try {

        if ( recv_from_slaves ( timing ) != iit::ecat::advr::EC_BOARD_OK ) {
            // TODO
            DPRINTF ( "recv_from_slaves FAIL !\n" );
            return;
        }

        if ( emergencyInXddp.xddp_read ( emg ) > 0 ) {
            DPRINTF ( "EMG 0x%X\n", emg );
            emergency_active = (emg == 0xE);
        }
        
        if  ( emergency_active ) {
            // empty queue
            while ( ! trj_queue.empty() ) { trj_queue.pop_front(); }
        }
        
        user_loop();
        
        // emergency_active DO NOT write tx_pdo
        // send_to_slaves( false ); DO NOT write
        send_to_slaves( ! emergency_active );

    } catch ( iit::ecat::EscWrpError &e ) {
        std::cout << e.what() << std::endl;
    }

}


void Ec_Thread_Boards_base::remove_rids_intersection(std::vector<int> &start_dest, const std::vector<int> &to_remove)
{
    start_dest.erase(
        std::remove_if(start_dest.begin(), start_dest.end(),
            [to_remove](const int &rid) {
                return ( std::find(to_remove.begin(),to_remove.end(),rid) != to_remove.end()); }), 
        start_dest.end()
    );
    
}

bool Ec_Thread_Boards_base::set_impedance_refs ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                                                 const advr::ImpTrj_ptr_map &imp_trj_map,
                                                 float eps, bool debug )
{
    float pos_ref, vel_ref, tor_ref;
    int slave_pos;
    std::array<advr::Trj_ptr,3> trj;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    iit::ecat::advr::Motor::motor_pdo_tx_t motor_pdo_tx;

    for ( auto const& item : motor_set ) {
        slave_pos = item.first;
        moto =  item.second;

        // check in the spline_trj map if the current slave_pos exist
        try {
            trj = imp_trj_map.at ( slave_pos );
        } catch ( const std::out_of_range& oor ) {
            continue;
        }

        pos_ref = (float)(*trj[0])();
        moto->set_posRef ( pos_ref );
        vel_ref = (float)(*trj[1])();
        moto->set_velRef ( vel_ref );
        tor_ref = (float)(*trj[2])();
        moto->set_torRef ( tor_ref );

        motor_pdo_rx = moto->getRxPDO();
        motor_pdo_tx = moto->getTxPDO();
        
        
    }

    return ( trj[0]->ended() && trj[1]->ended() && trj[2]->ended() );
}


/**
 * NOTE this is a step reference !!!
 * LoPowerMotor (i.e. Coman) has a trajectory generator with max speed 0.5 rad/s
 * HiPowerMotor does NOT have it
 */
bool Ec_Thread_Boards_base::go_there ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                                       const std::map<int,float> &target_pos,
                                       float eps, bool debug )
{
    int cond, cond_cnt, cond_sum;
    float pos_ref, motor_err, link_err, motor_link_err;
    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    std::vector<int> truth_vect;

    cond = cond_cnt = cond_sum = 0;

    for ( auto const& item : motor_set ) {
        slave_pos = item.first;
        moto =  item.second;

        // check in the target_pos map if the current slave_pos exist
        try {
            pos_ref = target_pos.at ( slave_pos );
        } catch ( const std::out_of_range& oor ) {
            continue;
        }

        //getRxPDO(slave_pos, motor_pdo_rx);
        moto->set_posRef ( pos_ref );

        motor_pdo_rx = moto->getRxPDO();
        link_err = fabs ( motor_pdo_rx.link_pos  - pos_ref );
        motor_err = fabs ( motor_pdo_rx.motor_pos - pos_ref );
        motor_link_err = fabs ( motor_pdo_rx.motor_pos - motor_pdo_rx.link_pos );

        cond = ( link_err <= eps || motor_err <= eps ) ? 1 : 0;
        cond_cnt++;
        cond_sum += cond;

        if ( debug ) {
            truth_vect.push_back ( cond );
            if ( ! cond ) {
                DPRINTF ( "rId %d\tposRef %f \t link %f{%f} \t motor %f{%f} \t |motor-link|{%f}\n",
                          pos2Rid ( slave_pos ), pos_ref,
                          motor_pdo_rx.link_pos, link_err,
                          motor_pdo_rx.motor_pos, motor_err,
                          motor_link_err );
            }
        }
    }

    if ( debug ) {
        DPRINTF ( "---\n" );
        for ( auto b : truth_vect ) {
            DPRINTF ( "%d ",b );
        }
        DPRINTF ( "\n=^=\n" );
    }

    return ( cond_cnt == cond_sum );
}


bool Ec_Thread_Boards_base::go_there ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                                       const advr::Trj_ptr_map &trj_map,
                                       float eps, bool debug )
{
    int cond, cond_cnt, cond_sum;
    float pos_ref, motor_err, link_err, motor_link_err;
    int slave_pos;
    advr::Trj_ptr trj;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    std::vector<int> truth_vect;

    cond = cond_cnt = cond_sum = 0;

    for ( auto const& item : motor_set ) {
        slave_pos = item.first;
        moto =  item.second;

        // check in the spline_trj map if the current slave_pos exist
        try {
            trj = trj_map.at ( slave_pos );
        } catch ( const std::out_of_range& oor ) {
            continue;
        }

        motor_pdo_rx = moto->getRxPDO();
        pos_ref = (float)(*trj)();
        //pos_ref = ( float ) trj();
        moto->set_posRef ( pos_ref );

        link_err = fabs ( motor_pdo_rx.link_pos  - pos_ref );
        motor_err = fabs ( motor_pdo_rx.motor_pos - pos_ref );
        motor_link_err = fabs ( motor_pdo_rx.motor_pos - motor_pdo_rx.link_pos );

        cond = ( ( link_err <= eps || motor_err <= eps ) && trj->ended() ) ? 1 : 0;
        cond_cnt++;
        cond_sum += cond;

        if ( debug ) {
            truth_vect.push_back ( cond );
            if ( ! cond ) {
                DPRINTF ( "rId %d\tposRef %f \t link %f{%f} \t motor %f{%f} \t |motor-link|{%f}\n",
                          pos2Rid ( slave_pos ), pos_ref,
                          motor_pdo_rx.link_pos, link_err,
                          motor_pdo_rx.motor_pos, motor_err,
                          motor_link_err );
            }
        }
    }

    if ( debug ) {
        DPRINTF ( "---\n" );
        for ( auto b : truth_vect ) {
            DPRINTF ( "%d ",b );
        }
        DPRINTF ( "\n=^=\n" );
    }

    return ( cond_cnt == cond_sum );
}


void Ec_Thread_Boards_base::smooth_splines_trj ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                                                 advr::Trj_ptr_map &new_trj_map,
                                                 const advr::Trj_ptr_map &old_trj_map,
                                                 double smooth_time )
{
    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    advr::Trj_ptr trj, old_trj;
    std::vector<double> Ys;
    std::vector<double> Xs;
    double t1 = smooth_time;
    double t0_point, t1_point;
    uint64_t old_spline_start_time;
    double old_spline_xtime;

    for ( auto const& item : motor_set ) {
        slave_pos = item.first;
        moto = item.second;
        motor_pdo_rx = moto->getRxPDO();

        try {
            trj = new_trj_map.at ( slave_pos );
        } catch ( const std::out_of_range& oor ) {
            DPRINTF ( "Error : new_spline_trj.at ( %d )\n", slave_pos );
            continue;
        }
        try {
            old_trj = old_trj_map.at ( slave_pos );
        } catch ( const std::out_of_range& oor ) {
            DPRINTF ( "old_spline error\n" );
            continue;
        }

        if ( old_trj->ended() ) {
            t0_point = old_trj->end_point();
            t1_point = old_trj->get_value ( old_trj->end_time() + ( smooth_time/5 ) ,false );
        } else {
            //t0_point =  motor_pdo_rx.link_pos;
            t0_point =  (*old_trj)();
            old_trj->get_start_time ( old_spline_start_time );
            old_spline_xtime = (double)(iit::ecat::get_time_ns() - old_spline_start_time) / 1000000000;
            t1_point = old_trj->get_value ( old_spline_xtime + ( smooth_time/5 ), false );
        }

        //
        if ( trj->end_time() > t1 ) {
            Xs = std::initializer_list<double> { 0, t1, trj->end_time() };
        } else {
            Xs = std::initializer_list<double> { 0, t1, t1+trj->end_time() };
        }
        Ys = std::initializer_list<double> {t0_point, t1_point, trj->end_point() };
        new_trj_map[slave_pos]->set_points ( Xs ,Ys );
    }

}


void Ec_Thread_Boards_base::get_trj_for_end_points ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                                                     advr::Trj_ptr_map &new_trj,
                                                     std::map<int,float> &end_points,
                                                     float secs )
{
    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    advr::Trj_ptr trj;
    std::vector<double> Ys;
    std::vector<double> Xs = std::initializer_list<double> { 0, secs };

    for ( auto const& item : motor_set ) {
        slave_pos = item.first;
        moto = item.second;
        motor_pdo_rx = moto->getRxPDO();

        try {
            end_points.at ( slave_pos );
        } catch ( const std::out_of_range& oor ) {
            DPRINTF ( "Skip ends_points for slave_pos %d\n", slave_pos );
            continue;
        }
        try {
            trj = new_trj.at ( slave_pos );
        } catch ( const std::out_of_range& oor ) {
            DPRINTF ( "new_spline error\n" );
            continue;
        }

        Ys = std::initializer_list<double> {motor_pdo_rx.link_pos, end_points[slave_pos] };
        new_trj[slave_pos]->set_points ( Xs ,Ys );
    }

}


void Ec_Thread_Boards_base::set_any2home ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                                           advr::Trj_ptr_map &new_trj,
                                           advr::Trj_ptr_map &old_trj )
{
    get_trj_for_end_points ( motor_set, new_trj, home, 5 );
    DPRINTF ( "Set_any2home\n" );
}



////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////

bool Ec_Thread_Boards_base::go_there ( const std::map<int,float> &target_pos,
                                       float eps, bool debug )
{
    return go_there(motors, target_pos, eps, debug);
}


bool Ec_Thread_Boards_base::go_there ( const advr::Trj_ptr_map &trj_map,
                                       float eps, bool debug )
{
    return go_there(motors, trj_map, eps, debug);
}


void Ec_Thread_Boards_base::smooth_splines_trj ( advr::Trj_ptr_map &new_trj,
                                                 const advr::Trj_ptr_map &old_trj,
                                                 double smooth_time )
{
    smooth_splines_trj(motors, new_trj, old_trj, smooth_time);
}

void Ec_Thread_Boards_base::get_trj_for_end_points ( advr::Trj_ptr_map &new_trj,
                                                     std::map<int,float> &end_points,
                                                     float secs )
{
    get_trj_for_end_points(motors, new_trj, end_points, secs);   
}

void Ec_Thread_Boards_base::set_any2home ( advr::Trj_ptr_map &new_trj,
                                           advr::Trj_ptr_map &old_trj )
{
    set_any2home(motors, new_trj, old_trj);
}






// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
