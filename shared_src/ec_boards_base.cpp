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

Ec_Thread_Boards_base::Ec_Thread_Boards_base ( const char * config_yaml ) : Ec_Boards_ctrl ( config_yaml ) {
    
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
        throw "something wrong";
    }

    get_esc_map_byclass ( pows );
    DPRINTF ( "found %lu pows\n", pows.size() );
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
            throw "something wrong";
        }

    }

    get_esc_map_byclass ( motors );
    DPRINTF ( "found %lu motors\n", motors.size() );
    get_esc_map_byclass ( fts );
    DPRINTF ( "found %lu fts\n", fts.size() );
    get_esc_map_byclass ( foot_sensors );
    DPRINTF ( "found %lu foot_sensors\n", foot_sensors.size() );
    get_esc_map_byclass ( tests );
    DPRINTF ( "found %lu tests\n", tests.size() );

    for ( auto const& item : fts ) {
        DPRINTF ("pos %d == %d rid2Pos() rid %d ==  %d pos2Rid()\n",
                 item.first, rid2Pos(item.second->get_robot_id()),
                 item.second->get_robot_id(), pos2Rid(item.first) );
        assert( item.first == rid2Pos(item.second->get_robot_id()) && item.second->get_robot_id() == pos2Rid(item.first) );
    }

    
    init_preOP();

    if ( set_operative() <= 0 ) {
        throw "something else wrong";
    }

    start_time = iit::ecat::get_time_ns();
    tNow, tPre = start_time;

    if ( config["ec_boards_base"]["create_pipes"].as<bool>() ) {
        xddps_init();
    }

    init_OP();

}

void Ec_Thread_Boards_base::th_loop ( void * ) {

    tNow = iit::ecat::get_time_ns();
    s_loop ( tNow - tPre );
    tPre = tNow;

    try {

        if ( recv_from_slaves ( timing ) != iit::ecat::advr::EC_BOARD_OK ) {
            // TODO
            DPRINTF ( "recv_from_slaves FAIL !\n" );
            return;
        }

        xddps_loop();
        
        user_loop();

        send_to_slaves();

    } catch ( iit::ecat::EscWrpError &e ) {
        std::cout << e.what() << std::endl;
    }

}


void Ec_Thread_Boards_base::xddps_init ( void ) {

    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Ft6ESC * ft;
    iit::ecat::advr::FootSensorESC * fs;
 
    for ( auto const& item : motors ) {
        slave_pos = item.first;
        moto = item.second;
        xddps[slave_pos] = XDDP_pipe();
        xddps[slave_pos].init ( "Motor_id_"+std::to_string ( moto->get_robot_id() ) );
    }

    for ( auto const& item : fts ) {
        slave_pos = item.first;
        ft = item.second;
        xddps[slave_pos] = XDDP_pipe();
        xddps[slave_pos].init ( "Ft_id_"+std::to_string ( ft->get_robot_id() ) );
    }
    
    for ( auto const& item : foot_sensors ) {
        slave_pos = item.first;
        fs = item.second;
        xddps[slave_pos] = XDDP_pipe();
        xddps[slave_pos].init ( "Foot_id_"+std::to_string ( fs->get_robot_id() ) );
    }

    for ( auto const& item : pows ) {
        slave_pos = item.first;
        xddps[slave_pos] = XDDP_pipe();
        xddps[slave_pos].init ( "PowWkm_pos_"+std::to_string ( slave_pos ) );
    }

    for ( auto const& item : powCmns ) {
        slave_pos = item.first;
        xddps[slave_pos] = XDDP_pipe();
        xddps[slave_pos].init ( "PowCmn_pos_"+std::to_string ( slave_pos ) );
    }

    for ( auto const& item : tests ) {
        slave_pos = item.first;
        xddps[slave_pos] = XDDP_pipe();
        xddps[slave_pos].init ( "Test_pos_"+std::to_string ( slave_pos ) );
    }

}

void Ec_Thread_Boards_base::xddps_loop ( void ) {

    int 	slave_pos;
    uint16_t	esc_type;

    for ( auto & item : xddps ) {

        slave_pos = item.first;
        esc_type = slaves[slave_pos]->get_ESC_type();
        switch ( esc_type ) {
        case iit::ecat::advr::LO_PWR_DC_MC :
        case iit::ecat::advr::HI_PWR_AC_MC :
        case iit::ecat::advr::HI_PWR_DC_MC :
            item.second.xddp_write ( motors[slave_pos]->getRxPDO() );
            break;
        case iit::ecat::advr::FT6 :
            item.second.xddp_write ( fts[slave_pos]->getRxPDO() );
            break;
        case iit::ecat::advr::FOOT_SENSOR :
            item.second.xddp_write ( foot_sensors[slave_pos]->getRxPDO() );
            break;
        case iit::ecat::advr::POW_BOARD :
            item.second.xddp_write ( pows[slave_pos]->getRxPDO() );
            break;
        case iit::ecat::advr::POW_CMN_BOARD :
            item.second.xddp_write ( powCmns[slave_pos]->getRxPDO() );
            break;
        case iit::ecat::advr::EC_TEST :
            item.second.xddp_write ( tests[slave_pos]->getRxPDO() );
            break;

        default:
            DPRINTF ( "[WARN] ESC type %d NOT handled %s\n", esc_type, __PRETTY_FUNCTION__ );
            break;
        }
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

        motor_pdo_rx = moto->getRxPDO();
        //getRxPDO(slave_pos, motor_pdo_rx);
        moto->set_posRef ( pos_ref );

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

bool Ec_Thread_Boards_base::go_there ( const std::map<int,float> &target_pos,
                                       float eps, bool debug )
{
    return go_there(motors, target_pos, eps, debug);
}

//template <typename T>
bool Ec_Thread_Boards_base::go_there ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                                       //const std::map<int,advr::trajectory<T>> &spline_map_trj,
                                       const advr::Spline_map &spline_map_trj,
                                       float eps, bool debug )
{
    int cond, cond_cnt, cond_sum;
    float pos_ref, motor_err, link_err, motor_link_err;
    int slave_pos;
    //advr::trajectory<T> trj;
    advr::Spline_Trj trj;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    std::vector<int> truth_vect;

    cond = cond_cnt = cond_sum = 0;

    for ( auto const& item : motor_set ) {
        slave_pos = item.first;
        moto =  item.second;

        // check in the spline_trj map if the current slave_pos exist
        try {
            trj = spline_map_trj.at ( slave_pos );
        } catch ( const std::out_of_range& oor ) {
            continue;
        }

        motor_pdo_rx = moto->getRxPDO();
        //pos_ref = (float)(*trj)();
        pos_ref = ( float ) trj();
        moto->set_posRef ( pos_ref );

        link_err = fabs ( motor_pdo_rx.link_pos  - pos_ref );
        motor_err = fabs ( motor_pdo_rx.motor_pos - pos_ref );
        motor_link_err = fabs ( motor_pdo_rx.motor_pos - motor_pdo_rx.link_pos );

        cond = ( ( link_err <= eps || motor_err <= eps ) && trj.finish() ) ? 1 : 0;
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

bool Ec_Thread_Boards_base::go_there ( const advr::Spline_map &spline_map_trj,
                                       float eps, bool debug )
{
    return go_there(motors, spline_map_trj, eps, debug);
}

void Ec_Thread_Boards_base::smooth_splines_trj ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                                                 advr::Spline_map &new_spline_trj,
                                                 const advr::Spline_map &old_spline_trj,
                                                 double smooth_time )
{
    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    advr::Spline_Trj spln, old_spln;
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
            spln = new_spline_trj.at ( slave_pos );
        } catch ( const std::out_of_range& oor ) {
            DPRINTF ( "Error : new_spline_trj.at ( %d )\n", slave_pos );
            continue;
        }
        try {
            old_spln = old_spline_trj.at ( slave_pos );
        } catch ( const std::out_of_range& oor ) {
            DPRINTF ( "old_spline error\n" );
            continue;
        }

        if ( old_spln.finish() ) {
            t0_point = old_spln.end_point();
            t1_point = old_spln.get_value ( old_spln.end_time() + ( smooth_time/5 ) ,false );
        } else {
            //t0_point =  motor_pdo_rx.link_pos;
            t0_point =  old_spln();
            old_spln.get_start_time ( old_spline_start_time );
            old_spline_xtime = (double)(iit::ecat::get_time_ns() - old_spline_start_time) / 1000000000;
            t1_point = old_spln.get_value ( old_spline_xtime + ( smooth_time/5 ), false );
        }

        //
        if ( spln.end_time() > t1 ) {
            Xs = std::initializer_list<double> { 0, t1, spln.end_time() };
        } else {
            Xs = std::initializer_list<double> { 0, t1, t1+spln.end_time() };
        }
        Ys = std::initializer_list<double> {t0_point, t1_point, spln.end_point() };
        new_spline_trj[slave_pos].set_points ( Xs ,Ys );
    }

}

void Ec_Thread_Boards_base::smooth_splines_trj ( advr::Spline_map &new_spline_trj,
                                                 const advr::Spline_map &old_spline_trj,
                                                 double smooth_time )
{
    smooth_splines_trj(motors, new_spline_trj, old_spline_trj, smooth_time);
}

void Ec_Thread_Boards_base::get_trj_for_end_points ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                                                     advr::Spline_map &new_spline_trj,
                                                     std::map<int,float> &end_points,
                                                     float secs )
{
    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    advr::Spline_Trj spln;
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
            spln = new_spline_trj.at ( slave_pos );
        } catch ( const std::out_of_range& oor ) {
            DPRINTF ( "new_spline error\n" );
            continue;
        }

        Ys = std::initializer_list<double> {motor_pdo_rx.link_pos, end_points[slave_pos] };
        new_spline_trj[slave_pos].set_points ( Xs ,Ys );
    }

}

void Ec_Thread_Boards_base::get_trj_for_end_points ( advr::Spline_map &new_spline_trj,
                                                     std::map<int,float> &end_points,
                                                     float secs )
{
    get_trj_for_end_points(motors, new_spline_trj, end_points, secs);   
}

void Ec_Thread_Boards_base::set_any2home ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                                           advr::Spline_map &new_spline_trj,
                                           advr::Spline_map &old_spline_trj )
{
    get_trj_for_end_points ( motor_set, new_spline_trj, home, 5 );
    DPRINTF ( "Set_any2home\n" );
}

void Ec_Thread_Boards_base::set_any2home ( advr::Spline_map &new_spline_trj,
                                           advr::Spline_map &old_spline_trj )
{
    set_any2home(motors, new_spline_trj, old_spline_trj);
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
