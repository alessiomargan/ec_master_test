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


Ec_Thread_Boards_base::~Ec_Thread_Boards_base() {

    stop_motors();
    iit::ecat::print_stat ( s_loop );
}


void Ec_Thread_Boards_base::th_init ( void * ) {

    const YAML::Node config = get_config_YAML_Node();

    init_done = false;

    // init Ec_Boards_ctrl
    if ( Ec_Boards_ctrl::init() != iit::ecat::advr::EC_BOARD_OK ) {
        throw "something wrong";
    }

    get_esc_map_byclass ( motors );
    get_esc_map_byclass ( fts );
    get_esc_map_byclass ( pows );
    get_esc_map_byclass ( powCmns );
    get_esc_map_byclass ( tests );

    init_preOP();

    if ( set_operative() <= 0 ) {
        throw "something else wrong";
    }

    start_time = iit::ecat::get_time_ns();
    tNow, tPre = start_time;

    if ( config["ec_boards_base"]["create_pipes"] ) {
        xddps_init();
    }

    init_OP();

    init_done = true;
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
    XDDP_pipe * xddp;

    for ( auto const& item : motors ) {
        slave_pos = item.first;
        moto = item.second;
        xddp = new XDDP_pipe();
        xddp->init ( "Motor_id_"+std::to_string ( moto->get_robot_id() ) );
        xddps[slave_pos] = xddp;
    }

    for ( auto const& item : fts ) {
        slave_pos = item.first;
        ft = item.second;
        xddp = new XDDP_pipe();
        xddp->init ( "Ft_id_"+std::to_string ( ft->get_robot_id() ) );
        xddps[slave_pos] = xddp;
    }

    for ( auto const& item : pows ) {
        slave_pos = item.first;
        xddp = new XDDP_pipe();
        xddp->init ( "Pow_pos_"+std::to_string ( slave_pos ) );
        xddps[slave_pos] = xddp;
    }

    for ( auto const& item : powCmns ) {
        slave_pos = item.first;
        xddp = new XDDP_pipe();
        xddp->init ( "PowCmn_pos_"+std::to_string ( slave_pos ) );
        xddps[slave_pos] = xddp;
    }

    for ( auto const& item : tests ) {
        slave_pos = item.first;
        xddp = new XDDP_pipe();
        xddp->init ( "Test_pos_"+std::to_string ( slave_pos ) );
        xddps[slave_pos] = xddp;
    }

}

void Ec_Thread_Boards_base::xddps_loop ( void ) {

    int 	slave_pos;
    uint16_t	esc_type;

    for ( auto const& item : xddps ) {

        slave_pos = item.first;
        esc_type = slaves[slave_pos]->get_ESC_type();
        switch ( esc_type ) {
        case iit::ecat::advr::LO_PWR_DC_MC :
        case iit::ecat::advr::HI_PWR_AC_MC :
        case iit::ecat::advr::HI_PWR_DC_MC :
            item.second->xddp_write ( motors[slave_pos]->getRxPDO() );
            //item.second->xddp_write(getRxPDO<iit::ecat::advr::Motor::motor_pdo_rx_t, iit::ecat::advr::Motor>(slave_pos));
            break;
        case iit::ecat::advr::FT6 :
            item.second->xddp_write ( fts[slave_pos]->getRxPDO() );
            //item.second->xddp_write(getRxPDO<iit::ecat::advr::Ft6ESC::pdo_rx_t, iit::ecat::advr::Ft6ESC>(slave_pos));
            break;
        case iit::ecat::advr::POW_BOARD :
            item.second->xddp_write ( pows[slave_pos]->getRxPDO() );
            //item.second->xddp_write(getRxPDO<iit::ecat::advr::PowESC::pdo_rx_t, iit::ecat::advr::PowESC>(slave_pos));
            break;
        case iit::ecat::advr::POW_CMN_BOARD :
            item.second->xddp_write ( powCmns[slave_pos]->getRxPDO() );
            //item.second->xddp_write(getRxPDO<iit::ecat::advr::PowComanESC::pdo_rx_t, iit::ecat::advr::PowComanESC>(slave_pos));
            break;
        case iit::ecat::advr::EC_TEST :
            //item.second->xddp_write(tests[slave_pos]->getRxPDO());
            item.second->xddp_write ( getRxPDO<iit::ecat::advr::TestEscPdoTypes::pdo_rx,iit::ecat::advr::TestESC> ( slave_pos ) );
            break;

        default:
            DPRINTF ( "[WARN] ESC type %d NOT handled %s\n", esc_type, __PRETTY_FUNCTION__ );
            break;
        }
    }
}

/**
 * NOTE this is a step reference !!!
 * LoPowerMotor (i.e. Coman) has a trajectory generator with max speed 0.5 rad/s
 * HiPowerMotor does NOT have it
 */
bool Ec_Thread_Boards_base::go_there ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                                       const std::map<int,float> &target_pos,
                                       float eps, bool debug ) {

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

//template <typename T>
bool Ec_Thread_Boards_base::go_there ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                                       //const std::map<int,advr::trajectory<T>> &spline_map_trj,
                                       const advr::Spline_map &spline_map_trj,
                                       float eps, bool debug ) {

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


void Ec_Thread_Boards_base::smooth_splines_trj ( advr::Spline_map &new_spline_trj,
        const advr::Spline_map &old_spline_trj,
        double smooth_time ) {
    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    advr::Spline_Trj spln, old_spln;
    std::vector<double> Ys;
    std::vector<double> Xs;
    double t1 = smooth_time;
    double t0_point, t1_point;
    std::chrono::time_point<std::chrono::steady_clock> old_spline_start_time;
    std::chrono::duration<double> old_spline_xtime;

    for ( auto const& item : motors ) {
        slave_pos = item.first;
        moto = item.second;
        motor_pdo_rx = moto->getRxPDO();

        try {
            spln = new_spline_trj.at ( slave_pos );
        } catch ( const std::out_of_range& oor ) {
            DPRINTF ( "new_spline error\n" );
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
            old_spline_xtime = std::chrono::steady_clock::now() - old_spline_start_time;
            t1_point = old_spln.get_value ( old_spline_xtime.count() + ( smooth_time/5 ), false );
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


void Ec_Thread_Boards_base::get_trj_for_end_points ( advr::Spline_map &new_spline_trj,
        std::map<int,float> &end_points,
        float secs ) {
    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    advr::Spline_Trj spln;
    std::vector<double> Ys;
    std::vector<double> Xs = std::initializer_list<double> { 0, secs };

    for ( auto const& item : motors ) {
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

void Ec_Thread_Boards_base::set_any2home ( advr::Spline_map &new_spline_trj, advr::Spline_map &old_spline_trj ) {

    get_trj_for_end_points ( new_spline_trj, home, 5 );

    DPRINTF ( "Set_any2home\n" );

}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
