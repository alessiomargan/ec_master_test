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
 	
    set_pre_op();
    stop_motors();
    iit::ecat::print_stat(s_loop);
}


void Ec_Thread_Boards_base::th_init(void *) {
    
    const YAML::Node config = get_config_YAML_Node();

    init_done = false;
    
    // init Ec_Boards_ctrl
    if ( Ec_Boards_ctrl::init() != iit::ecat::advr::EC_BOARD_OK) {
	throw "something wrong";
    }
    
    get_esc_map_byclass(motors);
    get_esc_map_byclass(fts);
    get_esc_map_byclass(pows);
    get_esc_map_byclass(powCmns);
    get_esc_map_byclass(tests);
    
    init_preOP();
    
    if ( set_operative() <= 0 ) {
	throw "something else wrong";
    }

    DPRINTF("warm up\n");
    ////////////////////////////////////////////////////////////////
    std::chrono::time_point<std::chrono::system_clock> start, now;
    start = now = std::chrono::system_clock::now();
    std::chrono::seconds loop_delay(3);
    while ( now - start <= loop_delay ) {
	try {
	    send_to_slaves();	
	} catch (iit::ecat::EscWrpError &e) {
		std::cout << e.what() << std::endl;
	}
	now = std::chrono::system_clock::now();
    }
    ////////////////////////////////////////////////////////////////
    DPRINTF("warm end\n");
    
    start_time = iit::ecat::get_time_ns();
    tNow, tPre = start_time;
    
    init_OP();
	
    if ( config["ec_boards_base"]["create_pipes"] ) {
	xddps_init();
    }
    
    init_done = true;
}

void Ec_Thread_Boards_base::th_loop(void *) {
	
    tNow = iit::ecat::get_time_ns();
    s_loop(tNow - tPre);
    tPre = tNow;
    
    try {
	
	if ( recv_from_slaves() != iit::ecat::advr::EC_BOARD_OK ) {
	    // TODO
	    DPRINTF("recv_from_slaves FAIL !\n");
	    return;
	}
	
	xddps_loop();
	
	user_loop();

	send_to_slaves();	
	
    } catch (iit::ecat::EscWrpError &e) {
	    std::cout << e.what() << std::endl;
    }
    
}


void Ec_Thread_Boards_base::xddps_init(void) {

    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Ft6ESC * ft;
    XDDP_pipe * xddp;
    
    for ( auto const& item : motors ) {
	slave_pos = item.first;
	moto = item.second;
	xddp = new XDDP_pipe();
	xddp->init("Motor_id_"+std::to_string(moto->get_robot_id()));
	xddps[slave_pos] = xddp; 
    }	
    
    for ( auto const& item : fts ) {
	slave_pos = item.first;
	ft = item.second;
	xddp = new XDDP_pipe();
	xddp->init("Ft_id_"+std::to_string(ft->get_robot_id()));
	xddps[slave_pos] = xddp;
    }
    
    for ( auto const& item : pows ) {
	slave_pos = item.first;
	xddp = new XDDP_pipe();
	xddp->init("Pow_pos_"+std::to_string(slave_pos));
	xddps[slave_pos] = xddp;
    }
    
    for ( auto const& item : powCmns ) {
	slave_pos = item.first;
	xddp = new XDDP_pipe();
	xddp->init("PowCmn_pos_"+std::to_string(slave_pos));
	xddps[slave_pos] = xddp;
    }

    for ( auto const& item : tests ) {
	slave_pos = item.first;
	xddp = new XDDP_pipe();
	xddp->init("Test_pos_"+std::to_string(slave_pos));
	xddps[slave_pos] = xddp;
    }

}

void Ec_Thread_Boards_base::xddps_loop(void) {
    
    int slave_pos;
    
    for ( auto const& item : xddps ) {
	    
	slave_pos = item.first;
	switch ( slaves[slave_pos]->get_ESC_type() ) {
	    case iit::ecat::advr::LO_PWR_DC_MC :
	    case iit::ecat::advr::HI_PWR_AC_MC :
	    case iit::ecat::advr::HI_PWR_DC_MC :
		item.second->xddp_write(motors[slave_pos]->getRxPDO());
		break;
	    case iit::ecat::advr::FT6 :
		item.second->xddp_write(fts[slave_pos]->getRxPDO());
		break;
	    case iit::ecat::advr::POW_BOARD :
		item.second->xddp_write(pows[slave_pos]->getRxPDO());
		break;
	    case iit::ecat::advr::POW_CMN_BOARD :
		item.second->xddp_write(powCmns[slave_pos]->getRxPDO());
		break;
	    case iit::ecat::advr::EC_TEST :
		item.second->xddp_write(tests[slave_pos]->getRxPDO());
		break;

	    default:
		break;
	}
    }
}

/**
 * NOTE this is a step reference !!!
 * LoPowerMotor (i.e. Coman) has a trajectory generator with max speed 0.5 rad/s
 * HiPowerMotor does NOT have it
 */
bool Ec_Thread_Boards_base::go_there(const std::map<int, iit::ecat::advr::Motor*> &motor_set,
				            const std::map<int,float> &target_pos,
				            float eps, bool debug) {

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
	try { pos_ref = target_pos.at(slave_pos); }
	catch ( const std::out_of_range& oor ) { continue; }
	
	motor_pdo_rx = moto->getRxPDO();
	moto->set_posRef(pos_ref);
	
	link_err = fabs(motor_pdo_rx.link_pos  - pos_ref);
	motor_err = fabs(motor_pdo_rx.motor_pos - pos_ref);
	motor_link_err = fabs(motor_pdo_rx.motor_pos - motor_pdo_rx.link_pos);

	cond = (link_err <= eps || motor_err <= eps) ? 1 : 0;
	cond_cnt++;
	cond_sum += cond;
	    
	if (debug) {
	    truth_vect.push_back(cond);
	    if ( ! cond ) {
		DPRINTF("%d %f %f{%f} %f{%f} {{%f}}\n",
			pos2Rid(slave_pos), pos_ref,
			motor_pdo_rx.link_pos, link_err,
			motor_pdo_rx.motor_pos, motor_err,
			motor_link_err);
	    }
	}
    }
    
    if (debug) {
	DPRINTF("---\n");
	for ( auto b : truth_vect ) { DPRINTF("%d ",b); }
	DPRINTF("\n=^=\n");
    }

    return (cond_cnt == cond_sum);
}

//template <typename T>
bool Ec_Thread_Boards_base::go_there(const std::map<int, iit::ecat::advr::Motor*> &motor_set,
					    //const std::map<int,advr::trajectory<T>> &spline_map_trj,
					    const advr::Spline_map &spline_map_trj,
					    float eps, bool debug) {

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
	try { trj = spline_map_trj.at(slave_pos); }
	catch ( const std::out_of_range& oor ) { continue; }
	
	motor_pdo_rx = moto->getRxPDO();
	//pos_ref = (float)(*trj)();
	pos_ref = (float)trj();
	moto->set_posRef(pos_ref);

	link_err = fabs(motor_pdo_rx.link_pos  - pos_ref);
	motor_err = fabs(motor_pdo_rx.motor_pos - pos_ref);
	motor_link_err = fabs(motor_pdo_rx.motor_pos - motor_pdo_rx.link_pos);
	
	cond = ((link_err <= eps || motor_err <= eps) && trj.finish()) ? 1 : 0;
	cond_cnt++;
	cond_sum += cond;
	    
	if (debug) {
	    truth_vect.push_back(cond);
	    if ( ! cond ) {
		DPRINTF("%d %f %f{%f} %f{%f} {{%f}}\n",
			pos2Rid(slave_pos), pos_ref,
			motor_pdo_rx.link_pos, link_err,
			motor_pdo_rx.motor_pos, motor_err,
			motor_link_err);
	    }
	}
    }
    
    if (debug) {
	DPRINTF("---\n");
	for ( auto b : truth_vect ) { DPRINTF("%d ",b); }
 	DPRINTF("\n=^=\n");
    }

    return (cond_cnt == cond_sum); 
}

void Ec_Thread_Boards_base::get_trj_for_end_points(advr::Spline_map &spline_map_trj,
							  std::map<int,float> &end_points,
							  float secs)
{
    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    std::vector<double> Ys;
    std::vector<double> Xs = std::initializer_list<double> { 0, secs };
    advr::Spline_Trj spln;
    
    for ( auto const& item : motors ) {
	slave_pos = item.first;
	moto = item.second;
	motor_pdo_rx = moto->getRxPDO();
	
	try { end_points.at(slave_pos); }
	catch ( const std::out_of_range& oor ) {
	    DPRINTF("Skip ends_points for slave_pos %d\n", slave_pos);
	    continue;
	}
	try { spln = spline_map_trj.at(slave_pos); }
	catch ( const std::out_of_range& oor ) {
	    DPRINTF("CAZZO\n");
	    continue;
	}
	
	Ys = std::initializer_list<double> { motor_pdo_rx.link_pos, end_points[slave_pos] };
	//Ys = std::initializer_list<double> { motor_pdo_rx.link_pos, motor_pdo_rx.link_pos + 0.01 };
	spline_map_trj[slave_pos].set_points(Xs ,Ys);
	//spln.set_points(Xs ,Ys);
	//spline_map_trj[slave_pos] = spln;
    }
    
    advr::reset_spline_trj(spline_map_trj);
}

void Ec_Thread_Boards_base::set_any2home(advr::Spline_map &spline_map_trj) {

#if 0    
    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    std::vector<double> Ys;
    std::vector<double> Xs = std::initializer_list<double> { 0, 5 };
    
    for ( auto const& item : motors ) {
	slave_pos = item.first;
	moto = item.second;
	motor_pdo_rx = moto->getRxPDO();
	Ys = std::initializer_list<double> { motor_pdo_rx.link_pos, home[slave_pos] };
	
	try { spline_map_trj.at(slave_pos); }
	catch ( const std::out_of_range& oor ) { DPRINTF("CAZZO\n");}
	
    	spline_map_trj[slave_pos].set_points(Xs ,Ys);
	
    }
    
    advr::reset_spline_trj(spline_map_trj);
#else
    get_trj_for_end_points(spline_map_trj, home, 5);
#endif    

    DPRINTF("Set_any2home\n");

}

