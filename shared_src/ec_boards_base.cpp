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



void Ec_Thread_Boards_base::th_init(void *) {
    
    init_done = false;
    
    // init Ec_Boards_ctrl
    if ( Ec_Boards_ctrl::init() != iit::ecat::advr::EC_BOARD_OK) {
	throw "something wrong";
    }
    // get Robot_Id map 
    rid2pos = get_Rid2PosMap();
    pos2rid = get_Pos2RidMap();
    
    get_esc_map_byclass(motors);
    get_esc_map_byclass(fts);
    get_esc_map_byclass(pows);
    get_esc_map_byclass(powCmns);
    get_esc_map_byclass(tests);
    
    init_preOP();
    
    if ( set_operative() <= 0 ) {
	throw "something else wrong";
    }
    
    start_time = iit::ecat::get_time_ns();
    tNow, tPre = start_time;
    
    init_OP();
	
    xddps_init();
    
    init_done = true;
}

void Ec_Thread_Boards_base::th_loop(void *) {
	
    tNow = get_time_ns();
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

