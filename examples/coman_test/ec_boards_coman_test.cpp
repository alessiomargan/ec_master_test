#include <ec_boards_coman_test.h>
#include <iit/advr/coman_robot_id.h>

#include <linux/joystick.h>
#include <spnav.h>

#define MID_POS(m,M)    (m+(M-m)/2)


typedef struct js_event	js_input_t;
typedef spnav_event	spnav_input_t;


static const std::vector<double> Xt_3s = std::initializer_list<double> { 0, 3 };
static const std::vector<double> Xt_5s = std::initializer_list<double> { 0, 5 };
static const std::vector<double> Xt_9s = std::initializer_list<double> { 0, 9 };


EC_boards_coman_test::EC_boards_coman_test(const char* config_yaml) :
    Ec_Thread_Boards_base(config_yaml)
{

    name = "EC_boards_coman_test";
    // not periodic 
    period.period = {0,1};

#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max(schedpolicy);
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
    
    // open pipe ... xeno xddp or fifo 
    jsInXddp.init("EC_board_js_input");
    navInXddp.init("EC_board_nav_input");
    
}

EC_boards_coman_test::~EC_boards_coman_test()
{
    
}

/*
 * 
 */
void EC_boards_coman_test::init_preOP(void) {

    iit::ecat::advr::Motor * moto;
    int slave_pos;
    float min_pos, max_pos, pos_ref_fb;
    
    std::vector<int> test_rid = std::initializer_list<int> {
	iit::ecat::advr::coman::LA_SH_1,
	iit::ecat::advr::coman::LA_SH_2,
	iit::ecat::advr::coman::LA_SH_3,
	iit::ecat::advr::coman::LA_EL,
	//
	iit::ecat::advr::coman::RA_SH_1,
	iit::ecat::advr::coman::RA_SH_2,
	iit::ecat::advr::coman::RA_SH_3,
	iit::ecat::advr::coman::RA_EL,
    };
    
    get_esc_map_byclass(left_leg,  iit::ecat::advr::coman::robot_left_leg_ids);
    DPRINTF("found %lu <Motor> left_leg\n", left_leg.size());

    get_esc_map_byclass(left_arm,  iit::ecat::advr::coman::robot_left_arm_ids);
    DPRINTF("found %lu <Motor> left_arm\n", left_arm.size());

    get_esc_map_byclass(right_leg, iit::ecat::advr::coman::robot_right_leg_ids);
    DPRINTF("found %lu <Motor> right_leg\n", right_leg.size());

    get_esc_map_byclass(right_arm, iit::ecat::advr::coman::robot_right_arm_ids);
    DPRINTF("found %lu <Motor> right_arm\n", right_arm.size());

    get_esc_map_byclass(waist,	   iit::ecat::advr::coman::robot_waist_ids);
    DPRINTF("found %lu <Motor> waist\n", waist.size());
    
    // !!!
    //get_esc_map_byclass(motors,  test_rid);
    
    // 
    Xs.reserve(128);
    Ys.reserve(128);
    
    for ( auto const& item : motors ) {
	slave_pos = item.first;
	moto = item.second;
	moto->readSDO("Min_pos", min_pos);
        moto->readSDO("Max_pos", max_pos);
        moto->readSDO("link_pos", start_pos[slave_pos]); 
        // home 
        home[slave_pos] = DEG2RAD(iit::ecat::advr::coman::robot_ids_home_pos_deg[pos2Rid(slave_pos)]);
	//home[slave_pos] = MID_POS(min_pos,max_pos);
	mid_pos[slave_pos] = MID_POS(min_pos,max_pos); //home[slave_pos] + 1.0;
        step_2[slave_pos] = start_pos[slave_pos]; //MID_POS(min_pos,max_pos) + 0.2;
	DPRINTF("Joint_id %d start %f home %f mid %f\n", pos2Rid(slave_pos), start_pos[slave_pos], home[slave_pos], mid_pos[slave_pos]);
	
	Ys =  std::initializer_list<double> { start_pos[slave_pos], home[slave_pos] };
	spline_start2home[slave_pos] = new advr::Spline_Trj();
	spline_start2home[slave_pos]->set_points(Xt_3s, Ys);

	Ys = std::initializer_list<double> { home[slave_pos], mid_pos[slave_pos] };
	spline_home2mid[slave_pos] = new advr::Spline_Trj();
	spline_home2mid[slave_pos]->set_points(Xt_5s, Ys);

	Ys = std::initializer_list<double> { mid_pos[slave_pos], home[slave_pos] };
	spline_mid2home[slave_pos] = new advr::Spline_Trj();
	spline_mid2home[slave_pos]->set_points(Xt_5s, Ys);

	spline_any2home[slave_pos] = new advr::Spline_Trj();

	//////////////////////////////////////////////////////////////////////////
	// start controller :
	// - read actual joint position and set as pos_ref 
	moto->start(CTRL_SET_MIX_POS_MODE);
    }

    /////////////////////////////////////////////////////
    // 
    /////////////////////////////////////////////////////
    user_state = IDLE;
    //home_state = END_HOME;
    //home_state = LEFT_LEG_HOME;
    
}

void EC_boards_coman_test::set_any2home(void) {
    
    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;

    for ( auto const& item : motors ) {
	slave_pos = item.first;
	moto = item.second;
	motor_pdo_rx = moto->getRxPDO();
	Ys[0] = motor_pdo_rx.link_pos;
	Ys[1] = home[slave_pos];
	spline_any2home[slave_pos]->set_points(Xt_5s,Ys);
    }
    DPRINTF("Set_any2home\n");

}

void EC_boards_coman_test::init_OP(void) {
    
#if 0
    char buff[128];
    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    iit::ecat::advr::Motor::motor_pdo_tx_t motor_pdo_tx;
    
    for ( auto const& item : motors ) {
	slave_pos = item.first;
	moto = item.second;
	motor_pdo_tx = moto->getTxPDO();
	motor_pdo_tx.sprint(buff, sizeof(buff));
	DPRINTF("[%d] %s\n",slave_pos, buff);
	motor_pdo_rx = moto->getRxPDO();
	motor_pdo_rx.sprint(buff, sizeof(buff));
	DPRINTF("[%d] %s\n",slave_pos, buff);
    }
#endif

    
    DPRINTF("End Init_OP\n");
}

int EC_boards_coman_test::user_loop(void) {

    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    static float ds; 
   
    //
    if ( user_input(ds) > 0 ) {
	DPRINTF(">> %f\n", ds);
    }

    //
    switch (user_state) {
	
	case IDLE :
	    break;
	    
	case HOMING :
	    switch (home_state) {
		case TEST_HOME :
		    if ( go_there(motors, spline_start2home, 0.05) ) {
			DPRINTF("Test at Home ....\n");
			home_state = END_HOME;
			advr::reset_spline_trj(spline_start2home);
		    }
		    break;
		///////////////////////////////////////////////////////////
		case LEFT_LEG_HOME :
		    if ( go_there(left_leg, spline_start2home, 0.05) ) {
			DPRINTF("Left leg at Home ....\n");
			home_state = LEFT_ARM_HOME;
			advr::reset_spline_trj(spline_start2home);
		    }
		    break;
		case LEFT_ARM_HOME :
		    if ( go_there(left_arm, spline_start2home, 0.05) ) {
			DPRINTF("Left arm at Home ....\n");
			home_state = RIGHT_LEG_HOME;		
			advr::reset_spline_trj(spline_start2home);
		    }
		    break;
		case RIGHT_LEG_HOME :
		    if ( go_there(right_leg, spline_start2home, 0.05) ) {
			DPRINTF("Right leg at Home ....\n");
			home_state = RIGHT_ARM_HOME;		
			advr::reset_spline_trj(spline_start2home);
		    }
		    break;
		case RIGHT_ARM_HOME :
		    if ( go_there(right_arm, spline_start2home, 0.05) ) {
			DPRINTF("Right arm at Home ....\n");
			home_state = WAIST_HOME;		
			advr::reset_spline_trj(spline_start2home);
		    }
		    break;
		case WAIST_HOME :
		    if ( go_there(waist, spline_start2home, 0.05) ) {
			DPRINTF("Waist at Home ....\n");
			home_state = END_HOME;		
		    }
		    break;
		/////////////////////////////////////////////////////////////
		case END_HOME :
		    user_state = IDLE;
		    break;
		default:
		    DPRINTF("Wrong home state ....\n");
		    break;
	    } // end home_state switch
	    break;
		    
	case HOME2MID :
#if 0
	    if ( ! go_there(left_leg, mid_pos, 0.05) ) {
		//
	    } else if ( ! go_there(left_arm, mid_pos, 0.05) ) {
		//
	    } else if ( ! go_there(right_leg, mid_pos, 0.05) ) {
		//
	    } else if ( ! go_there(right_arm, mid_pos, 0.05) ) {
		//
	    } else if ( ! go_there(waist, mid_pos, 0.05) ) {
		//
	    } else {
		DPRINTF("At Step 1 ....\n");
		user_state = MOVING;
	    }
#else
	    if ( go_there(motors, spline_home2mid, 0.05) ) {
		user_state = IDLE;
		DPRINTF("End home2mid ....state IDLE\n");
	    }
#endif
	    break;
	   
	case MID2HOME :
	    if ( go_there(motors, spline_mid2home, 0.05) ) {
		user_state = IDLE;
		DPRINTF("End mid2home ....state IDLE\n");
	    }
	    break;
	    
	case ANY2HOME :
	    if ( go_there(motors, spline_any2home, 0.05) ) {
		user_state = IDLE;
		DPRINTF("End ant2home ....state IDLE\n");
	    }
	    break;
	    
	case MOVING :
	    for ( auto const& item : motors ) {
		moto =  item.second;
		motor_pdo_rx = moto->getRxPDO();
		// pos_ref_fb is the previous reference
		moto->set_posRef(motor_pdo_rx.pos_ref_fb + ds);
	    }
	    break;
	    
	default:
	    DPRINTF("Wrong user state ....\n");
	    break;
    }
    
}

template<class C>
int EC_boards_coman_test::user_input(C &user_cmd) {
    
    static int	bytes_cnt;
    int		bytes;
    
    spnav_input_t	nav_cmd;
    js_input_t		js_cmd;
    
    if ( (bytes = navInXddp.xddp_read(nav_cmd)) > 0 ) {
	//user_cmd = process_spnav_input(nav_cmd);
	// [-1.0 .. 1.0] / 200 ==> 0.005 rad/ms
	if ( nav_cmd.type == SPNAV_EVENT_MOTION ) {
	    user_cmd = ((float)nav_cmd.motion.ry / (350.0)) / 200 ;
	} else if (nav_cmd.type == SPNAV_EVENT_BUTTON ) {
	    user_state = MOVING;
	    DPRINTF("Moving ....\n");
	}
    }
    
    bytes_cnt += bytes;
    //DPRINTF(">> %d %d\n",bytes, bytes_cnt);

    if ( (bytes = jsInXddp.xddp_read(js_cmd)) > 0 ) {
	
	//user_cmd = process_js_input(js_cmd);
	switch (js_cmd.type & ~JS_EVENT_INIT)
	{
	    case JS_EVENT_AXIS:
		switch ( js_cmd.number ) {
		    case 0 :
			// [-1.0 .. 1.0] / 500 ==> [-0.002 .. 0.002] rads
			user_cmd = ((float)js_cmd.value/(32767.0)) / 500 ;
			break;
		    case 2 :
			// [-1.0 .. 1.0] / 250 ==> [-0.004 .. 0.004] rads
			user_cmd = ((float)js_cmd.value/(32767.0)) / 250 ;
			break;
		    default:
			break;
		}
		break;
            
	    case JS_EVENT_BUTTON :
		switch ( js_cmd.number ) {
		    case 0 :
			if (js_cmd.value && user_state == IDLE) { 
			    user_state = HOME2MID;
			    advr::reset_spline_trj(spline_home2mid);
			}
			break;
		    case 1 :
			if (js_cmd.value && user_state == IDLE) { 
			    user_state = MID2HOME;
			    advr::reset_spline_trj(spline_mid2home);
			}
			break;
		    case 2 :
			if (js_cmd.value && user_state == IDLE) { 
			    set_any2home();
			    user_state = ANY2HOME;
			    advr::reset_spline_trj(spline_any2home);
			}
			break;
		    case 9 :
			if (js_cmd.value && user_state == IDLE) { 
			    user_state = HOMING;
			    home_state = TEST_HOME;
			    advr::reset_spline_trj(spline_start2home);
			}
			break;
		    case 4 :
		    case 5 :
		    case 6 :
		    case 7 :
			user_state == IDLE;
			DPRINTF("Set IDLE state\n");
			break;
		    default:
			DPRINTF("Not handle : cmd %d value %d\n", js_cmd.number, js_cmd.value);
			break;
		}
		break;
	    
	    default:
		break;
		    
	}
    }

    bytes_cnt += bytes;
    //DPRINTF(">> %d %d\n",bytes, bytes_cnt);

    return bytes;
}
