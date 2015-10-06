#include <ec_boards_joint_joy.h>
#include <coman_robot_id.h>

#define MID_POS(m,M)    (m+(M-m)/2)

EC_boards_joint_joy::EC_boards_joint_joy(const char* config_yaml) :
    Ec_Thread_Boards_base(config_yaml),
    JsInXddp(),
    NavInXddp()
{

    name = "EC_boards_joint_joy";
    // do not go above .... 
    period.period = {0,500};

#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max(schedpolicy);
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
    
    // open pipe ... xeno xddp or fifo 
    JsInXddp::init("EC_board_js_input");
    NavInXddp::init("EC_board_nav_input");
    
}

EC_boards_joint_joy::~EC_boards_joint_joy()
{
    
}

void EC_boards_joint_joy::init_preOP(void) {

    iit::ecat::advr::Motor * moto;
    int slave_pos;
    float min_pos, max_pos;
    
    //std::map<int, iit::ecat::advr::LpESC*> lp_motors;
    //get_esc_map_bytype(iit::ecat::advr::LO_PWR_DC_MC, lp_motors);
    
    std::vector<int> wrist_rid = std::initializer_list<int> {
	iit::ecat::advr::coman::LA_WR_1,
	iit::ecat::advr::coman::LA_WR_2,
	iit::ecat::advr::coman::LA_WR_3,
	
    };
    
    // fill motors map
    get_esc_map_byclass(motors);
    //get_esc_map_byclass(motors, iit::ecat::advr::coman::robot_left_arm_ids);
    //get_esc_map_byclass(motors, wrist_rid);
    DPRINTF("found %lu <Motor> instance\n", motors.size());
    
    for ( auto const& item : motors ) {
	slave_pos = item.first;
	moto = item.second;
	moto->start(CTRL_SET_MIX_POS_MODE);
        moto->getSDO("Min_pos", min_pos);
        moto->getSDO("Max_pos", max_pos);
        moto->getSDO("link_pos", start_pos[slave_pos]); 
        // set home to mid pos
        home[slave_pos] = MID_POS(min_pos,max_pos);
    }
    
    for ( auto const& item : motors ) {
	slave_pos = item.first;
	moto = item.second;
	while ( ! moto->move_to(home[slave_pos], 0.005) ) {
	    osal_usleep(2000);    
	}
    }

}

void EC_boards_joint_joy::init_OP(void) {
  
}

template<class C>
int EC_boards_joint_joy::user_input(C &user_cmd) {
    
    static int	bytes_cnt;
    int		bytes;
    
    spnav_input_t	nav_cmd;
    js_input_t		js_cmd;
    
    if ( (bytes = NavInXddp::xddp_read(nav_cmd)) > 0 ) {
	//user_cmd = process_spnav_input(nav_cmd);
	// [-1.0 .. 1.0] / 200 ==> 0.005 rad/ms
	user_cmd = ((float)nav_cmd.motion.ry / (350.0)) / 200 ;
    }
    
    bytes_cnt += bytes;
    //DPRINTF(">> %d %d\n",bytes, bytes_cnt);

    if ( (bytes = JsInXddp::xddp_read(js_cmd)) > 0 ) {
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
            
	    default:
		break;
		    
	}
	
    }

    bytes_cnt += bytes;
    //DPRINTF(">> %d %d\n",bytes, bytes_cnt);
    
#if 0
    //////////////////////////////////////////////////////////////////////
    float ax_value = 0;
    
    switch (cmd.type & ~JS_EVENT_INIT)
    {
	case JS_EVENT_AXIS:
	    // normalize to [-1.0 .. 1.0]
	    ax_value = (float)cmd.value/(32767.0);
	    
	    switch ( cmd.number ) {
		case 0 :
		    // [-0.002 .. 0.002] rads 
		    user_cmd = ax_value / 500;
		    break;
		case 2 :
		    // [-0.004 .. 0.004] rads 
		    user_cmd = ax_value / 250;
		    break;
		default:
		    break;
	    }
            break;
            
	case JS_EVENT_BUTTON:
	    switch ( cmd.number ) {
		case 0 :
		    break;
		default:
		    if (cmd.value) {
			//slave_as_LP(rid2pos[JOINT_TEST])->print_info();
		    }
		    break;
	    }
            break;
	
	default:
	    break;
		    
    }
    //////////////////////////////////////////////////////////////////////
#else
    // [-1..1] / 200 ==> 0.005 rad/ms
    //user_cmd = ((float)nav_cmd.motion.ry / (350.0)) / 200;
#endif

    return bytes;
}

int EC_boards_joint_joy::user_loop(void) {

    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::LpESC::pdo_rx_t motor_pdo_rx;
    
    static float ds; 

    //
    if ( user_input(ds) > 0 ) {
	DPRINTF(">> %f\n", ds);
    }

    for ( auto const& item : motors ) {
	moto =  item.second;
	motor_pdo_rx = moto->getRxPDO();
	// pos_ref_fb is the previous reference
	moto->set_posRef(motor_pdo_rx.pos_ref_fb + ds);
    }
    
#if 0
    moto = slave_as_Motor(rid2pos[10]);
    motor_pdo_rx = moto->getRxPDO();
    moto->set_posRef(motor_pdo_rx.pos_ref_fb + ds);

    moto = slave_as_Motor(rid2pos[15]);
    motor_pdo_rx = moto->getRxPDO();
    moto->set_posRef(motor_pdo_rx.pos_ref_fb + ds);

    moto = slave_as_Motor(rid2pos[8]);
    motor_pdo_rx = moto->getRxPDO();
    moto->set_posRef(motor_pdo_rx.pos_ref_fb + ds);

    moto = slave_as_Motor(rid2pos[13]);
    motor_pdo_rx = moto->getRxPDO();
    moto->set_posRef(motor_pdo_rx.pos_ref_fb + ds);

    moto = slave_as_Motor(rid2pos[9]);
    motor_pdo_rx = moto->getRxPDO();
    moto->set_posRef(motor_pdo_rx.pos_ref_fb + ds);

    moto = slave_as_Motor(rid2pos[14]);
    motor_pdo_rx = moto->getRxPDO();
    moto->set_posRef(motor_pdo_rx.pos_ref_fb + ds);
    
#endif
}
