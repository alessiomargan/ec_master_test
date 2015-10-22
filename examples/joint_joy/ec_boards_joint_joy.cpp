#include <ec_boards_joint_joy.h>
#include <iit/advr/coman_robot_id.h>

#define MID_POS(m,M)    (m+(M-m)/2)

EC_boards_joint_joy::EC_boards_joint_joy(const char* config_yaml) :
    Ec_Thread_Boards_base(config_yaml)
{

    name = "EC_boards_joint_joy";
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
    
    user_state = HOMING;
}

EC_boards_joint_joy::~EC_boards_joint_joy()
{
    
}

void EC_boards_joint_joy::init_preOP(void) {

    iit::ecat::advr::Motor * moto;
    int slave_pos;
    float min_pos, max_pos;
    
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
	moto->getSDO("Min_pos", min_pos);
        moto->getSDO("Max_pos", max_pos);
        moto->getSDO("link_pos", start_pos[slave_pos]); 
        // home to mid pos
        home[slave_pos] = MID_POS(min_pos,max_pos);
	// set pos to current position 
	moto->set_posRef(start_pos[slave_pos]);
	// start controller
	moto->start(CTRL_SET_MIX_POS_MODE);
    }
    
    for ( auto const& item : motors ) {
	slave_pos = item.first;
	moto = item.second;
        step_1[slave_pos] = home[slave_pos] + 2.0;
        step_2[slave_pos] = home[slave_pos] - 2.0;
    }

}

bool EC_boards_joint_joy::go_there(std::map<int, iit::ecat::advr::Motor*> motor_set,
				   std::map<int,float> target_pos,
				   float eps) {

    bool all_true = true;
    float pos_ref;
    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    std::vector<bool> truth_vect(motor_set.size()-1);
        
    for ( auto const& item : motor_set ) {
	slave_pos = item.first;
	moto =  item.second;
	
	// check in the target_pos map if the current slave_pos exist
	try { pos_ref = target_pos.at(slave_pos); }
	catch ( const std::out_of_range& oor ) { continue; }
	
	motor_pdo_rx = moto->getRxPDO();
	moto->set_posRef(pos_ref);
	
	truth_vect.push_back(
	    fabs(motor_pdo_rx.link_pos  - pos_ref) <= eps ||
	    fabs(motor_pdo_rx.motor_pos - pos_ref) <= eps );
    
	//DPRINTF("%f %f %f\n",pos_ref, motor_pdo_rx.link_pos, motor_pdo_rx.motor_pos);
    }
    
    //DPRINTF("---\n");
    std::for_each(truth_vect.begin(),truth_vect.end(),[&](bool b){
	all_true &= b;
	//DPRINTF("%d\n",b);
    });

    return all_true; 
}

void EC_boards_joint_joy::init_OP(void) {
    
}

template<class C>
int EC_boards_joint_joy::user_input(C &user_cmd) {
    
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
    
    //
    switch (user_state) {
	
	case HOMING :
	
	    if ( go_there(motors, home, 0.005) ) {
		user_state = STEP_1;
		DPRINTF("At Home ....\n");
	    }
	    break;
	
	case STEP_1 :
	    
	    if ( go_there(motors, step_1, 0.005) ) {
		user_state = STEP_2;
		DPRINTF("At Step 1 ....\n");
	    }
	    break;

	case STEP_2 :
	    
	    if ( go_there(motors, step_2, 0.005) ) {
		user_state = HOMING;
		DPRINTF("At Step 2 ....\n");
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
	    break;
    }
    
}
