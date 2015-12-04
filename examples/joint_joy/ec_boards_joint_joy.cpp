#include <ec_boards_joint_joy.h>
#include <iit/advr/coman_robot_id.h>

#include <linux/joystick.h>
#include <spnav_config.h>
#ifdef USE_X11
#undef USE_X11
#endif
#include <spnav.h>

#define MID_POS(m,M)    (m+(M-m)/2)

typedef struct js_event	js_input_t;
typedef spnav_event	spnav_input_t;

static const std::vector<double> Xt = std::initializer_list<double> { 0, 1, 2, 3, 4, 5, 6, 10 };
static const std::vector<double> Xt_1s  = std::initializer_list<double> { 0, 1 };
static const std::vector<double> Xt_3s  = std::initializer_list<double> { 0, 3 };
static const std::vector<double> Xt_5s  = std::initializer_list<double> { 0, 5 };
static const std::vector<double> Xt_10s = std::initializer_list<double> { 0, 10 };
static const std::vector<double> Xt_20s = std::initializer_list<double> { 0, 20 };


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
    
}

EC_boards_joint_joy::~EC_boards_joint_joy()
{
    
}

void EC_boards_joint_joy::init_preOP(void) {

    iit::ecat::advr::Motor * moto;
    int slave_pos;
    float min_pos, max_pos;

    std::vector<double> Ys;
    
    std::vector<int> test_rid = std::initializer_list<int> {
	
    };

    
    /////////////////////////////////////////////////////////////////////////
    // change motors map !!
    //get_esc_map_byclass(motors, test_rid);
    
    DPRINTF("found %lu <Motor> instance\n", motors.size());
    
    for ( auto const& item : motors ) {
	slave_pos = item.first;
	moto = item.second;
	moto->readSDO("Min_pos", min_pos);
        moto->readSDO("Max_pos", max_pos);
        moto->readSDO("link_pos", start_pos[slave_pos]); 
        // home 
        //home[slave_pos] = DEG2RAD(iit::ecat::advr::coman::robot_ids_home_pos_deg[pos2Rid(slave_pos)]);
	home[slave_pos] = MID_POS(min_pos,max_pos);
	step_1[slave_pos] = home[slave_pos] + 1.0;
        step_2[slave_pos] = home[slave_pos] - 1.0;
	DPRINTF("%d home %f mid %f step %f\n", pos2Rid(slave_pos), home[slave_pos],step_1[slave_pos],step_2[slave_pos]);

	Ys = std::initializer_list<double> { start_pos[slave_pos], home[slave_pos] };
	//spline_start2home[slave_pos] = new advr::Spline_Trj();
	spline_start2home[slave_pos].set_points(Xt_5s,Ys);

	Ys = std::initializer_list<double> { step_2[slave_pos], 1.5, 2.5, 1.0, 2.2, 1.0 , 2.0, home[slave_pos] };
	//spline1_trj[slave_pos] = new advr::Spline_Trj();
	spline1_trj[slave_pos].set_points(Xt,Ys);
	
	Ys = std::initializer_list<double> { home[slave_pos], 5.0, 3.5, 4.5, 4.0, 3.0 , 3.5, step_1[slave_pos] };
	//spline2_trj[slave_pos] = new advr::Spline_Trj();
	spline2_trj[slave_pos].set_points(Xt,Ys);
  
	//spline_any2home[slave_pos] = new advr::Spline_Trj();
	
	//////////////////////////////////////////////////////////////////////////
	// start controller :
	// - read actual joint position and set as pos_ref 
	moto->start(CTRL_SET_MIX_POS_MODE);
	
    }
    
    user_state = HOMING;

}


void EC_boards_joint_joy::init_OP(void) {

    running_spline = &spline_start2home;
    advr::reset_spline_trj(spline_start2home);
}

int EC_boards_joint_joy::user_loop(void) {

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
	    //if ( go_there(motors, home, 0.05) ) {
	    if ( go_there(motors, spline_start2home, 0.05, true) ) {
		user_state = STEP_1;		
		DPRINTF("At Home ....\n");
	    }
	    break;
	
	case STEP_1 :
	    if ( go_there(motors, step_1, 0.05) ) {
		user_state = STEP_2;
		DPRINTF("At Step 1 ....\n");
	    }
	    break;

	case STEP_2 :
	    if ( go_there(motors, step_2, 0.05) ) {
		user_state = TRJ_1;
		//user_state = HOMING;
		DPRINTF("At Step 2 ....\n");
		advr::reset_spline_trj(spline1_trj);
		running_spline = &spline1_trj;
	    }
	    break;

	case TRJ_1 :
	    if ( go_there(motors, spline1_trj, 0.05) ) {
		user_state = TRJ_2;
		DPRINTF("At trj_1 end ....\n");
		advr::reset_spline_trj(spline2_trj);
	    }
	    break;

	case TRJ_2 :
	    if ( go_there(motors, spline2_trj, 0.05) ) {
		//user_state = IDLE;
		/////////////////////////////
		user_state = ANY2HOME;
		set_any2home(spline_any2home,spline2_trj);
		DPRINTF("At trj_2 end ....\n");
	    }
	    break;
	    
	case ANY2HOME :
	    if ( go_there(motors, spline_any2home, 0.05) ) {
		user_state = STEP_1;
		DPRINTF("At Home ....\n");
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
	    if ( nav_cmd.button.press ) {
		switch ( nav_cmd.button.bnum ) {
		    case 1 :
		    	user_state = ANY2HOME;
			set_any2home(spline_any2home, *running_spline);
			DPRINTF("ANY2HOME ....\n");
			break;
		    case 0 :
		    	user_state = MOVING;
			DPRINTF("Moving ....\n");
			break;
		    default :
			break;
		}
	    } else {
		; // release btn
	    }
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
