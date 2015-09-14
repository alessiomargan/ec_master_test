#include <ec_boards_joint_joy.h>

#define MID_POS(m,M)    (m+(M-m)/2)

EC_boards_joint_joy::EC_boards_joint_joy(const char* config_yaml) : Ec_Boards_ctrl(config_yaml), InXddp()
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
    
}

EC_boards_joint_joy::~EC_boards_joint_joy()
{
    print_stat(s_loop);
}

void EC_boards_joint_joy::homing(void) {

    std::map<int, iit::ecat::advr::LpESC*> motors;
    get_esc_map_bytype(iit::ecat::advr::LO_PWR_DC_MC, motors);
    
    float min_pos, max_pos, velocity;
    for ( auto it = motors.begin(); it != motors.end(); it++ ) {
	
// 	if ( it->first != rid2pos[JOINT_TEST] ) {
// 	  continue;
// 	}
	it->second->start(CTRL_SET_MIX_POS_MODE);
	it->second->getSDO("Min_pos", min_pos);
	it->second->getSDO("Max_pos", max_pos);
	it->second->getSDO("link_pos", start_pos[it->first]);
	
	// set home to mid pos
	home[it->first] = MID_POS(min_pos,max_pos);
	while ( ! it->second->move_to(home[it->first], 0.003) ) {
            osal_usleep(2000);    
        }
    }

}

void EC_boards_joint_joy::th_init(void*)
{
    // init Ec_Boards_ctrl
    if ( Ec_Boards_ctrl::init() != iit::ecat::advr::EC_BOARD_OK) {
	throw "something wrong";
    }
    // get Robot_Id map 
    rid2pos = get_Rid2PosMap();
    
    InXddp::init("EC_board_input");
    
    homing();
    
    if ( set_operative() <= 0 ) {
	throw "something else wrong";
    }
    
    start_time = get_time_ns();
    tNow, tPre = start_time;
}

void EC_boards_joint_joy::th_loop(void*)
{
  
    tNow = get_time_ns();
    s_loop(tNow - tPre);
    tPre = tNow;
    
    try {
	
	if ( recv_from_slaves() != iit::ecat::advr::EC_BOARD_OK ) {
	    // TODO
	    DPRINTF("recv_from_slaves FAIL !\n");
	    return;
	}
	    
	user_loop();

	send_to_slaves();	
	
    } catch (iit::ecat::EscWrpError &e) {
            std::cout << e.what() << std::endl;
    }
            
}

template<class C>
int EC_boards_joint_joy::user_input(C &user_cmd) {
    
    static int	bytes_cnt;
    int		bytes;
    input_t	cmd;

    if ( (bytes=xddp_read(cmd)) <= 0 ) {
	return bytes;
    }
    
    bytes_cnt += bytes;
    //DPRINTF(">> %d %d\n",bytes, bytes_cnt);
    //DPRINTF(">> %d\n",cmd.value);

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
    
    return bytes;
}

int EC_boards_joint_joy::user_loop(void) {

    std::map<int, iit::ecat::advr::LpESC*> motors;
    get_esc_map_bytype(iit::ecat::advr::LO_PWR_DC_MC, motors);
    
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::LpESC::pdo_rx_t motor_pdo_rx;
    
    static float ds; 

    //
    if ( user_input(ds) > 0 ) {
	DPRINTF(">> %f\n", ds);
    }

    for ( auto it = motors.begin(); it != motors.end(); it++ ) {
	moto =  it->second;
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
