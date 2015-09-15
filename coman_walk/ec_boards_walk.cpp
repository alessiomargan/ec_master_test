#include <ec_boards_walk.h>
#include <coman_robot_id.h>

#include <RTControl.h>
extern void test_joint(std::vector<float> homePos, int size, double freq, int r_pos[]);

#define MID_POS(m,M)    (m+(M-m)/2)
#define DEG2RAD(X)  (X*M_PI)/180.0

// home position in degree
static const std::vector<float> homePos = {
    // lower body #15
//    0, -1,  0,  0,  0,  0,  0,   0,  0,  0,  0,   0,   0,  0,  0,
    0, 6,  0,  0,  0,  0,  0,   0,  0,  0,  0,   0,   0,  0,  0,
//  1,  2,  3,  4,  5,  6,  7,   8,  9, 10,  11, 12,  13, 14, 15
    // upper body #10 right arm to left arm, last 2 are right and left neck
    0, 90,  0,  -90, 0, -90,  0, -90,  0,  0};
// 16, 17, 18,  19, 20,  21, 22,  23, 24, 25


EC_boards_walk::EC_boards_walk(const char* config_yaml) : Ec_Boards_ctrl(config_yaml), InXddp()
{

    name = "EC_boards_walk";
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

EC_boards_walk::~EC_boards_walk()
{
    print_stat(s_loop);
}

void EC_boards_walk::homing(void) {

    int rId;
    float min_pos, max_pos, velocity;
    for ( auto it = motors.begin(); it != motors.end(); it++ ) {
	rId = rid2pos[it->first];
	it->second->start(CTRL_SET_MIX_POS_MODE);
	it->second->getSDO("Min_pos", min_pos);
	it->second->getSDO("Max_pos", max_pos);
	it->second->getSDO("link_pos", start_pos_rId[rId]);
	
	// set home to mid pos
	home_rId[rId] = MID_POS(min_pos,max_pos);
	//home_rId[rId] = DEG2RAD( homePos[rId-1] );
	while ( ! it->second->move_to(home_rId[rId], 0.002) ) {
            osal_usleep(1000);    
        }
    }
}

void EC_boards_walk::th_init(void*)
{
    // init Ec_Boards_ctrl
    if ( Ec_Boards_ctrl::init() != iit::ecat::advr::EC_BOARD_OK) {
	throw "something wrong";
    }

    InXddp::init("EC_board_input");
    
    // get Robot_Id map 
    rid2pos = get_Rid2PosMap();
    
    // initialize members class
    leftFoot = slave_as_FT(rid2pos[iit::ecat::advr::coman::LL_FT]);
    assert(leftFoot);
    rightFoot = slave_as_FT(rid2pos[iit::ecat::advr::coman::RL_FT]);
    assert(rightFoot);
    // get low_power motors
    get_esc_map_bytype(iit::ecat::advr::LO_PWR_DC_MC, motors);
     
    homing();
    
    if ( set_operative() <= 0 ) {
	throw "something else wrong";
    }
    
    start_time = get_time_ns();
    tNow, tPre = start_time;
}

void EC_boards_walk::th_loop(void*)
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
int EC_boards_walk::user_input(C &user_cmd) {
    
    static int	bytes_cnt;
    int		bytes;

    if ( (bytes=xddp_read(user_cmd)) <= 0 ) {
	return bytes;
    }
    
    bytes_cnt += bytes;
    DPRINTF(">> %d %d\n",bytes, bytes_cnt);
    //DPRINTF(">> %d\n",cmd.value);
   
    return bytes;
}

int EC_boards_walk::user_loop(void) {

    input_t what;
    user_input(what);
    
#if DEMO_TYPE==2
    user_loop_walk();
#elif DEMO_TYPE==1
    user_loop_test_joint();
#else
    #error "DEMO_TYPE not defined !!"
#endif
    
}

int EC_boards_walk::user_loop_walk(void) {

    static double   	RTtime;
    static float    	FTSensor[12];
    static char		buff[256];
    static int		rtControl_pos[32];
    
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::LpESC::pdo_rx_t motor_pdo_rx;
    iit::ecat::advr::Ft6ESC::pdo_rx_t ft6_pdo_rx;
    
    ft6_pdo_rx = rightFoot->getRxPDO();
    ft6_pdo_rx.sprint(buff, sizeof(buff));
    //DPRINTF("Right %s", buff);
    FTSensor[0] = ft6_pdo_rx.force_X * 0.001;
    FTSensor[1] = ft6_pdo_rx.force_Y * 0.001;
    FTSensor[2] = ft6_pdo_rx.force_Z * 0.001;
    FTSensor[3] = ft6_pdo_rx.torque_X * 0.001;
    FTSensor[4] = ft6_pdo_rx.torque_Y * 0.001;
    FTSensor[5] = ft6_pdo_rx.torque_Z * 0.001;
    
    ft6_pdo_rx = leftFoot->getRxPDO();
    ft6_pdo_rx.sprint(buff, sizeof(buff));
    //DPRINTF("Left %s", buff);
    FTSensor[6] = ft6_pdo_rx.force_X * 0.001;
    FTSensor[7] = ft6_pdo_rx.force_Y * 0.001;
    FTSensor[8] = ft6_pdo_rx.force_Z * 0.001;
    FTSensor[9] = ft6_pdo_rx.torque_X * 0.001;
    FTSensor[10] = ft6_pdo_rx.torque_Y * 0.001;
    FTSensor[11] = ft6_pdo_rx.torque_Z * 0.001;
    
    RTtime = (get_time_ns()-start_time)/1e9;

    //////////////// walking pattern ////////////////////////////
    RTControl(RTtime, FTSensor, homePos, homePos.size(), rtControl_pos);
    ////////////////////////////////////////////////////////////
    
    //set_position(_pos, sizeof(_pos));
    for ( auto it = motors.begin(); it != motors.end(); it++ ) {
	moto =  it->second;
	motor_pdo_rx = moto->getRxPDO();
	// pos_ref_fb is the previous reference
	moto->set_posRef(motor_pdo_rx.pos_ref_fb);
	// !! FIXME use rtControl_pos value
	// !! NOTE set_posRef use rad
	//moto->set_posRef(rtControl_pos[]);
    }
    
    return 0;
}

int EC_boards_walk::user_loop_test_joint(void) {

    static int		rtControl_pos[32];
    static double	RTtime;

    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::LpESC::pdo_rx_t motor_pdo_rx;
    
    RTtime = (get_time_ns()-start_time)/1e9;

    if (RTtime > 3 ) {

        test_joint(homePos, homePos.size(), 0.05, rtControl_pos);

        //set_position(_pos, sizeof(_pos));
	for ( auto it = motors.begin(); it != motors.end(); it++ ) {
	    moto =  it->second;
	    motor_pdo_rx = moto->getRxPDO();
	    // pos_ref_fb is the previous reference
	    moto->set_posRef(motor_pdo_rx.pos_ref_fb);
	    // !! FIXME use rtControl_pos value
	    // !! NOTE set_posRef use rad
	    //moto->set_posRef(rtControl_pos[]);
	}
    }

    return 0;
}
