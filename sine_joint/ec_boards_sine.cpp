#include <ec_boards_sine.h>

#define MID_POS(m,M)    (m+(M-m)/2)

Ec_Boards_sine::Ec_Boards_sine(const char* config_yaml) : Ec_Thread_Boards_base(config_yaml), InXddp()
{

    name = "EC_boards_basic";
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
    InXddp::init("EC_board_input");

}

Ec_Boards_sine::~Ec_Boards_sine() {
    
}

void Ec_Boards_sine::init_preOP(void) {

    iit::ecat::advr::Motor * moto;
    int slave_pos;
    float min_pos, max_pos;
    
    // fill motors map
    get_esc_map_byclass(motors);
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
    
//     moto = motors[rid2pos[1]];
//     while ( ! moto->move_to(home[slave_pos], 0.005) ) {
//     	osal_usleep(2000);    
//     }

}

void Ec_Boards_sine::init_OP(void) {
  
}

template<class C>
int Ec_Boards_sine::user_input(C &user_cmd) {
    
    static int	bytes_cnt;
    int		bytes;
    input_t	cmd;

    if ( (bytes=xddp_read(cmd)) <= 0 ) {
	return bytes;
    }
    
    bytes_cnt += bytes;
    DPRINTF(">> %d %d\n",bytes, bytes_cnt);
    //DPRINTF(">> %d\n",cmd.value);
   
    return bytes;
}

int Ec_Boards_sine::user_loop(void) {

    int what;
    user_input(what);
    
    //////////////////////////////////////////////////////
    // sine trajectory
    //////////////////////////////////////////////////////
    static uint64_t start_time_sine;
    start_time_sine = start_time_sine ? start_time_sine : iit::ecat::get_time_ns();
    uint64_t time = iit::ecat::get_time_ns();
    float dt = (time - start_time) / 1e9;
    // !!!!! if too fast adjust this
    float freq = 0.05;
    float A;
    iit::ecat::advr::Motor * moto;
    int slave_pos;
    for ( auto it = motors.begin(); it != motors.end(); it++ ) {
	moto = it->second;
	slave_pos = it->first;
	A = home[slave_pos] - 0.2;
	moto->set_posRef(home[slave_pos] + A * sinf(2*M_PI*freq*dt));
    }
    //////////////////////////////////////////////////////
    
    
}


