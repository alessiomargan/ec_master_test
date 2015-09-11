#include <ec_boards_basic.h>

#define MID_POS(m,M)    (m+(M-m)/2)

EC_boards_basic::EC_boards_basic(const char* config_yaml) : Ec_Boards_ctrl(config_yaml), InXddp()
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
    
}

EC_boards_basic::~EC_boards_basic()
{
    print_stat(s_loop);
}

void EC_boards_basic::homing(void) {

}

void EC_boards_basic::th_init(void*)
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

void EC_boards_basic::th_loop(void*)
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
int EC_boards_basic::user_input(C &user_cmd) {
    
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

int EC_boards_basic::user_loop(void) {

    int what;
    user_input(what);
}
