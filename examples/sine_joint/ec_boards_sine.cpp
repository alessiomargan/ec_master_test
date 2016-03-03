#include <ec_boards_sine.h>

#define MID_POS(m,M)    (m+(M-m)/2)

Ec_Boards_sine::Ec_Boards_sine ( const char* config_yaml ) : Ec_Thread_Boards_base ( config_yaml ) {

    name = "EC_boards_basic";
    // do not go above ....
    period.period = {0,500};

#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max ( schedpolicy );
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    // open pipe ... xeno xddp or fifo
    inXddp.init ( "EC_board_input" );

}

Ec_Boards_sine::~Ec_Boards_sine() {

}

void Ec_Boards_sine::init_preOP ( void ) {

    iit::ecat::advr::Motor * moto;
    int slave_pos;
    float min_pos, max_pos;

    // fill motors map
    DPRINTF ( "found %lu <Motor> instance\n", motors.size() );

    for ( auto const& item : motors ) {
        slave_pos = item.first;
        moto = item.second;
        moto->readSDO ( "Min_pos", min_pos );
        moto->readSDO ( "Max_pos", max_pos );
        moto->readSDO ( "link_pos", start_pos[slave_pos] );
        // set home to mid pos
        home[slave_pos] = MID_POS ( min_pos,max_pos );
        // set pos to current position
        moto->set_posRef ( start_pos[slave_pos] );
        // start controller
        moto->start ( CTRL_SET_MIX_POS_MODE );
    }

    for ( auto const& item : motors ) {
        slave_pos = item.first;
        moto = item.second;
        while ( ! moto->move_to ( home[slave_pos], 0.005 ) ) {
            osal_usleep ( 2000 );
        }
    }


}

void Ec_Boards_sine::init_OP ( void ) {

    iit::ecat::advr::Motor * moto;
    int slave_pos;

//     for ( auto const& item : motors ) {
// 	slave_pos = item.first;
// 	moto = item.second;
// 	moto->set_posRef(home[slave_pos]);
//     }

}

template<class C>
int Ec_Boards_sine::user_input ( C &user_cmd ) {

    static int	bytes_cnt;
    int		bytes;
    input_t	cmd;

    if ( ( bytes = inXddp.xddp_read ( cmd ) ) <= 0 ) {
        return bytes;
    }

    bytes_cnt += bytes;
    DPRINTF ( ">> %d %d\n",bytes, bytes_cnt );
    //DPRINTF(">> %d\n",cmd.value);

    return bytes;
}

int Ec_Boards_sine::user_loop ( void ) {

    int what;
    user_input ( what );

    //////////////////////////////////////////////////////
    // sine trajectory
    //////////////////////////////////////////////////////
    static uint64_t start_time_sine;
    start_time_sine = start_time_sine ? start_time_sine : iit::ecat::get_time_ns();
    uint64_t tNow = iit::ecat::get_time_ns();
    float dt = ( tNow - start_time ) / 1e9;
    // !!!!! if too fast adjust this
    float freq = 0.01;
    float A;
    iit::ecat::advr::Motor * moto;
    int slave_pos;
    for ( auto const& item : motors ) {
        slave_pos = item.first;
        moto = item.second;
        A = home[slave_pos] - 0.1;
        moto->set_posRef ( home[slave_pos] + A * sinf ( 2*M_PI*freq*dt ) );
    }
    //////////////////////////////////////////////////////


}


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
