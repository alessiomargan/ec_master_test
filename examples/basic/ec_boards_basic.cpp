#include <ec_boards_basic.h>

#define MID_POS(m,M)    (m+(M-m)/2)

Ec_Boards_basic::Ec_Boards_basic ( const char* config_yaml ) : Ec_Thread_Boards_base ( config_yaml ) {

    name = "EC_boards_basic";
    // non periodic
    period.period = {0,1};

#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max ( schedpolicy ) - 10 ;
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    // open pipe ... xeno xddp or fifo
    inXddp.init ( "EC_board_input" );

}

Ec_Boards_basic::~Ec_Boards_basic() {
    std::cout << "~" << typeid ( this ).name() << std::endl;
}

void Ec_Boards_basic::init_preOP ( void ) {

}

void Ec_Boards_basic::init_OP ( void ) {

#if 0
    // get first motor ....
    iit::ecat::advr::CentAcESC * moto = slave_as<iit::ecat::advr::CentAcESC>(1); 
    if ( moto ) {
        moto->run_torque_calibration( );
    }
#endif

}

template<class C>
int Ec_Boards_basic::user_input ( C &user_cmd ) {

    static int  bytes_cnt;
    int         bytes;
    input_t     cmd;

    if ( ( bytes = inXddp.xddp_read ( cmd ) ) <= 0 ) {
        return bytes;
    }

    bytes_cnt += bytes;
    DPRINTF ( ">> %d %d\n",bytes, bytes_cnt );
    //DPRINTF(">> %d\n",cmd.value);

    return bytes;
}

int Ec_Boards_basic::user_loop ( void ) {

    int what;
    user_input ( what );
   
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on