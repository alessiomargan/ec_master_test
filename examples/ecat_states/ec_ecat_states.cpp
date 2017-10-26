#include <ec_ecat_states.h>
#include <iit/ecat/advr/lxm32i_esc.h>

using namespace iit::ecat::advr;

Ec_Ecat_states::Ec_Ecat_states ( const char* config_yaml ) : Ec_Thread_Boards_base ( config_yaml ) {

    name = "EC_ecat_states";
    // non periodic
    period.period = {0,1};

#ifdef __COBALT__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max ( schedpolicy ) - 10 ;
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    // open pipe ... xeno xddp or fifo
    inXddp.init ( "EC_board_input" );

}

Ec_Ecat_states::~Ec_Ecat_states() {
    std::cout << "~" << typeid ( this ).name() << std::endl;
}

void Ec_Ecat_states::init_preOP ( void ) {

#if 0
    DPRINTF("+++++++++++++++++++++\n");
    Ec_Boards_ctrl::shutdown(false);
    // wait boards boot up
    sleep(1);
    // init Ec_Boards_ctrl
    if ( Ec_Boards_ctrl::init() != iit::ecat::advr::EC_BOARD_OK ) {
        throw "something wrong";
    }
#endif

    std::vector<int> to_control = std::initializer_list<int> {
        1,
        2,
        3,
    };

    get_esc_map_byclass ( lxm32i, to_control);
    DPRINTF ( "found %lu motors\n", lxm32i.size() );
    
}

void Ec_Ecat_states::init_OP ( void ) {

    //LXM32iESC * moto = slave_as<LXM32iESC>(1); 
    //assert ( moto );
    //moto->test_motor();
    
    //throw std::runtime_error("... esco !!");

    
}

template<class C>
int Ec_Ecat_states::user_input ( C &user_cmd ) {

    static int  bytes_cnt;
    int         bytes;

    if ( ( bytes = inXddp.xddp_read ( user_cmd ) ) <= 0 ) {
        return bytes;
    }

    bytes_cnt += bytes;
    //DPRINTF ( ">> %d %d\n",bytes, bytes_cnt );

    return bytes;
}

int Ec_Ecat_states::user_loop ( void ) {

    LXM32iESC * moto;
    int         slave_pos;
    char        cmd = 0;
    
    user_input( cmd );
        
    //moto = slave_as<LXM32iESC>(2);
    //moto->user_loop(cmd);
    
    for ( auto const& item : lxm32i ) {
        slave_pos = item.first;
        moto =  item.second;
        moto->user_loop(cmd);
    }
}


// kate: indent-mode cstyle; indent-width 4; replace-tabs on