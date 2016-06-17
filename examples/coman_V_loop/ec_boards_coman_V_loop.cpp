#include <ec_boards_coman_V_loop.h>
#include <iit/advr/coman_robot_id.h>

#define MID_POS(m,M)    (m+(M-m)/2)

Ec_Boards_V_loop::Ec_Boards_V_loop ( const char* config_yaml ) : Ec_Thread_Boards_base ( config_yaml ) {

    name = "Ec_Boards_V_loop";
    // non periodic
    period.period = {0,1};

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

Ec_Boards_V_loop::~Ec_Boards_V_loop() {

}

void Ec_Boards_V_loop::init_preOP ( void ) {

    iit::ecat::advr::Motor * moto;
    
    int slave_pos;
    float min_pos, max_pos;
    int16_t torque;

    const YAML::Node config = get_config_YAML_Node();

    std::vector<int> pos_rid = iit::ecat::advr::coman::robot_mcs_ids;
    std::vector<int> no_control = std::initializer_list<int> {
        iit::ecat::advr::coman::RA_HA,
        iit::ecat::advr::coman::LA_HA,
    };
    std::vector<int> volt_rid = std::initializer_list<int> {
        iit::ecat::advr::coman::RL_H_P,
        iit::ecat::advr::coman::LL_H_P,
        iit::ecat::advr::coman::RA_SH_1,
        iit::ecat::advr::coman::LA_SH_1
    };
        
    remove_rids_intersection(pos_rid, no_control);
    // remove from pos_rid what will be controlled in voltage
    remove_rids_intersection(pos_rid, volt_rid);

    get_esc_map_byclass ( motors_ctrl_pos,  pos_rid );
    for ( auto const& item : motors_ctrl_pos ) {
        slave_pos = item.first;
        moto = item.second;
        // start controller :
        moto->start ( CTRL_SET_POS_MODE);
    }

    get_esc_map_byclass ( motors_ctrl_volt,  volt_rid );
    for ( auto const& item : motors_ctrl_volt ) {
        slave_pos = item.first;
        moto = item.second;
        // start controller :
        moto->start ( CTRL_SET_VOLT_MODE);
    }

    //DPRINTF ( ">>> wait xddp terminal ....\n" );
    //char c; while ( termInXddp.xddp_read ( c ) <= 0 ) { osal_usleep(100); }
}

void Ec_Boards_V_loop::init_OP ( void ) {

}

template<class C>
int Ec_Boards_V_loop::user_input ( C &user_cmd ) {

    static int  bytes_cnt;
    int     bytes;
    char    cmd;

    if ( ( bytes = inXddp.xddp_read ( cmd ) ) <= 0 ) {
        return bytes;
    }

    bytes_cnt += bytes;
    DPRINTF ( ">> %d %d\n",bytes, bytes_cnt );

    return bytes;
}

int Ec_Boards_V_loop::user_loop ( void ) {

    int what;
    user_input ( what );
  
    //////////////////////////////////////////////////////
    // sine trajectory
    //////////////////////////////////////////////////////
    {
        static uint64_t start_time_loop;
        start_time_loop = start_time_loop ? start_time_loop : iit::ecat::get_time_ns();
        uint64_t tNow = iit::ecat::get_time_ns();
        float dt = ( tNow - start_time_loop ) / 1e9;
        // !!!!! if too fast adjust this
        float freq = 0.25;
        float mV;
        iit::ecat::advr::Motor * moto;
        int slave_pos;
        for ( auto const& item : motors_ctrl_volt ) {
            slave_pos = item.first;
            moto = item.second;
            mV = 2000; 
            moto->set_posRef ( mV * sinf ( 2*M_PI*freq*dt ) );
        }
    }

}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
