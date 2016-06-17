#include <ec_boards_sine.h>
#include <iit/advr/coman_robot_id.h>

#define MID_POS(m,M)    (m+(M-m)/2)

Ec_Boards_sine::Ec_Boards_sine ( const char* config_yaml ) : Ec_Thread_Boards_base ( config_yaml ) {

    name = "EC_boards_sine";
    // do not go above ....
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

Ec_Boards_sine::~Ec_Boards_sine() {
    std::cout << "~" << typeid ( this ).name() << std::endl;
}

void Ec_Boards_sine::init_preOP ( void ) {

    std::map<int, iit::ecat::advr::Motor*>  motors_to_start;
    iit::ecat::advr::Motor * moto;
    int slave_pos;
    float min_pos, max_pos;

    std::vector<int> test_rid = std::initializer_list<int> {

        // waist
//         iit::ecat::advr::coman::WAIST_Y,
//         iit::ecat::advr::coman::WAIST_P,
//         iit::ecat::advr::coman::WAIST_R,
        // right leg
//         iit::ecat::advr::coman::RL_H_P,
//         iit::ecat::advr::coman::RL_H_R,
//         iit::ecat::advr::coman::RL_H_Y,
//         iit::ecat::advr::coman::RL_K,
//         iit::ecat::advr::coman::RL_A_P,
//         iit::ecat::advr::coman::RL_A_R,
//         iit::ecat::advr::coman::RL_FT,
        // left leg
//         iit::ecat::advr::coman::LL_H_P,
//         iit::ecat::advr::coman::LL_H_R,
//         iit::ecat::advr::coman::LL_H_Y,
//         iit::ecat::advr::coman::LL_K,
//         iit::ecat::advr::coman::LL_A_P,
//         iit::ecat::advr::coman::LL_A_R,
//         iit::ecat::advr::coman::LL_FT,
        // right arm
//         iit::ecat::advr::coman::RA_SH_1,
//         iit::ecat::advr::coman::RA_SH_2,
//         iit::ecat::advr::coman::RA_SH_3,
//         iit::ecat::advr::coman::RA_EL,
//         iit::ecat::advr::coman::RA_WR_1,
        iit::ecat::advr::coman::RA_WR_2,
        iit::ecat::advr::coman::RA_WR_3,
        iit::ecat::advr::coman::RA_FT,
        iit::ecat::advr::coman::RA_HA,
        // left arm
//         iit::ecat::advr::coman::LA_SH_1,
//         iit::ecat::advr::coman::LA_SH_2,
//         iit::ecat::advr::coman::LA_SH_3,
//         iit::ecat::advr::coman::LA_EL,
//         iit::ecat::advr::coman::LA_WR_1,
        iit::ecat::advr::coman::LA_WR_2,
        iit::ecat::advr::coman::LA_WR_3,
        iit::ecat::advr::coman::LA_FT,
        iit::ecat::advr::coman::LA_HA,
    };

    // fill motors map
    DPRINTF ( "found %lu <Motor> instance\n", motors.size() );

    // !!!
    get_esc_map_byclass ( motors_to_start,  test_rid );

    for ( auto const& item : motors_to_start ) {
        slave_pos = item.first;
        moto = item.second;
        moto->readSDO ( "Min_pos", min_pos );
        moto->readSDO ( "Max_pos", max_pos );
        moto->readSDO ( "link_pos", start_pos[slave_pos] );
        // set home to mid pos
        home[slave_pos] = MID_POS ( min_pos,max_pos );
        
        //////////////////////////////////////////////////////////////////////////
        // start controller :
        // - read actual joint position and set as pos_ref  
        //moto->start ( CTRL_SET_MIX_POS_MODE );
        moto->start ( CTRL_SET_POS_MODE );
    }

    for ( auto const& item : motors_to_start ) {
        slave_pos = item.first;
        moto = item.second;
        while ( ! moto->move_to ( home[slave_pos], 0.005 ) ) {
            osal_usleep ( 2000 );
        }
    }

}

void Ec_Boards_sine::init_OP ( void ) {


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
    {
        static uint64_t start_time_sine;
        start_time_sine = start_time_sine ? start_time_sine : iit::ecat::get_time_ns();
        uint64_t tNow = iit::ecat::get_time_ns();
        float dt = ( tNow - start_time_sine ) / 1e9;
        // !!!!! if too fast adjust this
        float freq = 0.25;
        float A;
        iit::ecat::advr::Motor * moto;
        int slave_pos;
        for ( auto const& item : motors ) {
            slave_pos = item.first;
            moto = item.second;
            A = home[slave_pos] - 0.2;
            moto->set_posRef ( home[slave_pos] + A * sinf ( 2*M_PI*freq*dt ) );
        }
    }
    //////////////////////////////////////////////////////

}


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
