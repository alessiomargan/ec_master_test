#include <ec_boards_sine.h>
#include <iit/advr/coman_robot_id.h>
#include <iit/advr/walkman_robot_id.h>
#include <iit/advr/centauro_robot_id.h>

#define MID_POS(m,M)    (m+(M-m)/2)

using namespace iit::ecat::advr;

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
    float min_pos, max_pos, link_pos, motor_pos;

    std::vector<int> test_rid = std::initializer_list<int> {
    
        103
        
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
        moto->readSDO ( "motor_pos", motor_pos );
        moto->readSDO ( "link_pos", link_pos );
        start_pos[slave_pos] = motor_pos; 
        
        DPRINTF ( ">> Joint_id %d motor %f link %f start %f home %f\n", pos2Rid ( slave_pos ), motor_pos, link_pos, start_pos[slave_pos], home[slave_pos]);
        
        // set home to mid pos
        home[slave_pos] = MID_POS ( min_pos,max_pos );
        //home[slave_pos] = start_pos[slave_pos];
        //home[slave_pos] = DEG2RAD ( centauro::robot_ids_home_pos_deg[pos2Rid(slave_pos)] );
        
        //////////////////////////////////////////////////////////////////////////
        // start controller :
        // - read actual joint position and set as pos_ref  
        //assert ( moto->start ( CTRL_SET_MIX_POS_MODE ) == EC_BOARD_OK );
        //assert ( moto->start ( CTRL_SET_POS_MODE ) == EC_BOARD_OK );
        assert ( moto->start ( CTRL_SET_CURR_MODE ) == EC_BOARD_OK );
        //moto->start ( CTRL_SET_POS_LINK_MODE );
    }

    DPRINTF ( ">>> wait xddp terminal ....\n" );
    DPRINTF ( ">>> from another terminal run ec_master_test/scripts/xddp_term.py\n" );
    char c; while ( termInXddp.xddp_read ( c ) <= 0 ) { osal_usleep(100); }  

    
//     for ( auto const& item : motors_to_start ) {
//         slave_pos = item.first;
//         moto = item.second;
//         while ( ! moto->move_to ( home[slave_pos], 0.002 ) ) {
//             osal_usleep ( 1000 );
//         }
//     }

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
        float freq = 1;
        float A;
        iit::ecat::advr::Motor * moto;
        int slave_pos;
        for ( auto const& item : motors ) {
            slave_pos = item.first;
            moto = item.second;
            A = home[slave_pos] - 1;
            //moto->set_posRef ( home[slave_pos] + A * sinf ( 2*M_PI*freq*dt ) );
            //moto->set_torRef ( 2 * cosf ( 2*M_PI*freq*dt ) );
        }
    }
    //////////////////////////////////////////////////////
    //
    //////////////////////////////////////////////////////
#if 0
    {
        auto moto = slave_as<CentAcESC>(1);
        moto->set_ivRef(0);
    }
#endif
    
}


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
