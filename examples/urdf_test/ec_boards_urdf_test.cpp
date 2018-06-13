#include <ec_boards_urdf_test.h>
#include <iit/advr/centauro_robot_id.h>

#include <linux/joystick.h>

#include <spnav_config.h>
#ifdef USE_X11
#undef USE_X11
#endif
#include <spnav.h>

#ifdef USE_LPMS_IMU
#include <ImuData.h>
#endif

#define MID_POS(m,M)    (m+(M-m)/2)


typedef struct js_event	js_input_t;
typedef spnav_event	spnav_input_t;


using namespace iit::ecat::advr;

EC_boards_urdf::EC_boards_urdf ( const char* config_yaml ) :
    Ec_Thread_Boards_base ( config_yaml ), URDF_adapter( ) {

    name = "urd_test";
    // not periodic
    period.period = {0,1};

#ifdef __COBALT__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_RR;
#endif
    priority = sched_get_priority_max ( schedpolicy );
    stacksize = ECAT_PTHREAD_STACK_SIZE;

    // open pipe ... xeno xddp or fifo
    jsInXddp.init ( "EC_board_js_input" );
    navInXddp.init ( "EC_board_nav_input" );

    init_urdf ( get_config_YAML_Node() ["ec_boards_base"]["urdf_config_file"].as<std::string>() );

}

EC_boards_urdf::~EC_boards_urdf() {

}

/*
 *
 */
void EC_boards_urdf::init_preOP ( void ) {

    Motor * moto;
    int slave_pos;
    float min_pos, max_pos, link_pos, motor_pos;

    //std::vector<int> pos_rid = centauro::robot_mcs_ids;
    std::vector<int> pos_rid = get_urdf_rId();
    //std::vector<int> pos_rid = std::initializer_list<int> {   };
    std::vector<int> no_control = std::initializer_list<int> {   };

    remove_rids_intersection ( pos_rid, no_control );

    // !!!
    get_esc_map_byclass ( motors_to_start,  pos_rid );
    DPRINTF ( "found %lu <Motor> to_start >> %lu in urdf model\n", motors_to_start.size(), pos_rid.size() );

    for ( auto const& item : motors_to_start ) {

        slave_pos = item.first;
        moto = item.second;
        moto->readSDO ( "Min_pos", min_pos );
        moto->readSDO ( "Max_pos", max_pos );
        assert ( iit::ecat::EC_WRP_OK == moto->readSDO ( "motor_pos", motor_pos ) );
        assert ( iit::ecat::EC_WRP_OK == moto->readSDO ( "link_pos", link_pos ) );
        start_pos[slave_pos] = motor_pos;

        //
        Q ( rid2Urdf ( pos2Rid ( slave_pos ) ) ) = DEG2RAD ( centauro::robot_ids_home_pos_deg.at ( pos2Rid ( slave_pos ) ) );

        // home
        home[slave_pos] = DEG2RAD ( centauro::robot_ids_home_pos_deg.at ( pos2Rid ( slave_pos ) ) );

        auto Xs = std::initializer_list<double> { 0, 5 };
        auto Ys =  std::initializer_list<double> { start_pos[slave_pos], home[slave_pos] };
        trj_map["start@home"][slave_pos] = std::make_shared<advr::Smoother_trajectory> ( Xs, Ys );

        //////////////////////////////////////////////////////////////////////////
        // start controller :
        assert ( moto->start ( ) == EC_BOARD_OK );

    }

    DPRINTF ( ">>> wait xddp terminal ....\n" );
    DPRINTF ( ">>> from another terminal run ec_master_test/scripts/xddp_term.py\n" );
    char c;
    while ( termInXddp.xddp_read ( c ) <= 0 ) {
        osal_usleep ( 100 );
    }

    trj_queue.clear();
    //trj_queue.push_back ( trj_map.at("start@home") );

}


void EC_boards_urdf::init_OP ( void ) {


    try {
        advr::reset_trj ( trj_queue.at ( 0 ) );
    } catch ( const std::out_of_range &e ) {
        //throw std::runtime_error("Oh my gosh  ... trj_queue is empty !");
    }

    DPRINTF ( "End Init_OP\n" );

}

int EC_boards_urdf::user_loop ( void ) {

    static float ds;
    static uint64_t count;
    float spline_error = 0.07;

    if ( ( count++ ) % 1000 == 0 ) {
        DPRINTF ( "alive %ld\n", count/1000 );
    }

    ///////////////////////////////////////////////////////
    //
    if ( xddp_input ( ds ) > 0 ) {
        DPRINTF ( ">> %f\n", ds );
    }

    ///////////////////////////////////////////////////////
    //
    if ( ! trj_queue.empty() ) {
        if ( go_there ( motors_to_start, trj_queue.at ( 0 ), spline_error, false ) ) {
            // running trj has finish ... remove from queue  !!
            DPRINTF ( "running trj has finish ... remove from queue !!\n" );
            trj_queue.pop_front();
            try {
                advr::reset_trj ( trj_queue.at ( 0 ) );
            } catch ( const std::out_of_range &e ) {
                // add trajectory ....
            }
        }
    } else { // trj_queue is empty

    }

}

template<class C>
int EC_boards_urdf::xddp_input ( C &user_cmd ) {

    static int  bytes_cnt;
    int         bytes = 0;


    return bytes;
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
