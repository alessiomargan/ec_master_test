#include <ec_boards_impedance_test.h>

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

using namespace iit::ecat::advr;

typedef struct js_event	js_input_t;
typedef spnav_event	spnav_input_t;


static const std::vector<double> Xt_2s = std::initializer_list<double> { 0, 2 };
static const std::vector<double> Xt_3s = std::initializer_list<double> { 0, 3 };
static const std::vector<double> Xt_4s = std::initializer_list<double> { 0, 4 };
static const std::vector<double> Xt_5s = std::initializer_list<double> { 0, 5 };
static const std::vector<double> Xt_9s = std::initializer_list<double> { 0, 9 };

static const std::map<int, float> robot_ids_home_pos_deg = {

    {101,   0.0},
    {102,   0.0},
};

const std::map<int, float> robot_ids_test_pos_deg = {

    {101,   20.0},
    {102,   20.0},

};
EC_boards_impedance_test::EC_boards_impedance_test(const char* config_yaml) :
    Ec_Thread_Boards_base ( config_yaml ) {

    name = "Impedance_test";
    // not periodic
    period.period = {0,1};

#ifdef __COBALT__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max ( schedpolicy )-10;
    stacksize = ECAT_PTHREAD_STACK_SIZE;

    // open pipe ... xeno xddp or fifo
    jsInXddp.init ( "EC_board_js_input" );
}

EC_boards_impedance_test:: ~EC_boards_impedance_test() {

}

/*
 *
 */
void EC_boards_impedance_test::init_preOP ( void ) {

    Motor * moto;
    int slave_pos, motor_start;
    float min_pos, max_pos, pos_ref_fb;


    std::vector<double> vPos;
    std::vector<double> vVel;
    std::vector<double> vTor;
    std::vector<double> Xs;

    ///////////////////////////////////////////////////////////////////////
    // prepare trajectory for ALL motors
    for ( auto const& item : motors ) {
    
        slave_pos = item.first;
        moto = item.second;
        moto->readSDO ( "Min_pos", min_pos );
        moto->readSDO ( "Max_pos", max_pos );
        moto->readSDO ( "link_pos", start_pos[slave_pos] );
        // home
        home[slave_pos] = DEG2RAD ( robot_ids_home_pos_deg.at(pos2Rid(slave_pos)) );
        test_pos[slave_pos] = DEG2RAD ( robot_ids_test_pos_deg.at(pos2Rid(slave_pos)) );
        
        DPRINTF ( "Joint_id %d start %f home %f test_pos %f\n",
                  pos2Rid ( slave_pos ), start_pos[slave_pos], home[slave_pos], test_pos[slave_pos] );

        vPos = std::initializer_list<double> { start_pos[slave_pos], home[slave_pos] };
        vVel = std::initializer_list<double> { 0, 0 };
        vTor = std::initializer_list<double> { 0, 0 };
        trj_start2home[slave_pos] = { 
            std::make_shared<advr::Smoother_trajectory> ( Xt_5s, vPos ),
            std::make_shared<advr::Smoother_trajectory> ( Xt_5s, vVel ),
            std::make_shared<advr::Smoother_trajectory> ( Xt_5s, vTor )
        };

        vPos = std::initializer_list<double> { home[slave_pos], test_pos[slave_pos], -test_pos[slave_pos], home[slave_pos] };
        vVel = std::initializer_list<double> { 0, 0,  0, 0 };
        vTor = std::initializer_list<double> { 0, 5, -4, 0 };
        Xs = std::initializer_list<double>   { 0, 3,  6, 9 };
        trj_home2test_pos2home[slave_pos] = { 
            std::make_shared<advr::Smoother_trajectory> ( Xs, vPos ),
            std::make_shared<advr::Smoother_trajectory> ( Xs, vVel ),
            std::make_shared<advr::Smoother_trajectory> ( Xs, vTor )
        };

    }
    
    std::vector<int> pos_ctrl_ids = std::initializer_list<int> {
        101,102
    };

    std::vector<int> tor_ctrl_ids = std::initializer_list<int> {
    };
    
    std::vector<int> no_control = std::initializer_list<int> {
    };
    
    remove_rids_intersection(pos_ctrl_ids, no_control);
    get_esc_map_byclass ( motors2ctrl,  pos_ctrl_ids );

    for ( auto const& item : motors2ctrl ) {
        slave_pos = item.first;
        moto = item.second;
        //////////////////////////////////////////////////////////////////////////
        // start controller :
        // - read actual joint position and set as pos_ref
        //DPRINTF ( ">>> START %d wait xddp terminal ....\n", moto->get_robot_id() );
        //char c; while ( termInXddp.xddp_read ( c ) <= 0 ) { osal_usleep(100); }
        
        if (moto->get_ESC_type() == LO_PWR_DC_MC ) {
            motor_start = moto->start ( CTRL_SET_POS_MODE );
        } else if ( moto->get_ESC_type() == HI_PWR_AC_MC ) {
            motor_start = moto->start ( CTRL_SET_POS_MODE );
            //motor_start = moto->start ( CTRL_SET_IMPED_MODE );
        } else if ( moto->get_ESC_type() == HI_PWR_DC_MC) {
            motor_start = moto->start ( CTRL_SET_IMPED_MODE );
        } else {
            
        }

        assert( motor_start == EC_BOARD_OK );
        
        //while ( ! moto->move_to(home[slave_pos], 0.005) ) { osal_usleep(100);  }
    }

    motors2move = motors2ctrl;
    
    DPRINTF ( ">>> motors2ctrl %d motors2move %d\n", motors2ctrl.size(), motors2move.size() );
    DPRINTF ( ">>> wait xddp terminal ....\n" );
    char c; while ( termInXddp.xddp_read ( c ) <= 0 ) { osal_usleep(100); }  
    
    if ( motors2move.size() > 0 ) {
        imp_trj_queue.push ( &trj_start2home );
    }
}


void EC_boards_impedance_test::init_OP ( void ) {

    if ( ! imp_trj_queue.empty() ) {
        auto running_trj = imp_trj_queue.front();
        advr::reset_imp_trj ( *running_trj );
    }
    
    DPRINTF ( "End Init_OP\n" );
    
}

int EC_boards_impedance_test::user_loop ( void ) {

    static float ds;
    static uint64_t count;
    const float spline_error = 0.07;
    
    
//     if ( ( count++ ) % 1000 == 0 ) {
//         DPRINTF ( "alive %d\n", count/1000 );
//     }

    ///////////////////////////////////////////////////////
    //
    if ( xddp_input ( ds ) > 0 ) {
        DPRINTF ( ">> %f\n", ds );
    }


    if ( ! imp_trj_queue.empty() ) {

        auto running_trj = imp_trj_queue.front();
        if ( running_trj ) {
            // !@#%@$#%^^# ... tune error
            if ( set_impedance_refs ( motors2move, *running_trj, spline_error, false) ) {
                // running spline has finish !!
                imp_trj_queue.pop();
                if ( ! imp_trj_queue.empty() ) {
                    running_trj = imp_trj_queue.front();
                    advr::reset_imp_trj ( *running_trj );
                }
            }
        } else {
            DPRINTF ( "Error NULL running spline ... pop it\n" );
            imp_trj_queue.pop();
        }
    } else { // trj_queue is empty

        if ( motors2move.size() > 0 ) {
            // add splines ....
            imp_trj_queue.push ( &trj_home2test_pos2home );
            // !!! since queue was empty reset the first spline
            auto running_trj = imp_trj_queue.front();
            advr::reset_imp_trj ( *running_trj );
        }
    
    }

}


template<class C>
int EC_boards_impedance_test::xddp_input ( C &user_cmd ) {

    static int	bytes_cnt;
    int		bytes = 0;
    
//     char c;
//     if ( ( bytes = termInXddp.xddp_read ( c ) ) > 0 ) {
//         switch( c ) {
//             default :
//                 while ( ! trj_queue.empty() ) {
//                         trj_queue.pop();
//                     }
//                     get_trj_for_end_points ( trj_any2home, home, 2.0 );
//                     if ( running_trj ) {
//                         smooth_splines_trj ( trj_any2home, *running_trj, 1.0 );
//                     }
//                     advr::reset_trj ( trj_any2home );
//                     trj_queue.push ( &trj_any2home );
//                     DPRINTF ( "ANY2HOME ....\n" );
//             break;
//                 
//         }
//     }

    js_input_t  js_cmd;
    if ( ( bytes = jsInXddp.xddp_read ( js_cmd ) ) > 0 ) {

        //user_cmd = process_js_input(js_cmd);
        switch ( js_cmd.type & ~JS_EVENT_INIT ) {
        case JS_EVENT_AXIS:
            switch ( js_cmd.number ) {
            case 0 :
                // [-1.0 .. 1.0] / 500 ==> [-0.002 .. 0.002] rads
                user_cmd = ( ( float ) js_cmd.value/ ( 32767.0 ) ) / 500 ;
                break;
            case 2 :
                // [-1.0 .. 1.0] / 250 ==> [-0.004 .. 0.004] rads
                user_cmd = ( ( float ) js_cmd.value/ ( 32767.0 ) ) / 250 ;
                break;
            default:
                break;
            }
            break;

        case JS_EVENT_BUTTON :
            switch ( js_cmd.number ) {
            case 0 :
                break;
            case 1 :
                break;
            case 2 :
                break;
            case 9 :
                break;
            case 4 :
            case 5 :
            case 6 :
            case 7 :
                break;
            default:
                DPRINTF ( "Not handle : cmd %d value %d\n", js_cmd.number, js_cmd.value );
                break;
            }
            break;

        default:
            break;

        }
    }
    bytes_cnt += bytes;
    //DPRINTF(">> %d %d\n",bytes, bytes_cnt);

    return bytes;
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
