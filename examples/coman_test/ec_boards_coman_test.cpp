#include <ec_boards_coman_test.h>
#include <iit/advr/coman_robot_id.h>

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


static const std::vector<double> Xt_2s = std::initializer_list<double> { 0, 2 };
static const std::vector<double> Xt_3s = std::initializer_list<double> { 0, 3 };
static const std::vector<double> Xt_4s = std::initializer_list<double> { 0, 4 };
static const std::vector<double> Xt_5s = std::initializer_list<double> { 0, 5 };
static const std::vector<double> Xt_9s = std::initializer_list<double> { 0, 9 };

using namespace iit::ecat::advr;

EC_boards_coman_test::EC_boards_coman_test ( const char* config_yaml ) :
    Ec_Thread_Boards_base ( config_yaml ) {

    name = "coman_test";
    // not periodic
    period.period = {0,1};

#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max ( schedpolicy )-10;
    stacksize = ECAT_PTHREAD_STACK_SIZE;

    // open pipe ... xeno xddp or fifo
    jsInXddp.init ( "EC_board_js_input" );
    navInXddp.init ( "EC_board_nav_input" );
    imuInXddp.init ( "Lpms_imu" );
}

EC_boards_coman_test::~EC_boards_coman_test() {

}

/*
 *
 */
void EC_boards_coman_test::init_preOP ( void ) {

    Motor * moto;
    int slave_pos;
    float min_pos, max_pos;

    std::vector<int> pos_rid = coman::robot_mcs_ids;
    std::vector<int> no_control = std::initializer_list<int> {
        coman::RA_HA,
        coman::LA_HA,
    };
        
    remove_rids_intersection(pos_rid, no_control);

    get_esc_map_byclass ( left_leg,  coman::robot_left_leg_ids );
    DPRINTF ( "found %lu <Motor> left_leg\n", left_leg.size() );

    get_esc_map_byclass ( left_arm,  coman::robot_left_arm_ids );
    DPRINTF ( "found %lu <Motor> left_arm\n", left_arm.size() );

    get_esc_map_byclass ( right_leg, coman::robot_right_leg_ids );
    DPRINTF ( "found %lu <Motor> right_leg\n", right_leg.size() );

    get_esc_map_byclass ( right_arm, coman::robot_right_arm_ids );
    DPRINTF ( "found %lu <Motor> right_arm\n", right_arm.size() );

    get_esc_map_byclass ( waist,	   coman::robot_waist_ids );
    DPRINTF ( "found %lu <Motor> waist\n", waist.size() );

    // !!!
    get_esc_map_byclass ( motors_to_start,  pos_rid );

    std::vector<double> Ys;
    std::vector<double> Xs;

    for ( auto const& item : motors_to_start ) {
    
        slave_pos = item.first;
        moto = item.second;
        moto->readSDO ( "Min_pos", min_pos );
        moto->readSDO ( "Max_pos", max_pos );
        moto->readSDO ( "link_pos", start_pos[slave_pos] );
        // home
        home[slave_pos] = DEG2RAD ( coman::robot_ids_home_pos_deg[pos2Rid(slave_pos)] );
        test_pos[slave_pos] = DEG2RAD ( coman::robot_ids_test_pos_deg[pos2Rid(slave_pos)] );

        DPRINTF ( "Joint_id %d start %f home %f mid %f\n", pos2Rid ( slave_pos ), start_pos[slave_pos], home[slave_pos], test_pos[slave_pos] );

        Ys =  std::initializer_list<double> { start_pos[slave_pos], home[slave_pos] };
        spline_start2home[slave_pos].set_points ( Xt_5s, Ys );

        Ys = std::initializer_list<double> { home[slave_pos], test_pos[slave_pos] };
        spline_home2test[slave_pos].set_points ( Xt_5s, Ys );

        Ys = std::initializer_list<double> { test_pos[slave_pos], home[slave_pos] };
        spline_test2home[slave_pos].set_points ( Xt_5s, Ys );

        /* If k does not match the key of any element in the container, the function inserts a new element with that key
            * and returns a reference to its mapped value. Notice that this always increases the container size by one,
            * even if no mapped value is assigned to the element (the element is constructed using its default constructor).
            */
        Ys =  std::initializer_list<double> { start_pos[slave_pos], home[slave_pos] };
        spline_any2home[slave_pos].set_points ( Xt_5s, Ys );

        //////////////////////////////////////////////////////////////////////////
        // start controller :
        // - read actual joint position and set as pos_ref
        moto->start ( CTRL_SET_POS_MODE );
    }

    DPRINTF ( ">>> wait xddp terminal ....\n" );
    DPRINTF ( ">>> from another terminal run ec_master_test/scripts/xddp_term.py\n" );
    char c; while ( termInXddp.xddp_read ( c ) <= 0 ) { osal_usleep(100); }  
    
    if ( motors_to_start.size() > 0 ) {
        //
        q_spln.push ( &spline_start2home );
    }
}


void EC_boards_coman_test::init_OP ( void ) {

    user_state = HOMING;
    //user_state = IDLE;
    home_state = TEST_HOME;

    if ( ! q_spln.empty() ) {
        running_spline = q_spln.front();
        last_run_spline = running_spline;
        advr::reset_trj ( *running_spline );
    }
    
    DPRINTF ( "End Init_OP\n" );
    
}

int EC_boards_coman_test::user_loop ( void ) {

    static float ds;
    static uint64_t count;
    float spline_error = 0.07;

    if ( ( count++ ) % 1000 == 0 ) {
        DPRINTF ( "alive %ld\n", count/1000 );
        //DPRINTF("%ld\n", q_spln.size());
        //set_any2home(spline_any2home);
    }

    ///////////////////////////////////////////////////////
    //
    if ( xddp_input ( ds ) > 0 ) {
        DPRINTF ( ">> %f\n", ds );
    }

    if ( ! q_spln.empty() ) {

        running_spline = q_spln.front();
        if ( running_spline ) {
            // !@#%@$#%^^# ... tune error
            if ( go_there ( motors_to_start, *running_spline, spline_error, true) ) {
                // running spline has finish !!
                last_run_spline = running_spline;
                // pop running_spline
                q_spln.pop();
                if ( ! q_spln.empty() ) {
                    running_spline = q_spln.front();
                    smooth_splines_trj ( motors_to_start, *running_spline, *last_run_spline );
                    advr::reset_trj ( *running_spline );
                }
            }
        } else {
            DPRINTF ( "Error NULL running spline ... pop it\n" );
            q_spln.pop();
        }
    } else { // q_spln is empty
        
        running_spline = last_run_spline = 0;
#if 1
        if ( motors_to_start.size() > 0 ) {
            // add splines ....
            q_spln.push ( &spline_home2test );
            q_spln.push ( &spline_test2home );
            // !!! since queue was empty reset the first spline
            running_spline = q_spln.front();
            last_run_spline = running_spline;
            advr::reset_trj ( *running_spline );
        }
#endif
    }

}

template<class C>
int EC_boards_coman_test::xddp_input ( C &user_cmd ) {

    static int	bytes_cnt;
    int		bytes = 0;

#ifdef USE_LPMS_IMU
    ///////////////////////////////////////////////////////
    //
    ImuData lpms_data;
    if ( ( bytes = imuInXddp.xddp_read ( lpms_data ) ) > 0 ) {
        DPRINTF ( "Timestamp=%f, qW=%f, qX=%f, qY=%f, qZ=%f\n",
                  lpms_data.timeStamp, lpms_data.q[0], lpms_data.q[1], lpms_data.q[2], lpms_data.q[3] );

    }
    bytes_cnt += bytes;
#endif

    ///////////////////////////////////////////////////////
    //
    spnav_input_t	nav_cmd;
    if ( ( bytes = navInXddp.xddp_read ( nav_cmd ) ) > 0 ) {
        //user_cmd = process_spnav_input(nav_cmd);
        // [-1.0 .. 1.0] / 500 ==> 0.002 rad/ms
        if ( nav_cmd.type == SPNAV_EVENT_MOTION ) {
            user_cmd = ( ( float ) nav_cmd.motion.ry / ( 350.0 ) ) / 500 ;
        } else if ( nav_cmd.type == SPNAV_EVENT_BUTTON ) {
            if ( nav_cmd.button.press ) {
                switch ( nav_cmd.button.bnum ) {
                case 1 :
                    while ( ! q_spln.empty() ) {
                        q_spln.pop();
                    }
                    get_trj_for_end_points ( motors_to_start, spline_any2home, home, 2.0 );
                    if ( running_spline ) {
                        smooth_splines_trj ( motors_to_start, spline_any2home, *running_spline, 1.0 );
                    }
                    advr::reset_trj ( spline_any2home );
                    q_spln.push ( &spline_any2home );
                    DPRINTF ( "ANY2HOME ....\n" );
                    break;
                case 0 :
                    user_state = MOVING;
                    DPRINTF ( "Moving ....\n" );
                    break;
                default :
                    break;
                }
            } else {
                ; // release btn
            }
        }
    }
    bytes_cnt += bytes;
    //DPRINTF(">> %d %d\n",bytes, bytes_cnt);

    ///////////////////////////////////////////////////////
    //
    js_input_t		js_cmd;
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
                if ( js_cmd.value && user_state == IDLE ) {
                    user_state = HOME2MID;
                    advr::reset_trj ( spline_home2test );
                }
                break;
            case 1 :
                if ( js_cmd.value && user_state == IDLE ) {
                    user_state = MID2HOME;
                    advr::reset_trj ( spline_test2home );
                }
                break;
            case 2 :
                if ( js_cmd.value && user_state == IDLE ) {
                    set_any2home ( motors_to_start, spline_any2home, *running_spline );
                    user_state = ANY2HOME;
                    advr::reset_trj ( spline_any2home );
                }
                break;
            case 9 :
                if ( js_cmd.value && user_state == IDLE ) {
                    user_state = HOMING;
                    home_state = TEST_HOME;
                    advr::reset_trj ( spline_start2home );
                }
                break;
            case 4 :
            case 5 :
            case 6 :
            case 7 :
                user_state == IDLE;
                DPRINTF ( "Set IDLE state\n" );
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
