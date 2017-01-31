#include <ec_boards_walkman_test.h>
#include <iit/advr/walkman_robot_id.h>

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


EC_boards_walkman_test::EC_boards_walkman_test(const char* config_yaml) :
    Ec_Thread_Boards_base ( config_yaml ) {

    name = "Walkman_test";
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

EC_boards_walkman_test:: ~EC_boards_walkman_test() {

}

/*
 *
 */
void EC_boards_walkman_test::init_preOP ( void ) {

    Motor * moto;
    int slave_pos, motor_start;
    float min_pos, max_pos, pos_ref_fb;

    get_esc_map_byclass ( left_leg,  walkman::robot_left_leg_ids );
    DPRINTF ( "found %lu <Motor> left_leg\n", left_leg.size() );

    get_esc_map_byclass ( left_arm,  walkman::robot_left_arm_ids );
    DPRINTF ( "found %lu <Motor> left_arm\n", left_arm.size() );

    get_esc_map_byclass ( right_leg, walkman::robot_right_leg_ids );
    DPRINTF ( "found %lu <Motor> right_leg\n", right_leg.size() );

    get_esc_map_byclass ( right_arm, walkman::robot_right_arm_ids );
    DPRINTF ( "found %lu <Motor> right_arm\n", right_arm.size() );

    get_esc_map_byclass ( waist,	   walkman::robot_waist_ids );
    DPRINTF ( "found %lu <Motor> waist\n", waist.size() );

    get_esc_map_byclass ( head,       walkman::robot_head_ids );
    DPRINTF ( "found %lu <Motor> head\n", head.size() );

    get_esc_map_byclass ( hands,       walkman::robot_hands_ids );
    DPRINTF ( "found %lu <Motor> hands\n", hands.size() );

    std::vector<double> vPos;
    std::vector<double> vVel;
    std::vector<double> vTor;
    std::vector<double> Xs;

    ///////////////////////////////////////////////////////////////////////
    // prepare trajectory splines for ALL motors
    for ( auto const& item : motors ) {
    
        slave_pos = item.first;
        moto = item.second;
        moto->readSDO ( "Min_pos", min_pos );
        moto->readSDO ( "Max_pos", max_pos );
        moto->readSDO ( "link_pos", start_pos[slave_pos] );
        // home
        home[slave_pos] = DEG2RAD ( walkman::robot_ids_home_pos_deg[pos2Rid(slave_pos)] );
        test_pos[slave_pos] = DEG2RAD ( walkman::robot_ids_test_pos_deg[pos2Rid(slave_pos)] );
        
        DPRINTF ( "Joint_id %d start %f home %f test_pos %f\n",
                  pos2Rid ( slave_pos ), start_pos[slave_pos], home[slave_pos], test_pos[slave_pos] );

        vPos =  std::initializer_list<double> { start_pos[slave_pos], home[slave_pos] };
        trj_start2home[slave_pos] = std::make_shared<advr::Smoother_trajectory> ( Xt_5s, vPos );

        vPos = std::initializer_list<double> { home[slave_pos], test_pos[slave_pos], home[slave_pos] };
        trj_home2test_pos2home[slave_pos] = std::make_shared<advr::Smoother_trajectory> ( std::initializer_list<double>{ 0, 3, 6 }, vPos );

        /* If k does not match the key of any element in the container, the function inserts a new element with that key
            * and returns a reference to its mapped value. Notice that this always increases the container size by one,
            * even if no mapped value is assigned to the element (the element is constructed using its default constructor).
            */
        vPos =  std::initializer_list<double> { start_pos[slave_pos], home[slave_pos] };
        trj_any2home[slave_pos] = std::make_shared<advr::Smoother_trajectory> ( Xt_5s, vPos );

    }
    
    
    std::vector<int> pos_ctrl_ids = walkman::robot_mcs_ids;
//     std::vector<int> pos_ctrl_ids = walkman::robot_left_arm_ids;
//     std::vector<int> pos_ctrl_ids = walkman::robot_right_leg_ids;
//     std::vector<int> pos_ctrl_ids = walkman::robot_left_leg_ids;
//     std::vector<int> pos_ctrl_ids = std::initializer_list<int> {
//     };

    std::vector<int> tor_ctrl_ids = std::initializer_list<int> {
        walkman::RL_H_R, // 41
    };

    
    std::vector<int> no_control = std::initializer_list<int> {
    };
    
//    remove_rids_intersection(pos_ctrl_ids, no_control);
//     remove_rids_intersection(pos_ctrl_ids, walkman::robot_left_arm_ids);
//     remove_rids_intersection(pos_ctrl_ids, walkman::robot_right_arm_ids);
    remove_rids_intersection(pos_ctrl_ids, walkman::robot_waist_ids);
//     remove_rids_intersection(pos_ctrl_ids, walkman::robot_left_leg_ids);
//     remove_rids_intersection(pos_ctrl_ids, walkman::robot_right_leg_ids);
//     remove_rids_intersection(pos_ctrl_ids, walkman::robot_head_ids);
//     remove_rids_intersection(pos_ctrl_ids, walkman::robot_hands_ids);
    
    get_esc_map_byclass ( motors2ctrl,  pos_ctrl_ids );

    for ( auto const& item : motors2ctrl ) {
        slave_pos = item.first;
        moto = item.second;

        //DPRINTF ( ">>> START %d wait xddp terminal ....\n", moto->get_robot_id() );
        //char c; while ( termInXddp.xddp_read ( c ) <= 0 ) { osal_usleep(100); }
        
        //////////////////////////////////////////////////////////////////////////
        // start controller :
        // - read actual joint position and set as pos_ref
        if (moto->get_ESC_type() == LO_PWR_DC_MC ) {
            motor_start = moto->start ( CTRL_SET_POS_MODE );
        } else if ( moto->get_ESC_type() == HI_PWR_AC_MC ) {
            //motor_start = moto->start ( CTRL_SET_MIX_POS_MODE );
            motor_start = moto->start ( CTRL_SET_IMPED_MODE );
        } else if ( moto->get_ESC_type() == HI_PWR_DC_MC) {
            //motor_start = moto->start ( CTRL_SET_POS_MODE );
            //motor_start = moto->start ( CTRL_SET_MIX_POS_MODE );
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
        trj_queue.push ( &trj_start2home );
    }
}


void EC_boards_walkman_test::init_OP ( void ) {

    //DPRINTF ( ">>> wait xddp terminal ....\n" );
    //char c; while ( termInXddp.xddp_read ( c ) <= 0 ) { osal_usleep(100); }  

    if ( ! trj_queue.empty() ) {
        running_trj = trj_queue.front();
        last_run_trj = running_trj;
        advr::reset_trj ( *running_trj );
    }
    
    DPRINTF ( "End Init_OP\n" );
    
}

int EC_boards_walkman_test::user_loop ( void ) {

    static float ds;
    static uint64_t count;
    const float spline_error = 0.07;

    if ( ( count++ ) % 1000 == 0 ) {
        DPRINTF ( "alive %d\n", count/1000 );
    }

    ///////////////////////////////////////////////////////
    //
    if ( xddp_input ( ds ) > 0 ) {
        DPRINTF ( ">> %f\n", ds );
    }


    if ( ! trj_queue.empty() ) {

        running_trj = trj_queue.front();
        if ( running_trj ) {
            // !@#%@$#%^^# ... tune error
            if ( go_there ( motors2move, *running_trj, spline_error, false) ) {
                // running spline has finish !!
                last_run_trj = running_trj;
                trj_queue.pop();
                if ( ! trj_queue.empty() ) {
                    running_trj = trj_queue.front();
                    advr::reset_trj ( *running_trj );
                }
            }
        } else {
            DPRINTF ( "Error NULL running spline ... pop it\n" );
            trj_queue.pop();
        }
    } else { // trj_queue is empty

        running_trj = last_run_trj = 0;
            
        if ( motors2move.size() > 0 ) {
            // add splines ....
            trj_queue.push ( &trj_home2test_pos2home );
            // !!! since queue was empty reset the first spline
            running_trj = trj_queue.front();
            last_run_trj = running_trj;
            advr::reset_trj ( *running_trj );
        }
    
    }

}

void EC_boards_walkman_test::move_head( float pitch_pos, float roll_pos ) {
    
    float pitchRef, rollRef;
    Motor::motor_pdo_rx_t motor_pdo_rx;
    Motor::motor_pdo_tx_t motor_pdo_tx;
    
    Motor * head_pitch = slave_as_Motor( rid2Pos(walkman::HEAD_P) );
    if ( head_pitch ) {
        motor_pdo_rx = head_pitch->getRxPDO();
        //motor_pdo_tx = head_pitch->getTxPDO();
        pitchRef = motor_pdo_rx.link_pos + pitch_pos;
        head_pitch->set_posRef( pitchRef );
    }
    Motor * head_roll = slave_as_Motor( rid2Pos(walkman::HEAD_R) );
    if ( head_roll ) {
        motor_pdo_rx = head_roll->getRxPDO();
        //motor_pdo_tx = head_roll->getTxPDO();
        rollRef = motor_pdo_rx.link_pos + roll_pos;
        head_roll->set_posRef( rollRef );
    }
    //DPRINTF ( "Head pitch %f\troll %f\n", pitchRef, rollRef );
}

void EC_boards_walkman_test::move_hands( float left_pos, float right_pos ) {

    float leftRef, rightRef;
    //Motor::motor_pdo_rx_t motor_pdo_rx;
    //Motor::motor_pdo_tx_t motor_pdo_tx;
    
    Motor * left_hand = slave_as_Motor( rid2Pos(walkman::LA_HA) );
    if ( left_hand ) {
        //motor_pdo_rx = left_hand->getRxPDO();
        //motor_pdo_tx = left_hand->getTxPDO();
        leftRef = left_pos;
        left_hand->set_posRef( leftRef );
    }
    Motor * right_hand = slave_as_Motor( rid2Pos(walkman::RA_HA) );
    if ( right_hand ) {
        //motor_pdo_rx = right_hand->getRxPDO();
        //motor_pdo_tx = right_hand->getTxPDO();
        rightRef = right_pos;
        right_hand->set_posRef( rightRef );
    }
    //DPRINTF ( "Head pitch %f\troll %f\n", pitchRef, rollRef );
}

template<class C>
int EC_boards_walkman_test::xddp_input ( C &user_cmd ) {

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

#ifdef USE_LPMS_IMU
    ///////////////////////////////////////////////////////
    //
    {
        ImuData lpms_data;
        if ( ( bytes = imuInXddp.xddp_read ( lpms_data ) ) > 0 ) {
            DPRINTF ( "Timestamp=%f, qW=%f, qX=%f, qY=%f, qZ=%f\n",
                    lpms_data.timeStamp, lpms_data.q[0], lpms_data.q[1], lpms_data.q[2], lpms_data.q[3] );
        }
    bytes_cnt += bytes;
    }
#endif
    ///////////////////////////////////////////////////////
    //
    {
    spnav_input_t	nav_cmd;
    float pitch_ry, roll_rx;
    
    if ( ( bytes = navInXddp.xddp_read ( nav_cmd ) ) > 0 ) {
        //user_cmd = process_spnav_input(nav_cmd);
        // [-1.0 .. 1.0] / 500 ==> 0.002 rad/ms
        if ( nav_cmd.type == SPNAV_EVENT_MOTION ) {
            
            roll_rx = ( ( float ) nav_cmd.motion.rx / ( 350.0 ) ) / 500 ;
            pitch_ry  = ( ( float ) nav_cmd.motion.ry / ( 350.0 ) ) / 500 ;
            move_head( pitch_ry, roll_rx );

        } else if ( nav_cmd.type == SPNAV_EVENT_BUTTON ) {
            if ( nav_cmd.button.press ) {
                switch ( nav_cmd.button.bnum ) {
                case 1 :
                    // open hands
                    move_hands( 0, 0 );
                    break;
                case 0 :
                    // close hands
                    move_hands( 6, 6 );
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
    }

    ///////////////////////////////////////////////////////
    //
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
