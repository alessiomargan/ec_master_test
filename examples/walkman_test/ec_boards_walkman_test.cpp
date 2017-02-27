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

typedef struct js_event js_input_t;
typedef spnav_event     spnav_input_t;


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
    float pos_ref_fb;

    get_esc_map_byclass ( left_leg, walkman::robot_left_leg_ids );
    DPRINTF ( "found %lu <Motor> left_leg\n", left_leg.size() );

    get_esc_map_byclass ( left_arm, walkman::robot_left_arm_ids );
    DPRINTF ( "found %lu <Motor> left_arm\n", left_arm.size() );

    get_esc_map_byclass ( right_leg, walkman::robot_right_leg_ids );
    DPRINTF ( "found %lu <Motor> right_leg\n", right_leg.size() );

    get_esc_map_byclass ( right_arm, walkman::robot_right_arm_ids );
    DPRINTF ( "found %lu <Motor> right_arm\n", right_arm.size() );

    get_esc_map_byclass ( waist, walkman::robot_waist_ids );
    DPRINTF ( "found %lu <Motor> waist\n", waist.size() );

    get_esc_map_byclass ( head, walkman::robot_head_ids );
    DPRINTF ( "found %lu <Motor> head\n", head.size() );

    get_esc_map_byclass ( hands, walkman::robot_hands_ids );
    DPRINTF ( "found %lu <Motor> hands\n", hands.size() );

    ///////////////////////////////////////////////////////////////////////////
    //
    const YAML::Node config_poses = get_config_YAML_Node();

    std::map<std::string, std::map<walkman::Robot_IDs_t, float>> joints_pose_deg;
    std::map<std::string, std::map<std::string, float>> poses_map; 
    std::map<std::string, std::vector<std::pair<std::string, float>>> trj_map;
    std::map<std::string, std::vector<std::string>> joint_group_by_ctrl;
    std::map<std::string, std::vector<int>> map_of_ctrlIDs;
        
    try { 
        std::string poses_file = config_poses["ec_boards_base"]["joints_poses_file"].as<std::string>();
        const YAML::Node poses_config = YAML::LoadFile ( poses_file );
        
        trj_map = poses_config["trjs_map"].as<std::map<std::string, std::vector<std::pair<std::string, float>>>>();
        auto trjs_keys = extract_keys( trj_map );
        auto trjs_values = extract_values( trj_map );
        std::cout << '[';
        for ( auto const &k: trjs_keys ) { std::cout << k << ','; }
        std::cout << ']' << std::endl; 
        std::cout << '[';
        for ( auto const &v: trjs_values ) { 
            std::cout << '[';
            for ( auto const &p: v ) { std::cout << p.first << ' ' << p.second << ','; }
            std::cout << ']'; 
        }
        std::cout << ']' << std::endl;

        poses_map = poses_config["pos_map"].as<std::map<std::string,std::map<std::string,float>>>();
        auto poses_keys = extract_keys( poses_map );
        for ( auto const &p: poses_keys )  { std::cout << p << ' '; } std::cout << std::endl; 
        for ( auto const &pose : poses_map ) {
            auto pose_name = pose.first;
            auto pose_joints = pose.second;
            for ( auto const &kv : pose_joints ) {
                auto joint_name = kv.first;
                auto joint_pose_value = kv.second;
                joints_pose_deg[pose.first][walkman::robot_ids_names.at(joint_name)] = joint_pose_value;
                DPRINTF(">> %s %d_%s\t%f\n",
                        pose.first.c_str(),
                        walkman::robot_ids_names.at(joint_name),
                        joint_name.c_str(),
                        joints_pose_deg.at(pose.first).at(walkman::robot_ids_names.at(joint_name)));
            }
        }

        joint_group_by_ctrl = poses_config["joint_group_by_controller"].as<std::map<std::string, std::vector<std::string>>>();
        auto joint_group_by_ctrl_keys = extract_keys( joint_group_by_ctrl );
        for ( auto const &p: joint_group_by_ctrl_keys )  { std::cout << p << ' '; } std::cout << std::endl; 
        for ( auto const &c: joint_group_by_ctrl )  {
            auto ctrl_name = c.first;
            auto ctrl_set = c.second;
            map_of_ctrlIDs[ctrl_name];
            for ( auto const &g: ctrl_set )  {
                if ( g[0] == '@' ) {
                    for ( auto const &ids : walkman::robot_ids_group_names.at(g) ) {
                        map_of_ctrlIDs[ctrl_name].push_back( ids );
                    }
                } else {
                    map_of_ctrlIDs[ctrl_name].push_back( walkman::robot_ids_names.at(g) );
                }
            }
        }
        
    } catch ( const std::exception &e ) {
        std::cout << e.what() << '\n';
        //
        joints_pose_deg.empty();
        trj_map.empty();
        map_of_ctrlIDs.empty();
    }
    
    //////////////////////////////////////////////////////////////////////////
    // prepare trajectory for ALL motors
    DPRINTF ( "### Prepare trayectories\n" );
    std::vector<double> vPos;
    std::vector<double> vVel;
    std::vector<double> vTor;
    std::vector<double> Xs;

    for ( auto const& item : motors ) {
    
        slave_pos = item.first;
        moto = item.second;
        moto->readSDO ( "link_pos", start_pos[slave_pos] );
        // home
        home[slave_pos] = DEG2RAD ( joints_pose_deg.at("home").at((walkman::Robot_IDs)pos2Rid(slave_pos)) );
        
        DPRINTF ("Joint_id %d start %f home %f\n", pos2Rid ( slave_pos ), start_pos[slave_pos], home[slave_pos]);

        vPos =  std::initializer_list<double> { start_pos[slave_pos], home[slave_pos] };
        trj_names["start2home"][slave_pos] = std::make_shared<advr::Smoother_trajectory> ( Xt_5s, vPos );

        for ( auto const & trj_name : extract_keys( trj_map ) ) {
            Xs.clear();
            vPos.clear();
            for ( auto const & trj_pos_time : trj_map[trj_name] ) {
                auto pos = joints_pose_deg.at(trj_pos_time.first).at((walkman::Robot_IDs)pos2Rid(slave_pos));
                vPos.push_back( DEG2RAD ( pos ) );
                auto time = trj_pos_time.second;
                Xs.push_back( time );
                DPRINTF("%s time %f pos %f\n", trj_name.c_str(), time, pos);
            }
            trj_names[trj_name][slave_pos] = std::make_shared<advr::Smoother_trajectory> ( Xs, vPos );
        }

    }

    //////////////////////////////////////////////////////////////////////////
    //
    DPRINTF ( "### Prepare controller group\n" );
    std::vector<int> ids2ctrl, tmp_union;
    std::vector<int> ids_pos = map_of_ctrlIDs.at("position_ctrl");
    std::vector<int> ids_imp = map_of_ctrlIDs.at("impedance_ctrl");
    std::vector<int> ids_tor = map_of_ctrlIDs.at("torque_ctrl");
    std::vector<int> ids_dir = map_of_ctrlIDs.at("direct_ctrl");
    std::vector<int> ids_NO  = map_of_ctrlIDs.at("NO_ctrl");
    for ( auto v : std::initializer_list<std::vector<int>>{ ids_pos, ids_imp, ids_tor, ids_dir, ids_NO } ) {
        std::sort (v.begin(), v.end());
    }
    std::set_union(ids_pos.begin(), ids_pos.end(),ids_imp.begin(), ids_imp.end(),  std::back_inserter(ids2ctrl) );
    std::set_union(ids2ctrl.begin(), ids2ctrl.end(),ids_tor.begin(), ids_tor.end(),  std::back_inserter(tmp_union) );
    std::set_union(tmp_union.begin(), tmp_union.end(),ids_dir.begin(), ids_dir.end(),  std::back_inserter(ids2ctrl) );
    
    remove_rids_intersection( ids2ctrl, ids_NO );
    
    get_esc_map_byclass ( motors2ctrl,  ids2ctrl );

    for ( auto const& item : motors2ctrl ) {
        slave_pos = item.first;
        moto = item.second;
        motor_start = -1;
        //DPRINTF ( ">>> START %d wait xddp terminal ....\n", moto->get_robot_id() );
        //char c; while ( termInXddp.xddp_read ( c ) <= 0 ) { osal_usleep(100); }
        
        //////////////////////////////////////////////////////////////////////////
        // start controller :
        // - read actual joint position and set as pos_ref
        if ( std::find( ids_pos.begin(), ids_pos.end(), pos2Rid(slave_pos) ) != ids_pos.end() ) {

            if (moto->get_ESC_type() == LO_PWR_DC_MC ) {
                motor_start = moto->start ( CTRL_SET_POS_MODE );
            } else if ( (moto->get_ESC_type() == HI_PWR_AC_MC) || (moto->get_ESC_type() == HI_PWR_DC_MC) ) {
                motor_start = moto->start ( CTRL_SET_MIX_POS_MODE );
            } else {
                
            }
                    
        } else if ( std::find( ids_imp.begin(), ids_imp.end(), pos2Rid(slave_pos) ) != ids_imp.end() ) {

            motor_start = moto->start ( CTRL_SET_IMPED_MODE );
            
        } else if ( std::find( ids_tor.begin(), ids_tor.end(), pos2Rid(slave_pos) ) != ids_tor.end() ) {

            motor_start = moto->start ( CTRL_SET_IMPED_MODE );
            
        } else if ( std::find( ids_dir.begin(), ids_dir.end(), pos2Rid(slave_pos) ) != ids_dir.end() ) {

            motor_start = moto->start ( CTRL_SET_CURR_MODE );
        } 
        

        assert( motor_start == EC_BOARD_OK );
        
        //while ( ! moto->move_to(home[slave_pos], 0.005) ) { osal_usleep(100);  }
    }

    motors2move = motors2ctrl;
    
    DPRINTF ( ">>> motors2ctrl %ld motors2move %ld\n", motors2ctrl.size(), motors2move.size() );
    DPRINTF ( ">>> wait xddp terminal ....\n" );
    char c; while ( termInXddp.xddp_read ( c ) <= 0 ) { osal_usleep(100); }  
    
    if ( motors2move.size() > 0 ) {
        trj_queue.push ( &trj_names.at("start2home") );
    }
}


void EC_boards_walkman_test::init_OP ( void ) {

    //DPRINTF ( ">>> wait xddp terminal ....\n" );
    //char c; while ( termInXddp.xddp_read ( c ) <= 0 ) { osal_usleep(100); }  

    if ( ! trj_queue.empty() ) {
        running_trj = trj_queue.front();
        advr::reset_trj ( *running_trj );
    }
    
    DPRINTF ( "End Init_OP\n" );
    
}

int EC_boards_walkman_test::user_loop ( void ) {

    static float ds;
    static uint64_t count;
    const float spline_error = 0.07;

    if ( ( count++ ) % 1000 == 0 ) {
        DPRINTF ( "alive %ld\n", count/1000 );
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
    } else {
        // trj_queue is empty
        running_trj = 0;
            
        if ( motors2move.size() > 0 ) {
            // add trajectory ....
            trj_queue.push ( &trj_names["home@pos1@pos2@pos3@home"] );
            // !!! since queue was empty reset the first spline
            if ( ! trj_queue.empty() ) {
                running_trj = trj_queue.front();
                advr::reset_trj ( *running_trj );
            }
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
