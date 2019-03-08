#include <ec_boards_joint_joy.h>
#include <iit/advr/coman_robot_id.h>
#include <iit/advr/walkman_robot_id.h>
#include <iit/advr/centauro_robot_id.h>

#include <linux/joystick.h>
#include <spnav_config.h>
#ifdef USE_X11
#undef USE_X11
#endif
#include <spnav.h>

#define MID_POS(m,M)    (m+(M-m)/2)

using namespace iit::ecat::advr;

typedef struct js_event js_input_t;
typedef spnav_event     spnav_input_t;

static const std::vector<double> Xt10_10s = std::initializer_list<double> { 0, 1, 2, 3, 4, 5, 6, 10 };
static const std::vector<double> Xt_1s  = std::initializer_list<double> { 0, 1 };
static const std::vector<double> Xt_3s  = std::initializer_list<double> { 0, 3 };
static const std::vector<double> Xt_5s  = std::initializer_list<double> { 0, 5 };
static const std::vector<double> Xt_10s = std::initializer_list<double> { 0, 10 };
static const std::vector<double> Xt_20s = std::initializer_list<double> { 0, 20 };


EC_boards_joint_joy::EC_boards_joint_joy ( const char* config_yaml ) :
    Ec_Thread_Boards_base ( config_yaml ) {

    name = "Ecat_Joint_joy";
    // not periodic
    period.period = {0,1};

#ifdef __COBALT__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max ( schedpolicy );
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    // open pipe ... xeno xddp or fifo
    jsInXddp.init ( "EC_board_js_input" );
    navInXddp.init ( "EC_board_nav_input" );

}

EC_boards_joint_joy::~EC_boards_joint_joy() {

}

void EC_boards_joint_joy::init_preOP ( void ) {

    iit::ecat::advr::Motor * moto;
    int slave_pos;
    float min_pos, max_pos, motor_pos, link_pos;

    std::vector<double> Ys;
    std::vector<double> Xs;
    
    std::vector<int> test_rid = std::initializer_list<int> {

    };


    /////////////////////////////////////////////////////////////////////////
    // change motors map !!
    //get_esc_map_byclass(motors, test_rid);

    DPRINTF ( "found %lu <Motor> instance\n", motors.size() );

    for ( auto const& item : motors ) {
        slave_pos = item.first;
        moto = item.second;
        assert ( iit::ecat::EC_WRP_OK == moto->readSDO ( "Min_pos", min_pos ));
        assert ( iit::ecat::EC_WRP_OK == moto->readSDO ( "Max_pos", max_pos ));
        assert ( iit::ecat::EC_WRP_OK == moto->readSDO ( "motor_pos", motor_pos ));
        assert ( iit::ecat::EC_WRP_OK == moto->readSDO ( "link_pos", link_pos ));

        start_pos[slave_pos] = motor_pos; 
        // set home pos
        home[slave_pos] = MID_POS ( min_pos,max_pos );
        //home[slave_pos] = DEG2RAD(iit::ecat::advr::coman::robot_ids_home_pos_deg[pos2Rid(slave_pos)]);
        step_1[slave_pos] = home[slave_pos] + 1.0;
        step_2[slave_pos] = home[slave_pos] - 1.0;
        DPRINTF ( "%d home %f mid %f step %f\n", pos2Rid ( slave_pos ), home[slave_pos],step_1[slave_pos],step_2[slave_pos] );

        Ys =  std::initializer_list<double> { start_pos[slave_pos], home[slave_pos] };
        Xs = std::initializer_list<double> { 0, 5 };
        trj_names["start@home"][slave_pos] = std::make_shared<advr::Smoother_trajectory> ( Xs, Ys );
        
        Ys = std::initializer_list<double> { home[slave_pos], 1.5, 2.5, 1.0, 2.2, 1.0 , 2.0, home[slave_pos] };
        trj_names["trj_1"][slave_pos] = std::make_shared<advr::Smoother_trajectory> ( Xt10_10s,Ys );
        Ys = std::initializer_list<double> { home[slave_pos], 5.0, 3.5, 4.5, 4.0, 3.0 , 3.5, home[slave_pos] };
        trj_names["trj_2"][slave_pos] = std::make_shared<advr::Smoother_trajectory> ( Xt10_10s,Ys );


        //////////////////////////////////////////////////////////////////////////
        // start controller :
        // - read actual joint position and set as pos_ref
        //assert ( moto->start ( CTRL_SET_POS_MOTOR_MODE ) == EC_BOARD_OK );
        assert ( moto->start ( ) == EC_BOARD_OK );

    }

    user_state = HOMING;

    trj_queue.clear();
    trj_queue.push_back ( trj_names.at("start@home") );
}


void EC_boards_joint_joy::init_OP ( void ) {

    try { advr::reset_trj ( trj_queue.at(0) ); }
    catch ( const std::out_of_range &e ) {
        throw std::runtime_error("Oh my gosh  ... trj_queue is empty !");
    }    

}

int EC_boards_joint_joy::user_loop ( void ) {

    const float spline_error = 0.05;
    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    static float ds;

    //
    if ( user_input ( ds ) > 0 ) {
        DPRINTF ( ">> %f\n", ds );
    }

    try { 
        if ( go_there ( motors, trj_queue.at(0), spline_error, false) ) {
            // running trj has finish ... remove from queue  !!
            DPRINTF ( "running trj has finish ... remove from queue !!\n" );
            trj_queue.pop_front();
            try { advr::reset_trj ( trj_queue.at(0) ); }
            catch ( const std::out_of_range &e ) {
                // add trajectory ....
                //trj_queue.push_back ( trj_names.at("trj_1") );
                //advr::reset_trj ( trj_queue.at(0) );
            }
        }
    } catch ( const std::out_of_range &e ) {
        //throw std::runtime_error("Ooops ... trj_queue is empty !");
        //DPRINTF("Ooops ... trj_queue is empty !");
    }

    
    //
    switch ( user_state ) {

    case IDLE :
        break;

    case HOMING :
        break;

    case MOVING :
        for ( auto const& item : motors ) {
            moto =  item.second;
            motor_pdo_rx = moto->getRxPDO();
            // pos_ref_fb is the previous reference
            moto->set_posRef ( motor_pdo_rx.link_pos + ds );
        }
        break;

    default:
        DPRINTF ( "Wrong user state ....\n" );
        break;
    }

    return 0;
}


template<class C>
int EC_boards_joint_joy::user_input ( C &user_cmd ) {

    static int  bytes_cnt;
    int         bytes;

    ///////////////////////////////////////////////////////////////////////////
    //
    {
        spnav_input_t   nav_cmd;

        if ( ( bytes = navInXddp.xddp_read ( nav_cmd ) ) > 0 ) {
            //user_cmd = process_spnav_input(nav_cmd);
            // [-1.0 .. 1.0] / 200 ==> 0.005 rad/ms
            if ( nav_cmd.type == SPNAV_EVENT_MOTION ) {
                user_cmd = ( ( float ) nav_cmd.motion.ry / ( 500.0 ) ) / 10 ;
                DPRINTF ( "%f\n", user_cmd);
            } else if ( nav_cmd.type == SPNAV_EVENT_BUTTON ) {
                if ( nav_cmd.button.press ) {
                    switch ( nav_cmd.button.bnum ) {
                    case 1 :
                        user_state = ANY2HOME;
                        //set_any2home ( motors, trj_any2home, trj_queue.at(0) );
                        DPRINTF ( "ANY2HOME ....\n" );
                        trj_queue.clear();
                        break;
                    case 0 :
                        user_state = MOVING;
                        DPRINTF ( "Moving ....\n" );
                        trj_queue.clear();
                        break;
                    default :
                        break;
                    }
                } else {
                    ; // release btn
                }
            }
        }
    }
    //
    ///////////////////////////////////////////////////////////////////////////
    
    bytes_cnt += bytes;
    //DPRINTF(">> %d %d\n",bytes, bytes_cnt);
#if 0    
    ///////////////////////////////////////////////////////////////////////////
    //
    {
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

            default:
                break;

            }
        }
    }
    //
    ///////////////////////////////////////////////////////////////////////////
    
    bytes_cnt += bytes;
    //DPRINTF(">> %d %d\n",bytes, bytes_cnt);
#endif
    
#if 0
    //////////////////////////////////////////////////////////////////////
    float ax_value = 0;

    switch ( cmd.type & ~JS_EVENT_INIT ) {
    case JS_EVENT_AXIS:
        // normalize to [-1.0 .. 1.0]
        ax_value = ( float ) cmd.value/ ( 32767.0 );

        switch ( cmd.number ) {
        case 0 :
            // [-0.002 .. 0.002] rads
            user_cmd = ax_value / 500;
            break;
        case 2 :
            // [-0.004 .. 0.004] rads
            user_cmd = ax_value / 250;
            break;
        default:
            break;
        }
        break;

    case JS_EVENT_BUTTON:
        switch ( cmd.number ) {
        case 0 :
            break;
        default:
            if ( cmd.value ) {
                //slave_as_LP(rid2pos[JOINT_TEST])->print_info();
            }
            break;
        }
        break;

    default:
        break;

    }
    //////////////////////////////////////////////////////////////////////
#else
    // [-1..1] / 200 ==> 0.005 rad/ms
    //user_cmd = ((float)nav_cmd.motion.ry / (350.0)) / 200;
#endif

    return bytes;
}
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
