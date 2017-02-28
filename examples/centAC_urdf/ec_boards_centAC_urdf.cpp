#include <ec_boards_centAC_urdf.h>
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


static const std::vector<double> Xt_2s = std::initializer_list<double> { 0, 2 };
static const std::vector<double> Xt_3s = std::initializer_list<double> { 0, 3 };
static const std::vector<double> Xt_4s = std::initializer_list<double> { 0, 4 };
static const std::vector<double> Xt_5s = std::initializer_list<double> { 0, 5 };
static const std::vector<double> Xt_9s = std::initializer_list<double> { 0, 9 };
static const std::vector<double> Xt3_6s = std::initializer_list<double> { 0, 4, 6 };

using namespace iit::ecat::advr;

EC_boards_centAC_urdf::EC_boards_centAC_urdf ( const char* config_yaml ) :
    Ec_Thread_Boards_base ( config_yaml ), URDF_adapter( ) {

    name = "centauro_test";
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
    
    init_urdf(get_config_YAML_Node()["ec_boards_base"]["urdf_config_file"].as<std::string>());
         
}

EC_boards_centAC_urdf::~EC_boards_centAC_urdf() {

    delete demo_class;
}

/*
 *
 */
void EC_boards_centAC_urdf::init_preOP ( void ) {

    Motor * moto;
    int slave_pos;
    float min_pos, max_pos, link_pos, motor_pos;
   
    //std::vector<int> pos_rid = centauro::robot_mcs_ids;
    std::vector<int> pos_rid = get_urdf_rId();
    std::vector<int> no_control = std::initializer_list<int> {
        centauro::RA_HA,
        centauro::LA_HA,
    };
       
    remove_rids_intersection(pos_rid, no_control);

    // !!!
    get_esc_map_byclass ( motors_to_start,  pos_rid );
    DPRINTF ( "found %lu <Motor> to_start >> %lu in urdf model\n", motors_to_start.size(), pos_rid.size() );

    std::vector<double> Ys;
    std::vector<double> Xs;

    
    for ( auto const& item : motors_to_start ) {
    
        slave_pos = item.first;
        moto = item.second;
        moto->readSDO ( "Min_pos", min_pos );
        moto->readSDO ( "Max_pos", max_pos );
        assert ( EC_WRP_OK == moto->readSDO ( "motor_pos", motor_pos ));
        assert ( EC_WRP_OK == moto->readSDO ( "link_pos", link_pos ));
        start_pos[slave_pos] = motor_pos; 
        
        //
        Q(rid2Urdf(pos2Rid(slave_pos))) = DEG2RAD ( centauro::robot_ids_home_pos_deg.at(pos2Rid(slave_pos)) );
        
        // home
        home[slave_pos] = DEG2RAD ( centauro::robot_ids_home_pos_deg.at(pos2Rid(slave_pos)) );

        Ys =  std::initializer_list<double> { start_pos[slave_pos], home[slave_pos], home[slave_pos] };
        spline_start2home[slave_pos] = std::make_shared<advr::Spline_trajectory> ( Xt3_6s, Ys );

        //////////////////////////////////////////////////////////////////////////
        // start controller :
        // - read actual joint position and set as pos_ref
        moto->start ( CTRL_SET_POS_MODE );
    }

    //////////////////////////////////////////////////////////////////////
    // Set trajcetories
    UpperbodyDemoClass::TrajectoryParameters left_arm_trj;
    left_arm_trj.type = UpperbodyDemoClass::TrajectoryParameters::circular;//linear
    left_arm_trj.center = Eigen::Vector3d(0.55, 0.406, 0.85-0.717);//(0.55, 0.406, 0.85-0.717);
    left_arm_trj.direction = Eigen::Vector3d(0, 0, 1);
    left_arm_trj.radius = 0.15;
    left_arm_trj.length = 0.4;
    left_arm_trj.period = 6.0;

    UpperbodyDemoClass::TrajectoryParameters right_arm_trj;
    right_arm_trj.type = UpperbodyDemoClass::TrajectoryParameters::linear;
    right_arm_trj.center = Eigen::Vector3d(0.55, -0.406, 0.85-0.717);//(0.55, -0.406, 0.85-0.867);
    right_arm_trj.direction = Eigen::Vector3d(0, 0, 1);
    right_arm_trj.radius = 0.1;
    right_arm_trj.length = 0.4;
    right_arm_trj.period = 6.0;

    std::map<std::string, UpperbodyDemoClass::TrajectoryParameters> traj_map;
    traj_map["arm1_7"] = left_arm_trj;
    traj_map["arm2_7"] = right_arm_trj;

    demo_class = new UpperbodyDemoClass(model_xml_string);
    
    demo_class->rbdl_to_map(Q, q_map);
    demo_class->init(model_xml_string, Q, traj_map);
    
    DPRINTF ( ">>> wait xddp terminal ....\n" );
    DPRINTF ( ">>> from another terminal run ec_master_test/scripts/xddp_term.py\n" );
    char c; while ( termInXddp.xddp_read ( c ) <= 0 ) { osal_usleep(100); }  
    
    if ( motors_to_start.size() > 0 ) {
        //
        trj_queue.push_back ( spline_start2home );
    }
}


void EC_boards_centAC_urdf::init_OP ( void ) {


    try { advr::reset_trj ( trj_queue.at(0) ); }
    catch ( const std::out_of_range &e ) {
        throw std::runtime_error("Oh my gosh  ... trj_queue is empty !");
    }    

    
    DPRINTF ( "End Init_OP\n" );
    
}

int EC_boards_centAC_urdf::user_loop ( void ) {

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
        if ( go_there ( motors_to_start, trj_queue.at(0), spline_error, false) ) {
            // running trj has finish ... remove from queue  !!
            DPRINTF ( "running trj has finish ... remove from queue !!\n" );
            trj_queue.pop_front();
            try { advr::reset_trj ( trj_queue.at(0) ); }
            catch ( const std::out_of_range &e ) {
                // add trajectory ....
            }
        }
    } else { // trj_queue is empty
        
        ///////////////////////////////////////////////////////
        //
        {
        static float start_time_sec;
        static float prev_time_sec;
        
        start_time_sec = start_time_sec ? start_time_sec : (float)iit::ecat::get_time_ns() / NSEC_PER_SEC;
        float tNow_sec = (float)iit::ecat::get_time_ns() / NSEC_PER_SEC;
        float dt = tNow_sec - prev_time_sec;
        float elapsed = tNow_sec - start_time_sec;
        
        static float IterativeTime = 0.0;
        IterativeTime +=0.001;

        DPRINTF(">> %f %f\n", elapsed, dt);
        
        Motor * moto;
        int slave_pos;
        for ( auto const& item : motors_to_start ) {
            slave_pos = item.first;
            moto = item.second;
            Q(rid2Urdf(pos2Rid(slave_pos))) = moto->getRxPDO().motor_pos;
            
        }
        
         Eigen::Vector3d p_right =  rbd::CalcBodyToBaseCoordinates(*model, q_rbdl, model->GetBodyId("torso_2"), Vector3dZero);
         Eigen::Vector3d p_right_world = rbd::CalcBaseToBodyCoordinates(*model, q_rbdl, model->GetBodyId("pelvis"), p_right, false);
//         //Eigen::Vector3d p_right =  rbd::CalcBodyToBaseCoordinates(*model, Q, model->GetBodyId("arm2_7"), Vector3dZero);
//         //Eigen::Vector3d p_right_world = rbd::CalcBaseToBodyCoordinates(*model, Q, model->GetBodyId("pelvis"), p_right, false);
         Eigen::Vector3d p_left =  rbd::CalcBodyToBaseCoordinates(*model, Q, model->GetBodyId("arm1_7"), Vector3dZero);
         Eigen::Vector3d p_left_world = rbd::CalcBaseToBodyCoordinates(*model, Q, model->GetBodyId("pelvis"), p_left, false);
        
         RigidBodyDynamics::CalcPointJacobian(*model, Q, model->GetBodyId("pelvis"), p_left, Jack, false );

            std::stringstream ss;
//        ss << p_right_world << std::endl << p_left_world;
        
//        demo_class->run(IterativeTime, dt, Q);
        prev_time_sec = tNow_sec;  

        ss << Q;
        DPRINTF("%s\n", ss.str().c_str());

        for ( auto const& item : motors_to_start ) {
    
            slave_pos = item.first;
            moto = item.second;
            // !!!!
            //moto->set_posRef(Q(rid2Urdf(pos2Rid(slave_pos))));
        }
        }

        
    }

}

template<class C>
int EC_boards_centAC_urdf::xddp_input ( C &user_cmd ) {

    static int	bytes_cnt;
    int		bytes = 0;


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
                    break;
                case 0 :
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
            default:
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
