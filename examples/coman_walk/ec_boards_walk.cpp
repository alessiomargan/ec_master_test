#include <ec_boards_walk.h>
#include <iit/advr/coman_robot_id.h>

#include <ImuData.h>

#include <RTControl.h>
extern void test_joint ( std::vector<float> homePos, int size, double freq, float r_pos[] );
extern void JointTestState ( char cmd );

#define MID_POS(m,M)    (m+(M-m)/2)

// home position in degree
static const std::vector<float> homePos = {
    // lower body #15
//    0, -1,  0,  0,  0,  0,  0,   0,  0,  0,  0,   0,   0,  0,  0,
    0, 0,  0,  0,  0,  0,  0,   0,  0,  0,  0,   0,   0,  0,  0,
//  1,  2,  3,  4,  5,  6,  7,   8,  9, 10,  11, 12,  13, 14, 15
    // upper body #10 right arm to left arm, last 2 are right and left neck
    30, 70,  0,  -100,  30, -70,   0,  -100,  0,  0,  0,  0,  0,  0,  0,  0,  0
};
// 16, 17, 18,  19,   20,   21,   22,  23, 24, 25, 26, 27, 28, 29, 30, 31, 32

// //
// //
using namespace iit::ecat::advr::coman;
std::map<int, float> walking_robot_home_pos_deg = {

    {WAIST_Y, homePos[0]}, {WAIST_P, homePos[1]}, {WAIST_R, homePos[2]},

    {RL_H_R, 0.0}, {RL_H_Y, 0.0}, {RL_H_P, -0.5}, {RL_K, 1.0}, {RL_A_P, -0.5}, {RL_A_R, 0.0},

    {LL_H_R, 0.0}, {LL_H_Y, 0.0}, {LL_H_P, -0.5}, {LL_K, 1.0}, {LL_A_P, -0.5}, {LL_A_R, 0.0},

    {RA_SH_1, homePos[15]}, {RA_SH_2, homePos[16]}, {RA_SH_3, 0.0}, {RA_EL, homePos[18]}, {RA_WR_1, 0.0}, {RA_WR_2, 0.0}, {RA_WR_3, 0.0}, {RA_HA, 0.0},

    {LA_SH_1, homePos[19]}, {LA_SH_2, homePos[20]}, {LA_SH_3, 0.0}, {LA_EL, homePos[22]}, {LA_WR_1, 0.0}, {LA_WR_2, 0.0}, {LA_WR_3, 0.0}, {LA_HA, 0.0}

};

static const std::vector<double> Xt_2s = std::initializer_list<double> { 0, 2 };
static const std::vector<double> Xt_3s = std::initializer_list<double> { 0, 3 };
static const std::vector<double> Xt_5s = std::initializer_list<double> { 0, 5 };
static const std::vector<double> Xt_9s = std::initializer_list<double> { 0, 9 };



EC_boards_walk::EC_boards_walk ( const char* config_yaml ) : Ec_Thread_Boards_base ( config_yaml ) {

    name = "EC_boards_walk";
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
    jsInXddp.init ( "EC_board_js_input" );
    keyInXddp.init ( "EC_board_key_input" );
    imuInXddp.init ( "Lpms_imu" );

    const YAML::Node config = get_config_YAML_Node();
    dt = config["ec_board_ctrl"]["sync_cycle_time_ns"].as<double>();

}

EC_boards_walk::~EC_boards_walk() {
    iit::ecat::print_stat ( s_loop );
}

void EC_boards_walk::init_preOP ( void ) {

    iit::ecat::advr::Motor * moto;
    int slave_pos;
    float min_pos, max_pos, pos_ref_fb;

    // initialize members class
    leftFoot = fts[rid2Pos(iit::ecat::advr::coman::LL_FT)];
    assert ( leftFoot );
    rightFoot = fts[rid2Pos(iit::ecat::advr::coman::RL_FT)];
    assert ( rightFoot );


    std::vector<double> Ys;

    for ( auto const& item : motors ) {
        slave_pos = item.first;
        moto = item.second;
        moto->readSDO ( "Min_pos", min_pos );
        moto->readSDO ( "Max_pos", max_pos );
        moto->readSDO ( "link_pos", start_pos[slave_pos] );
        // home
        home[slave_pos] = DEG2RAD ( walking_robot_home_pos_deg[pos2Rid ( slave_pos )] );
        DPRINTF ( "Joint_id %d start %f home %f\n", pos2Rid ( slave_pos ), start_pos[slave_pos], home[slave_pos] );

        Ys =  std::initializer_list<double> { start_pos[slave_pos], home[slave_pos] };
        spline_start2home[slave_pos].set_points ( Xt_3s, Ys );

        //////////////////////////////////////////////////////////////////////////
        // start controller :
        // - read actual joint position and set as pos_ref
        moto->start ( CTRL_SET_POS_MODE );
    }

    //
    q_spln.push ( &spline_start2home );

#if DEMO_TYPE==2
    std::cout<<"========================= walking test ============================="<<std::endl;
    InitializeWalkState();
    SetInitialFlag();
#elif DEMO_TYPE==1
    std::cout<<"========================= joint test ============================="<<std::endl;
#else
#error "DEMO_TYPE not defined !!"
#endif

}

void EC_boards_walk::init_OP ( void ) {

    if ( ! q_spln.empty() ) {
        running_spline = q_spln.front();
        last_run_spline = running_spline;
        advr::reset_spline_trj ( *running_spline );
// 	while ( ! go_there(motors, *running_spline, 0.05, true) ) { usleep(1000); }
    }
    DPRINTF ( "End Init_OP\n" );


//     if ( ! q_spln.empty() ) {
// 		running_spline = q_spln.front();
// 		if ( running_spline ) {
// 		    if ( go_there(motors, *running_spline, 0.05, false) ) {
// 			// running spline has finish !!
// 			last_run_spline = running_spline;
// 			q_spln.pop();
// 				if ( ! q_spln.empty() ) {
// 				    running_spline = q_spln.front();
// 				    smooth_splines_trj(*running_spline, *last_run_spline);
// 				    advr::reset_spline_trj(*running_spline);
// 				}
// 		    }
// 		}
// 		else {
// 		    DPRINTF("Error NULL running spline ... pop it\n");
// 		    q_spln.pop();
// 		}
//     }
}


template<class C>
int EC_boards_walk::user_input ( C &user_cmd ) {

    static int	bytes_cnt;
    int		bytes;
    ImuData		lpms_data;

//    if ( (bytes = jsInXddp.xddp_read(user_cmd)) <= 0 ) {
    // return bytes;
//    }

    if ( ( bytes = keyInXddp.xddp_read ( user_cmd ) ) > 0 ) {
        DPRINTF ( ">> %c\n",user_cmd );
#if DEMO_TYPE==2
        KeyBoardControl ( user_cmd );
#elif DEMO_TYPE==1
        JointTestState ( user_cmd );
#else
#error "KeyBoardControl not defined !!"
#endif
    }

    bytes_cnt += bytes;
    // DPRINTF(">> %d %d\n",bytes, bytes_cnt);
    //DPRINTF(">> %d\n",cmd.value);
    //


    ///////////////////////////////////////////////////////
    //
    if ( ( bytes = imuInXddp.xddp_read ( lpms_data ) ) > 0 ) {
        printf ( "Timestamp=%f, qW=%f, qX=%f, qY=%f, qZ=%f\n",
                 lpms_data.timeStamp, lpms_data.q[0], lpms_data.q[1], lpms_data.q[2], lpms_data.q[3] );

    }

    bytes_cnt += bytes;

    return bytes;
}

bool IsEnterLoop = false;
int idx = 0;
int EC_boards_walk::user_loop ( void ) {



    // input_t what;
    char what;
    user_input ( what );

    if ( ! q_spln.empty() ) {

        running_spline = q_spln.front();
        if ( running_spline ) {
            DPRINTF ( "Moving to home positions ..........................%d\n",idx++ );
            if ( go_there ( motors, *running_spline, 0.05, false ) ) {
                // running spline has finish !!
                last_run_spline = running_spline;
                q_spln.pop();
                if ( ! q_spln.empty() ) {
                    running_spline = q_spln.front();
                    smooth_splines_trj ( *running_spline, *last_run_spline );
                    advr::reset_spline_trj ( *running_spline );
                }
            }
        } else {
            DPRINTF ( "Error NULL running spline ... pop it\n" );
            q_spln.pop();
        }
    } else {
//       DPRINTF("Finish moving to home positions ..........................\n");
        if ( !IsEnterLoop ) {
            DPRINTF ( "First time enters user loop!.........................\n" );
            IsEnterLoop=true;
        }

#if DEMO_TYPE==2
        user_loop_walk();
#elif DEMO_TYPE==1
        user_loop_test_joint();
#else
#error "DEMO_TYPE not defined !!"
#endif

    }

}

bool IsEnterWalkLoop = false;
int EC_boards_walk::user_loop_walk ( void ) {

    if ( !IsEnterWalkLoop ) {
        DPRINTF ( "First time enters walk loop!.........................\n" );
        IsEnterWalkLoop=true;
    }

    static double   RTtime=0.0;
    static float    FTSensor[12];
    static char		buff[256];
    static float	rtControl_pos[32];
    int rId;
    static double RT_start_time = start_time;


    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::LpESC::pdo_rx_t motor_pdo_rx;
    iit::ecat::advr::Ft6ESC::pdo_rx_t ft6_pdo_rx;

    ft6_pdo_rx = rightFoot->getRxPDO();
    ft6_pdo_rx.sprint ( buff, sizeof ( buff ) );
    //DPRINTF("Right %s", buff);
    FTSensor[0] = ft6_pdo_rx.force_X * 0.001;
    FTSensor[1] = ft6_pdo_rx.force_Y * 0.001;
    FTSensor[2] = ft6_pdo_rx.force_Z * 0.001;
    FTSensor[3] = ft6_pdo_rx.torque_X * 0.001;
    FTSensor[4] = ft6_pdo_rx.torque_Y * 0.001;
    FTSensor[5] = ft6_pdo_rx.torque_Z * 0.001;

    ft6_pdo_rx = leftFoot->getRxPDO();
    ft6_pdo_rx.sprint ( buff, sizeof ( buff ) );
    //DPRINTF("Left %s", buff);
    FTSensor[6] = ft6_pdo_rx.force_X * 0.001;
    FTSensor[7] = ft6_pdo_rx.force_Y * 0.001;
    FTSensor[8] = ft6_pdo_rx.force_Z * 0.001;
    FTSensor[9] = ft6_pdo_rx.torque_X * 0.001;
    FTSensor[10] = ft6_pdo_rx.torque_Y * 0.001;
    FTSensor[11] = ft6_pdo_rx.torque_Z * 0.001;

//     RTtime = (iit::ecat::get_time_ns()-start_time)/1e9;

//     //////////////// walking pattern ////////////////////////////
    RTControl ( RTtime, FTSensor, homePos, homePos.size(), rtControl_pos );
//     ////////////////////////////////////////////////////////////
//
//             std::cout << setprecision(8);
//         std::cout <<RTtime<<" ";

// 	for (int i = 3; i < 15; ++i){
//            std::cout << setw(5) <<57.3*(float(rtControl_pos[i]))<<'\t';
//         }
//         std::cout<<std::endl;
// //

//     for ( auto const& item : motors ) {
    for ( auto it = motors.begin(); it != motors.end(); it++ ) {
        moto = it->second;
        rId = pos2rid[it->first]-1;
        if ( rId >= 3 &&  rId < 15 ) {
            moto->set_posRef ( float ( rtControl_pos[rId] ) );
            //DPRINTF(" %f", float ( rtControl_pos[rId]) );
        }
    }
    //DPRINTF("\n");


    return 0;
}

int EC_boards_walk::user_loop_test_joint ( void ) {

    static float	rtControl_pos[32];
    static double	RTtime;
    int rId;

    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::LpESC::pdo_rx_t motor_pdo_rx;

    RTtime = ( iit::ecat::get_time_ns()-start_time ) /1e9;

    if ( RTtime > 3 ) {

        test_joint ( homePos, homePos.size(), 0.03, rtControl_pos );

        //set_position(_pos, sizeof(_pos));
        for ( auto it = motors.begin(); it != motors.end(); it++ ) {
            moto =  it->second;
// 	    motor_pdo_rx = moto->getRxPDO();
// 	    // pos_ref_fb is the previous reference
// 	    moto->set_posRef(motor_pdo_rx.pos_ref_fb);
// 	    // !! FIXME use rtControl_pos value
// 	    // !! NOTE set_posRef use rad
// 	    //moto->set_posRef(rtControl_pos[]);

            rId = pos2rid[it->first]-1;
//             if(   rId <= 30 )
            {
                moto->set_posRef ( rtControl_pos[rId] );
            }
        }
    }

    return 0;
}
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
