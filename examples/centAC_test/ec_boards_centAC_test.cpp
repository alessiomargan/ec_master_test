#include <ec_boards_centAC_test.h>
#include <iit/advr/centauro_robot_id.h>

#define MID_POS(m,M)    (m+(M-m)/2)

#define Xt(s)   std::initializer_list<double> { 0, s }

static const std::vector<double> Xt_1s = std::initializer_list<double> { 0, 1 };
static const std::vector<double> Xt_2s = std::initializer_list<double> { 0, 2 };
static const std::vector<double> Xt_3s = std::initializer_list<double> { 0, 3 };
static const std::vector<double> Xt_5s = std::initializer_list<double> { 0, 5 };

static const std::vector<double> Xt3_2s = std::initializer_list<double> { 0, 1, 2 };
static const std::vector<double> Xt4_3s = std::initializer_list<double> { 0, 1, 2, 3 };
static const std::vector<double> Xt5_4s = std::initializer_list<double> { 0, 1, 2, 3, 4 };
static const std::vector<double> Xt5_16s = std::initializer_list<double> { 0, 4, 8, 12, 16 };
static const std::vector<double> Xt7_6s = std::initializer_list<double> { 0, 1, 2, 3, 4, 5, 6 };

static const std::vector<double> Xt6_2s = std::initializer_list<double> { 0, 0.05, 0.1, 1.9, 1.95, 2 };
static const std::vector<double> Xt6_1s = std::initializer_list<double> { 0, 0.05, 0.1, 1.2, 1.35, 1.5 };

using namespace iit::ecat::advr;

EC_boards_centAC_test::EC_boards_centAC_test ( const char* config_yaml ) :
    Ec_Thread_Boards_base ( config_yaml ) {

    name = "centauro_test";
    // not periodic
    period.period = {0,1};

#ifdef __COBALT__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max ( schedpolicy )-10;
    stacksize = ECAT_PTHREAD_STACK_SIZE;

}

EC_boards_centAC_test::~EC_boards_centAC_test() {

}

/*
 *
 */
void EC_boards_centAC_test::init_preOP ( void ) {

    Motor * moto;
    int     slave_pos, motor_start;
    float   min_pos, max_pos, link_pos, motor_pos;

    std::vector<int> pos_rid = centauro::robot_mcs_ids;
    std::vector<int> no_control = std::initializer_list<int> {
        centauro::RA_HA,
        centauro::LA_HA,

//         centauro::WAIST_Y,
//         centauro::RA_SH_1,
//         centauro::RA_SH_2,
//         centauro::RA_SH_3,
//         centauro::RA_EL,
//         centauro::RA_WR_1,
//         centauro::RA_WR_2,
//         centauro::RA_WR_3,
//         centauro::LA_SH_1,
//         centauro::LA_SH_2,
//         centauro::LA_SH_3,
//         centauro::LA_EL,
//         centauro::LA_WR_1,
//         centauro::LA_WR_2,
//         centauro::LA_WR_3,

    };
        
    remove_rids_intersection(pos_rid, no_control);

    get_esc_map_byclass ( left_arm,     centauro::robot_left_arm_ids );
    DPRINTF ( "found %lu <Motor> left_arm\n", left_arm.size() );
    
    get_esc_map_byclass ( right_arm,    centauro::robot_right_arm_ids );
    DPRINTF ( "found %lu <Motor> right_arm\n", right_arm.size() );
    
    get_esc_map_byclass ( waist,        centauro::robot_waist_ids );
    DPRINTF ( "found %lu <Motor> waist\n", waist.size() );
    
    get_esc_map_byclass ( left_front_leg,   centauro::robot_left_front_leg_ids );
    DPRINTF ( "found %lu <Motor> left_front_leg\n", left_front_leg.size() );
    
    get_esc_map_byclass ( left_hind_leg,    centauro::robot_left_hind_leg_ids );
    DPRINTF ( "found %lu <Motor> left_hind_leg\n", left_hind_leg.size() );
    
    get_esc_map_byclass ( right_front_leg,   centauro::robot_right_front_leg_ids );
    DPRINTF ( "found %lu <Motor> right_front_leg\n", right_front_leg.size() );
    
    get_esc_map_byclass ( right_hind_leg,    centauro::robot_right_hind_leg_ids );
    DPRINTF ( "found %lu <Motor> right_hind_leg\n", right_hind_leg.size() );

    // !!!
    get_esc_map_byclass ( motors_to_start,  pos_rid );

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
        home[slave_pos] = DEG2RAD ( centauro::robot_ids_home_pos_deg.at(pos2Rid(slave_pos)) );

        DPRINTF ( ">> Joint_id %d motor %f link %f start %f home %f\n",
                  pos2Rid ( slave_pos ), motor_pos, link_pos, start_pos[slave_pos], home[slave_pos] );

        Ys =  std::initializer_list<double> { start_pos[slave_pos], home[slave_pos] };
        trj_start2home[slave_pos] = std::make_shared<advr::Smoother_trajectory>( Xt_5s, Ys );
#if 0        
        Ys = std::initializer_list<double> { 
            home[slave_pos],
            DEG2RAD ( centauro::robot_ids_zero_pos_deg.at(pos2Rid(slave_pos)) )
        };
        trj_home2zero[slave_pos] = std::make_shared<advr::Smoother_trajectory>( Xt_2s, Ys );
        
        Ys = std::initializer_list<double> { 
            DEG2RAD ( centauro::robot_ids_zero_pos_deg.at(pos2Rid(slave_pos)) ),
            DEG2RAD ( centauro::robot_ids_up_pos_deg.at(pos2Rid(slave_pos)) ),
            DEG2RAD ( centauro::robot_ids_extend_pos_deg.at(pos2Rid(slave_pos)) ),
            DEG2RAD ( centauro::robot_ids_up_pos_deg.at(pos2Rid(slave_pos)) ),
            DEG2RAD ( centauro::robot_ids_zero_pos_deg.at(pos2Rid(slave_pos)) ),
        };
        trj_zero2up2extend2zero[slave_pos] = std::make_shared<advr::Smoother_trajectory> ( Xt5_4s, Ys );
#endif        
        //////////////////////////////////////////////////////////////////////////
        // start controller :
        // - read actual joint position and set as pos_ref
        motor_start = moto->start ( CTRL_SET_POS_MODE );
    }

    DPRINTF ( ">>> wait xddp terminal ....\n" );
    DPRINTF ( ">>> from another terminal run ec_master_test/scripts/xddp_term.py\n" );
    char c; while ( termInXddp.xddp_read ( c ) <= 0 ) { osal_usleep(100); }  
    
    trj_queue.clear();
    //trj_queue.push_back ( trj_start2home );
    //trj_queue.push_back ( trj_home2zero );
    //trj_queue.push_back ( trj_zero2up2extend2zero );
        
}


void EC_boards_centAC_test::init_OP ( void ) {

    try { advr::reset_trj ( trj_queue.at(0) ); }
    catch ( const std::out_of_range &e ) {
        //throw std::runtime_error("Oh my gosh  ... trj_queue is empty !");
        DPRINTF ("Oh my gosh  ... trj_queue is empty !");
    }    
    
    DPRINTF ( "End Init_OP\n" );
    
}

int EC_boards_centAC_test::user_loop ( void ) {

    static iit::advr::Ec_board_base_input pbEcInput;
    static uint64_t count;
    const float trj_error = 0.07;

    if ( ( count++ ) % 10000 == 0 ) {
        DPRINTF ( "alive %ld\n", count/1000 );
    }

    ///////////////////////////////////////////////////////
    //
    if ( xddp_input ( pbEcInput ) > 0 ) {
        //DPRINTF ( ">> %s\n", pbEcInput. );
    }

    if ( ! trj_queue.empty() ) { 
        if ( go_there ( motors_to_start, trj_queue.at(0), trj_error, false) ) {
            // running trj has finish ... remove from queue  !!
            DPRINTF ( "running trj has finish ... remove from queue !!\n" );
            trj_queue.pop_front();
            try { advr::reset_trj ( trj_queue.at(0) ); }
            catch ( const std::out_of_range &e ) {
                // add trajectory ....
                trj_queue.push_back ( trj_zero2up2extend2zero );
                advr::reset_trj ( trj_queue.at(0) );
            }
        }
    }
}

void EC_boards_centAC_test::tune_gains( std::vector<float> gains_incr ) {
    
    std::ostringstream oss;
    Motor::motor_pdo_rx_t motor_pdo_rx;
    Motor::motor_pdo_tx_t motor_pdo_tx;
    
    Motor * moto = slave_as_Motor( rid2Pos(centauro::WAIST_Y) );
    if ( moto ) {
        motor_pdo_tx = moto->getTxPDO();
        
        motor_pdo_tx.gain_0 += gains_incr[0];
        motor_pdo_tx.gain_1 += gains_incr[1];
        motor_pdo_tx.gain_2 += gains_incr[2];
        motor_pdo_tx.gain_3 += gains_incr[3];
        motor_pdo_tx.gain_4 += gains_incr[4];
        moto->setTxPDO(motor_pdo_tx);
        oss << motor_pdo_tx;
        DPRINTF ( "\ttx_pdo %s\n", oss.str().c_str() );
    }
}


int EC_boards_centAC_test::xddp_input ( iit::advr::Ec_board_base_input &pbEcInput ) {

        uint32_t msg_size;
        uint8_t  msg_buff[1024];
        int nbytes;
#if 0
        
        nbytes = read ( termInXddp.get_fd(), ( void* ) &msg_size, sizeof ( msg_size ) );
        if ( nbytes > 0 ) {
            nbytes += read ( termInXddp.get_fd(), ( void* ) msg_buff, msg_size );
            pbEcInput.ParseFromArray(msg_buff, msg_size);
            std::cout << pbEcInput.type() << " "  << std::endl;
        }
#else
    char c;
    std::vector<float> gains_incr(5);
    if ( termInXddp.xddp_read ( c ) > 0 ) {
        switch ( c ) {
            case 'p':
                gains_incr[0] = -10;
                break;
            case 'P':
                gains_incr[0] = 10;
                break;
            case 'd':
                gains_incr[1] = -1;
                break;
            case 'D':
                gains_incr[1] = 1;
                break;
        }
        tune_gains(gains_incr);
    }
#endif

    return nbytes;
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
