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

#ifdef __COBALT__
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

    iit::ecat::advr::Motor * moto;
    int slave_pos;
    float min_pos, max_pos, link_pos, motor_pos;

    std::vector<int> test_rid = std::initializer_list<int> {
    
        //103,
        1,
        2,
        //6,
        //7,
        
        // front right
        //41,42,43,44,45,
        
        //51,52,/*53,54,*/55,
        
        //61,62,/*63,64,*/65,
        
        // front left
        //71,72,73,74,75, 
        
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
        assert ( EC_WRP_OK == moto->readSDO ( "motor_pos", motor_pos ));
        assert ( EC_WRP_OK == moto->readSDO ( "link_pos", link_pos ));


        start_pos[slave_pos] = motor_pos; 
        // set home pos
        home[slave_pos] = MID_POS ( min_pos,max_pos );
        //home[slave_pos] = start_pos[slave_pos];
        //home[slave_pos] = DEG2RAD ( centauro::robot_ids_home_pos_deg.at(pos2Rid(slave_pos)) );
        //home[slave_pos] = 0.6;
        //home[slave_pos] = DEG2RAD ( -65 );

        DPRINTF ( ">> Joint_id %d motor %f link %f start %f home %f\n", pos2Rid ( slave_pos ), motor_pos, link_pos, start_pos[slave_pos], home[slave_pos]);
        
        //////////////////////////////////////////////////////////////////////////
        // trajectory
        auto Xs = std::initializer_list<double> { 0, 5 };
        auto Ys =  std::initializer_list<double> { start_pos[slave_pos], home[slave_pos] };
        trj_map["start@home"][slave_pos] = std::make_shared<advr::Smoother_trajectory>( Xs, Ys );
        //trj_map["sineFROMhome"][slave_pos] = std::make_shared<advr::Sine_trajectory> ( 0.1, 3.0, home[slave_pos], std::initializer_list<double> { 0, 60 } );
        trj_map["sineFROMhome"][slave_pos] = std::make_shared<advr::Sine_trajectory> ( 0.5, 0.5, home[slave_pos], std::initializer_list<double> { 0, 60 } );
        //trj_map["sineFROMhome"][slave_pos] = std::make_shared<advr::Sine_trajectory> ( 0.3, 2.0, home[slave_pos], std::initializer_list<double> { 0, 60*30 } );

        //////////////////////////////////////////////////////////////////////////
        // start controller :
        // - read actual joint position and set as pos_ref  
        //assert ( moto->start ( CTRL_SET_MIX_POS_MODE ) == EC_BOARD_OK );
        assert ( moto->start ( CTRL_SET_POS_MODE ) == EC_BOARD_OK );
        //assert ( moto->start ( CTRL_SET_CURR_MODE ) == EC_BOARD_OK );
        //assert ( moto->start ( CTRL_SET_VEL_MODE ) == EC_BOARD_OK );
    }

//     DPRINTF ( ">>> wait xddp terminal ....\n" );
//     DPRINTF ( ">>> from another terminal run ec_master_test/scripts/xddp_term.py\n" );
//     char c; while ( termInXddp.xddp_read ( c ) <= 0 ) { osal_usleep(100); }  
    
    trj_queue.clear();
    trj_queue.push_back ( trj_map.at("start@home") );
    trj_queue.push_back ( trj_map.at("sineFROMhome") );
    
}

void Ec_Boards_sine::init_OP ( void ) {

    try { advr::reset_trj ( trj_queue.at(0) ); }
    catch ( const std::out_of_range &e ) {
        throw std::runtime_error("Oh my gosh  ... trj_queue is empty !");
    }    
    
    DPRINTF ( "End Init_OP\n" );

}


int Ec_Boards_sine::user_loop ( void ) {

    float trj_error = 0.07;
    int what;
    user_input ( what );
   
    if ( ! trj_queue.empty() ) {
        if ( go_there ( motors_to_start, trj_queue.at(0), trj_error, false) ) {
            // running trj has finish ... remove from queue  !!
            DPRINTF ( "running trj has finish ... remove from queue !!\n" );
            trj_queue.pop_front();
            try { advr::reset_trj ( trj_queue.at(0) ); }
            catch ( const std::out_of_range &e ) {
                // add trajectory ....
                //advr::reset_trj ( trj_queue.at(0) );
            }
        }
    }
    
#if 0
    {
        auto moto = slave_as<CentAcESC>(1);
        moto->set_ivRef(0);
    }
#endif
    
}

template<class C>
int Ec_Boards_sine::user_input ( C &user_cmd ) {

    static int  bytes_cnt;
    int     bytes;
    input_t cmd;

    if ( ( bytes = inXddp.xddp_read ( cmd ) ) <= 0 ) {
        return bytes;
    }

    bytes_cnt += bytes;
    DPRINTF ( ">> %d %d\n",bytes, bytes_cnt );
    //DPRINTF(">> %d\n",cmd.value);

    return bytes;
}


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
