#include <ec_ecat_states.h>
#include <iit/ecat/advr/lxm32i_esc.h>

using namespace iit::ecat::advr;

Ec_Ecat_states::Ec_Ecat_states ( const char* config_yaml ) : Ec_Thread_Boards_base ( config_yaml ) {

    name = "EC_ecat_states";
    // non periodic
    period.period = {0,1};

#ifdef __COBALT__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max ( schedpolicy ) - 10 ;
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    // open pipe ... xeno xddp or fifo
    inXddp.init ( "EC_board_input" );

}

Ec_Ecat_states::~Ec_Ecat_states() {
    std::cout << "~" << typeid ( this ).name() << std::endl;
}

void Ec_Ecat_states::init_preOP ( void ) {

#if 0
    DPRINTF("+++++++++++++++++++++\n");
    Ec_Boards_ctrl::shutdown(false);
    // wait boards boot up
    sleep(1);
    // init Ec_Boards_ctrl
    if ( Ec_Boards_ctrl::init() != iit::ecat::advr::EC_BOARD_OK ) {
        throw "something wrong";
    }
#endif

    std::vector<int> to_control = std::initializer_list<int> {
        1,
        2,
        3,
    };

    get_esc_map_byclass ( lxm32i, to_control);
    DPRINTF ( "found %lu motors\n", lxm32i.size() );
    
}

void Ec_Ecat_states::init_OP ( void ) {

    LXM32iESC * moto;
    int         slave_pos; 

    for ( auto const& item : lxm32i ) {
        
        slave_pos = item.first;
        moto = item.second;
        start_pos[slave_pos] = moto->getRxPDO()._p_act * 25 / 131072;  
        home[slave_pos] = 0;
        
        //////////////////////////////////////////////////////////////////////////
        // trajectory
        auto Xs = std::initializer_list<double> { 0, 5 };
        auto Ys = std::initializer_list<double> { start_pos[slave_pos], home[slave_pos] };
        trj_map["start@home"][slave_pos] = std::make_shared<advr::Smoother_trajectory>( Xs, Ys );
        Xs = std::initializer_list<double> { 0, 1, 2, 3 };
        Ys = std::initializer_list<double> { home[slave_pos], 25, 75, home[slave_pos]};
        trj_map["start@point"][slave_pos] = std::make_shared<advr::Smoother_trajectory>( Xs, Ys );
        trj_map["sineFROMstart"][slave_pos] = std::make_shared<advr::Sine_trajectory> ( 3, 25, home[slave_pos], std::initializer_list<double> { 0, 60 } );
                
    }
    
    trj_queue.clear();
    DPRINTF ( "End Init_OP\n" );
    
}

template<class C>
int Ec_Ecat_states::user_input ( C &user_cmd ) {

    static int  bytes_cnt;
    int         bytes;

    if ( ( bytes = inXddp.xddp_read ( user_cmd ) ) <= 0 ) {
        return bytes;
    }

    bytes_cnt += bytes;
    //DPRINTF ( ">> %d %d\n",bytes, bytes_cnt );

    return bytes;
}

int Ec_Ecat_states::user_loop ( void ) {

    int         slave_pos;
    char        cmd = 0;

    LXM32iESC           * moto;
    LXM32iESC::pdo_rx_t pdo_rx;
    advr::Trj_ptr       trj;
    
    if ( user_input( cmd ) > 0 ) {
        
        for ( auto const& item : lxm32i ) {
            slave_pos = item.first;
            moto =  item.second;
            moto->user_loop(cmd);
        }
        
        if ( cmd == 'q' ) {
 
            trj_queue.clear();
            trj_queue.push_back ( trj_map.at("start@home") );
            trj_queue.push_back ( trj_map.at("start@point") );
            trj_queue.push_back ( trj_map.at("sineFROMstart") );
    
            try { advr::reset_trj ( trj_queue.at(0) ); }
            catch ( const std::out_of_range &e ) {
                throw std::runtime_error("Oh my gosh  ... trj_queue is empty !");
            } 
 
        }
    }
    
    
    if ( ! trj_queue.empty() ) {
        if ( go_there ( lxm32i, trj_queue.at(0), 0, false) ) {
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
    
    
}


bool Ec_Ecat_states::go_there ( const std::map<int, LXM32iESC*> &motor_set,
                                const advr::Trj_ptr_map &trj_map,
                                float eps, bool debug )
{
    int cond, cond_cnt, cond_sum;
    float pos_ref, motor_err, link_err, motor_link_err;
    int slave_pos;
    advr::Trj_ptr trj;
    //iit::ecat::advr::Motor * moto;
    //iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    LXM32iESC               * moto;
    LXM32iESC::pdo_rx_t     motor_pdo_rx;
    std::vector<int> truth_vect;

    cond = cond_cnt = cond_sum = 0;

    for ( auto const& item : motor_set ) {
        slave_pos = item.first;
        moto =  item.second;

        // check in the spline_trj map if the current slave_pos exist
        try {
            trj = trj_map.at ( slave_pos );
        } catch ( const std::out_of_range& oor ) {
            continue;
        }

        motor_pdo_rx = moto->getRxPDO();
        pos_ref = (float)(*trj)();
        //pos_ref = ( float ) trj();
        moto->set_pos_target ( pos_ref );

        link_err = 0;         // fabs ( motor_pdo_rx.link_pos  - pos_ref );
        motor_err = 0;        // fabs ( motor_pdo_rx.motor_pos - pos_ref );
        motor_link_err = 0;   // fabs ( motor_pdo_rx.motor_pos - motor_pdo_rx.link_pos );

        cond = ( ( link_err <= eps || motor_err <= eps ) && trj->ended() ) ? 1 : 0;
        cond_cnt++;
        cond_sum += cond;

        if ( debug ) {
            truth_vect.push_back ( cond );
            if ( ! cond ) {
//                 DPRINTF ( "rId %d\tposRef %f \t link %f{%f} \t motor %f{%f} \t |motor-link|{%f}\n",
//                           pos2Rid ( slave_pos ), pos_ref,
//                           motor_pdo_rx.link_pos, link_err,
//                           motor_pdo_rx.motor_pos, motor_err,
//                           motor_link_err );
            }
        }
    }

    if ( debug ) {
        DPRINTF ( "---\n" );
        for ( auto b : truth_vect ) {
            DPRINTF ( "%d ",b );
        }
        DPRINTF ( "\n=^=\n" );
    }

    return ( cond_cnt == cond_sum );
}


// kate: indent-mode cstyle; indent-width 4; replace-tabs on