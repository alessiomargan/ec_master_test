#include <ec_ecat_states.h>
#include <iit/ecat/advr/lxm32i_esc.h>
#include <iostream>
#include <fstream>
using namespace iit::ecat::advr;

typedef struct {
    float t;
    float a;
    float b;
    float c;
} row_t;

std::istream& operator >> (std::istream &is, row_t &rhs)
{
  char delim;
  is >> rhs.t
     >> delim
     >> rhs.a
     >> delim
     >> rhs.b
     >> delim
     >> rhs.c
     >> delim;
     return is;
}

static void read_csv(const std::string inFile, std::vector<std::map<int,float>> & _trj_points) {

    std::string line;
    std::ifstream csv_file (inFile);
    std::map<int,float> kv;
    row_t row;
    
    if ( csv_file.is_open() )
    {
        while ( std::getline (csv_file,line) )
        {
            // t,1,2,3
            std::cout << line << '\n';
            std::stringstream linestr (line);
            linestr >> row;
            kv[1] = row.a * 1000;
            kv[2] = (row.b - 0.09) * 1000;
            kv[3] = (row.c - 0.285) * 1000;
            for ( auto const &_kv : kv ) {
                std::cout << _kv.first << " " << _kv.second << " ";
            }
            std::cout << "\n";

            _trj_points.push_back(kv);
        }
        csv_file.close();
    } else std::cout << "*** Unable to open file" << "\n";

//     for ( auto const &item : _trj_points ) {
//         for ( auto const &_kv : item ) {
//             std::cout << _kv.first << " " << _kv.second << " ";
//         }
//         std::cout << "\n";
//     }
    
}

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

    const YAML::Node config_file = get_config_YAML_Node();
    std::vector<int> to_control = config_file["ec_boards_base"]["motor_to_control"].as<std::vector<int32_t>>();
    
    get_esc_map_byclass ( lxm32i, to_control);
    DPRINTF ( "found %lu motors\n", lxm32i.size() );
    
    std::string csv_file = config_file["ec_boards_base"]["trj_file"].as<std::string>();
    read_csv( csv_file, trj_points );
}

void Ec_Ecat_states::init_OP ( void ) {

    LXM32iESC * moto;
    int         slave_pos; 

    // set start and home position
    for ( auto const& item : lxm32i ) {
        slave_pos = item.first;
        moto = item.second;
        if ( slave_pos == 3 ) {
            start_pos[slave_pos] = moto->getRxPDO()._p_act * 32 / 131072;  
        } else {
            start_pos[slave_pos] = moto->getRxPDO()._p_act * 25 / 131072;  
        }
        home[slave_pos] = 0;
    }
        
    first_pos = trj_points[0];
    tp_it = trj_points.end();
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
            
        if ( cmd == 'h' ) {
            
            for ( auto const& item : lxm32i ) {
                slave_pos = item.first;
                moto = item.second;
                if ( slave_pos == 3 ) {
                    start_pos[slave_pos] = moto->getRxPDO()._p_act * 32 / 131072;  
                } else {
                    start_pos[slave_pos] = moto->getRxPDO()._p_act * 25 / 131072;  
                }
                home[slave_pos] = 0;
            }
        }
        
        if ( cmd == 'q' ) {

            tp_it = trj_points.begin();
            
            for ( auto const& item : lxm32i ) {
                slave_pos = item.first;
                moto = item.second;
                if ( slave_pos == 3 ) {
                    start_pos[slave_pos] = moto->getRxPDO()._p_act * 32 / 131072;  
                } else {
                    start_pos[slave_pos] = moto->getRxPDO()._p_act * 25 / 131072;  
                }

                //////////////////////////////////////////////////////////////////////////
                // trajectory
                auto Xs = std::initializer_list<double> { 0, 5 };
                auto Ys = std::initializer_list<double> { start_pos[slave_pos], home[slave_pos] };
                auto Ws = std::initializer_list<double> { home[slave_pos], first_pos[slave_pos] };
                trj_map["start2home"][slave_pos] = std::make_shared<advr::Smoother_trajectory>( Xs, Ys );
                //trj_map["usr_trj"][slave_pos] = std::make_shared<advr::Smoother_trajectory>( moto->trj_Xs, moto->trj_Ys );
                trj_map["first_pos"][slave_pos] = std::make_shared<advr::Smoother_trajectory>( Xs, Ws  );
            }
            
            trj_queue.clear();
            trj_queue.push_back ( trj_map.at("start2home") );
            //trj_queue.push_back ( trj_map.at("usr_trj") );
            trj_queue.push_back ( trj_map.at("first_pos") );
            
            try { advr::reset_trj ( trj_queue.at(0) ); }
            catch ( const std::out_of_range &e ) {
                throw std::runtime_error("Oh my gosh  ... trj_queue is empty !");
            } 
 
        }
    }    
    
    //////////////////////////////////////////////////
    
    
    if ( ! trj_queue.empty() ) {
        if ( go_there ( lxm32i, trj_queue.at(0), 0, false) ) {
            // running trj has finish ... remove from queue  !!
            DPRINTF ( "running trj has finish ... remove from queue !!\n" );
            trj_queue.pop_front();
            try { advr::reset_trj ( trj_queue.at(0) ); }
            catch ( const std::out_of_range &e ) {
                // add trajectory ....
            }
        }
    } else {
        
        if ( tp_it != trj_points.end() ) {
            go_there ( lxm32i, *tp_it, 0, false);
//             for ( auto const &_kv : *tp_it ) {
//                 std::cout << _kv.first << " " << _kv.second << " ";
//             }
//             std::cout << "\n";
            tp_it ++;
        } else {
            tp_it = trj_points.begin();
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


bool Ec_Ecat_states::go_there ( const std::map<int, iit::ecat::advr::LXM32iESC*> &motor_set,
                                const std::map<int,float> &target_pos,
                                float eps, bool debug )
{
    int cond, cond_cnt, cond_sum;
    float pos_ref, motor_err, link_err, motor_link_err;
    int slave_pos;
    //iit::ecat::advr::Motor * moto;
    //iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    LXM32iESC               * moto;
    LXM32iESC::pdo_rx_t     motor_pdo_rx;
    std::vector<int> truth_vect;

    cond = cond_cnt = cond_sum = 0;

    for ( auto const& item : motor_set ) {
        slave_pos = item.first;
        moto =  item.second;

        // check in the target_pos map if the current slave_pos exist
        try {
            pos_ref = target_pos.at ( slave_pos );
        } catch ( const std::out_of_range& oor ) {
            continue;
        }

        motor_pdo_rx = moto->getRxPDO();
        moto->set_pos_target ( pos_ref );

        link_err = 0;           //fabs ( motor_pdo_rx.link_pos  - pos_ref );
        motor_err = 0;          //fabs ( motor_pdo_rx.motor_pos - pos_ref );
        motor_link_err = 0;     //fabs ( motor_pdo_rx.motor_pos - motor_pdo_rx.link_pos );

        cond = ( link_err <= eps || motor_err <= eps ) ? 1 : 0;
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