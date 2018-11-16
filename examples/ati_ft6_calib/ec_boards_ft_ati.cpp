#include <ec_boards_ft_ati.h>


using namespace iit::ecat::advr;

Ec_Boards_ft_ati::Ec_Boards_ft_ati ( const char* config_yaml ) : Ec_Thread_Boards_base ( config_yaml ) {

    name = "Ec_Boards_ft_ati";
    // non periodic
    period.period = {0,1};

#ifdef __COBALT__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_RR;
#endif
    priority = sched_get_priority_max ( schedpolicy );
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    sens_log.set_capacity ( LOG_SIZE );
    
    ati = new iit::advr::Ati_Sens ( );
    ati->config( true );

}

Ec_Boards_ft_ati::~Ec_Boards_ft_ati() {
    
    delete ati;

    if ( ati_config["dump_log"].as<bool>() ) {
        std::string filename = "/tmp/sens.txt";
        iit::ecat::dump_buffer ( filename, sens_log );
    }

    std::cout << "~" << typeid ( this ).name() << std::endl;
}

void Ec_Boards_ft_ati::init_preOP ( void ) {

    ati_config = get_config_YAML_Node()["ati_config"];
    std::vector<std::vector<float>> cal_matrix;
    int ft_ecat_pos = 0;
    
    if ( ati_config ) {
        
        ft_ecat_pos = ati_config["ft_ecat_pos"].as<int>();
        ft = slave_as<Ft6ESC>(ft_ecat_pos);
        if ( ! ft ) {
            throw iit::ecat::advr::EcBoardsError(0,"No Ft6ESC at pos " + std::to_string(ft_ecat_pos));
        }
        
        ftAtiOutXddp.init( "NoNe@Ft_id_"+std::to_string ( ft->get_robot_id() ) );
        
        set_flash_cmd_X ( ft, CTRL_REMOVE_TORQUE_OFFS );
        try {
            std::string calib_file = ati_config["calib_mat_file"].as<std::string>();
            if ( ! load_matrix( calib_file, cal_matrix ) && ati_config["save_calib_matrix"].as<bool>() ) {
                ft->set_cal_matrix ( cal_matrix );
                set_flash_cmd_X ( ft, FLASH_SAVE );
            }
        } catch ( YAML::Exception &e ) {
            DPRINTF ( "Catch Exception ... %s\n", e.what() );
        } 
    }

}

void Ec_Boards_ft_ati::init_OP ( void ) {

    start = iit::ecat::get_time_ns();
}

template<class C>
int Ec_Boards_ft_ati::user_input ( C &user_cmd ) {

}

int Ec_Boards_ft_ati::user_loop ( void ) {
    
    //
    sens_data.ts = iit::ecat::get_time_ns() - start;
    
    // get ati data
    ati->get_last_sample ( sample );
    for ( int i=0; i < 6; i++ ) sample.ft[i] = sample.ft[i] / 1000 ;
    memcpy ( ( void* ) &sens_data.ati, &sample.ft, sizeof ( float ) * 6 );
    
    // get ft data
    auto ft_pdo_rx = ft->getRxPDO();
    memcpy ( ( void* ) &sens_data.iit, &ft_pdo_rx.force_X, sizeof ( float ) * 6 );

    sens_log.push_back ( sens_data );

    serializeToXddp(&sens_data);
    
#if 0
    if ( fabs(sample.ft[0]) > 300 || 
            fabs(sample.ft[1]) > 300 ||
            fabs(sample.ft[2]) > 300 ||
            fabs(sample.ft[3]) > 15 ||
            fabs(sample.ft[4]) > 15 ||
            fabs(sample.ft[5]) > 15 ) { printf(ANSI_COLOR_RED); }
    sens_data.fprint ( stderr );
    printf(ANSI_COLOR_RESET);
#endif
}
   


// kate: indent-mode cstyle; indent-width 4; replace-tabs on