#include <signal.h>
#include <sys/mman.h>
#include <execinfo.h>
#include <stdio.h>
#include <stdint.h>

#include <iostream>

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"


#ifdef __COBALT__
    #include <rtdm/rtdm.h>
#endif

#include <boost/circular_buffer.hpp>
#include <boost/tokenizer.hpp>

#include <iit/ecat/advr/ec_boards_iface.h>
#include <ati_iface.h>

extern void main_common ( int *argcp, char *const **argvp, __sighandler_t sig_handler );


using namespace iit::ecat::advr;

static int run_loop = 1;

typedef struct {
    uint64_t    ts;
    float       ati[6];
    float       iit[6];
    float       dummy[6];
    void sprint ( char *buff, size_t size ) {
        snprintf ( buff, size, "%lu\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f", ts,
// big sensor
//                   iit[0],iit[1],iit[2],iit[3],iit[4],iit[5],
//                   -ati[0],ati[1],ati[2],-ati[3],ati[4],ati[5] );
// small sensor
                 iit[0],iit[1],iit[2],iit[3],iit[4],iit[5],
                 ati[0],ati[1],ati[2],ati[3],ati[4],ati[5]);
    }
    void fprint ( FILE *fp ) {
        fprintf ( fp, "%lu\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", ts,
// big sensor
//                  iit[0],iit[1],iit[2],iit[3],iit[4],iit[5],
//                  -ati[0],ati[1],ati[2],-ati[3],ati[4],ati[5] );
// small sensor
                iit[0],iit[1],iit[2],iit[3],iit[4],iit[5],
                ati[0],ati[1],ati[2],ati[3],ati[4],ati[5]);
    }
} sens_data_t ;


int load_matrix ( std::string filename, std::vector<std::vector<float>> &cal_matrix ) {

    std::string     line;
    std::fstream    file ( filename, std::ios::in );

    typedef boost::tokenizer<boost::char_separator<char>> Tokenizer;
    boost::char_separator<char> sep ( "," );

    if ( ! file ) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return -1;
    }

    while ( getline ( file, line ) ) {

        Tokenizer info ( line, sep ); // tokenize the line of data

        std::vector<float> values;

        for ( Tokenizer::iterator it = info.begin(); it != info.end(); ++it ) {
            // convert data into double value, and store
            values.push_back ( atof ( it->c_str() ) );
        }
        // store array of values
        cal_matrix.push_back ( values );
    }

    auto it = cal_matrix.begin();
    while ( it != cal_matrix.end() ) {
        std::vector<float> row ( *it );
        std::vector<float>::const_iterator rit = row.begin();
        while ( rit != row.end() ) {
            std::cout << ( *rit ) << "\t";
            rit ++;
        }
        std::cout << std::endl;
        it ++;
    }

    
    return 0;
}

static void warn_upon_switch ( int sig __attribute__ ( ( unused ) ) ) {
    // handle rt to nrt contex switch
    void *bt[3];
    int nentries;

    /* Dump a backtrace of the frame which caused the switch to
       secondary mode: */
    nentries = backtrace ( bt,sizeof ( bt ) /sizeof ( bt[0] ) );
    // dump backtrace
    backtrace_symbols_fd ( bt,nentries,fileno ( stdout ) );
}

static void shutdown ( int sig __attribute__ ( ( unused ) ) ) {
    run_loop = 0;
    DPRINTF ( "got signal .... Shutdown\n" );
}

static void set_signal_handler ( void ) {
    signal ( SIGINT, shutdown );
    signal ( SIGINT, shutdown );
    signal ( SIGKILL, shutdown );
#ifdef __COBALT__
    // call pthread_set_mode_np(0, PTHREAD_WARNSW) to cause a SIGXCPU
    // signal to be sent when the calling thread involontary switches to secondary mode
    signal ( SIGXCPU, warn_upon_switch );
#endif
}

///////////////////////////////////////////////////////////////////////////////

using namespace iit::ecat;

Ec_Boards_ctrl * ec_boards_ctrl;


int main ( int argc, char * const argv[] ) {
    int ret;

    boost::circular_buffer<sens_data_t> sens_log;
    sens_log.set_capacity ( LOG_SIZE );

    set_signal_handler();

    if ( argc != 2 ) {
        printf ( "Usage: %s config.yaml\n", argv[0] );
        return 0;
    }

    main_common (&argc, &argv, 0 );
    
    ///////////////////////////////////////////////////////////////////////

    Ati_Sens * ati = new Ati_Sens ( true );

    ///////////////////////////////////////////////////////////////////////

    Ec_Boards_ctrl          * ec_boards_ctrl;
    Ft6ESC                  * ft; 
    Ft6EscPdoTypes::pdo_rx  ft_pdo_rx;
    Ft6EscPdoTypes::pdo_tx  ft_pdo_tx;

    ec_boards_ctrl = new Ec_Boards_ctrl ( argv[1] );

    if ( ec_boards_ctrl->init() != EC_BOARD_OK ) {
        std::cout << "Error in boards init()... cannot proceed!" << std::endl;
        delete ec_boards_ctrl;
        return 0;
    }

    const YAML::Node config = YAML::LoadFile ( argv[1] );
    const YAML::Node ati_config = config["ati_config"];
    std::vector<std::vector<float>> cal_matrix;
    int ft_ecat_pos = 0;
    if ( ati_config ) {
        
        ft_ecat_pos = ati_config["ft_ecat_pos"].as<int>();
        ft = ec_boards_ctrl->slave_as<Ft6ESC>(ft_ecat_pos);
        if ( ! ft ) {
            std::cout << "No Ft6ESC at pos " << ft_ecat_pos << std::endl;
            delete ec_boards_ctrl;
            return 0;
        }
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

    sleep(1.0);
    
    ///////////////////////////////////////////////////////////////////////

   
    ati_log_t   sample;
    sens_data_t sens_data;

    if ( ec_boards_ctrl->set_operative() <= 0 ) {
        std::cout << "Error in boards set_operative()... cannot proceed!" << std::endl;
        delete ec_boards_ctrl;
        return 0;
    }

    //sleep(3);

    uint64_t    start = get_time_ns();
    ec_timing_t timing;
    
    while ( run_loop ) {

        ec_boards_ctrl->recv_from_slaves(timing);
        ati->get_last_sample ( sample );
        for ( int i=0; i < 6; i++ ) sample.ft[i] = sample.ft[i] / 1000 ;
        ///////////////////////////////////////////////////////////////////////

        //ec_boards_ctrl->getRxPDO ( FT_ECAT_POS, ft_pdo_rx );
        ft_pdo_rx = ft->getRxPDO();
        
        //DPRINTF("IIT\n");
        //ft_pdo_rx.fprint(stderr);
        //DPRINTF("ATI\n");
        //sample.fprint(stderr);
        //DPRINTF("\n");

        sens_data.ts = get_time_ns() - start;
        memcpy ( ( void* ) &sens_data.ati, &sample.ft, sizeof ( float ) * 6 );
        memcpy ( ( void* ) &sens_data.iit, &ft_pdo_rx.force_X, sizeof ( float ) * 6 );
        sens_log.push_back ( sens_data );
        
        if ( fabs(sample.ft[0]) > 300 || 
             fabs(sample.ft[1]) > 300 ||
             fabs(sample.ft[2]) > 300 ||
             fabs(sample.ft[3]) > 15 ||
             fabs(sample.ft[4]) > 15 ||
             fabs(sample.ft[5]) > 15 ) { printf(ANSI_COLOR_RED); }
        sens_data.fprint ( stderr );
        printf(ANSI_COLOR_RESET);
        
        ///////////////////////////////////////////////////////////////////////

        ec_boards_ctrl->send_to_slaves(true);

    }

    delete ec_boards_ctrl;
    delete ati;

    /////////////////////////////////////////////////////////////////
    
    if ( ati_config["dump_log"].as<bool>() ) {
        std::string filename = "/tmp/sens.txt";
        dump_buffer ( filename, sens_log );
    }
    return 0;
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
