//#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <exception>
#include <iostream>
#include <yaml-cpp/yaml.h>

#include <iit/ecat/ec_master_iface.h>
#include <iit/ecat/advr/pipes.h>
#include <iit/advr/thread_util.h>

extern void main_common ( int *argcp, char *const **argvp, __sighandler_t sig_handler );

static int main_loop = 1;

void shutdown ( int sig __attribute__ ( ( unused ) ) ) {
    main_loop = 0;
    printf ( "got signal .... Shutdown\n" );
}

#define PIPE_NAME "ec_timing"
    
////////////////////////////////////////////////////
// 
////////////////////////////////////////////////////
class UI_thread : public Thread_hook {

    iit::ecat::stat_t   loop_time;
    uint64_t            tNow, dt;
    int                 xddp_fd;
    iit::ecat::ec_timing_t  timing;
    std::string         pipe_name;
public:

    UI_thread() {

        name = "UI_thread";
        // periodic
        period.period = {0,500};

        schedpolicy = SCHED_OTHER;
        priority = sched_get_priority_max ( schedpolicy );
        stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

        pipe_name = PIPE_NAME;
    }

    ~UI_thread() {
        close(xddp_fd);
        //iit::ecat::print_stat ( loop_time );
    }

    virtual void th_init ( void * ) {
        std::string pipe ( pipe_prefix + pipe_name );
        std::cout << "... try opening " << pipe << std::endl;
        xddp_fd = open ( pipe.c_str(), O_RDONLY |O_NONBLOCK );
        if (xddp_fd <= 0) { exit(0); }
    }
    virtual void th_loop ( void * ) {
        
        int nbytes = read ( xddp_fd, ( void* ) &timing, sizeof ( timing ) );
        if ( nbytes > 0 ) {
            //if ( timing.loop_time > sync_cycle_time_ns *2 ) {
                std::cout << "[UI] ec_timing_t " \
                          << timing.recv_dc_time <<"\t" \
                          << timing.loop_time <<"\t" \
                          << timing.offset <<"\t" \
                          << timing.delta <<"\t" << std::endl;
            //}
        } else {
            //printf ( "Nothing from pipe\n");
        }
        
    }
};

////////////////////////////////////////////////////
// 
////////////////////////////////////////////////////
class EC_thread : public Thread_hook {

    uint64_t                tNow, dt;
    int                     expected_wkc;
    iit::ecat::stat_t       loop_time;
    iit::ecat::ec_timing_t  timing;
    XDDP_pipe               OutXddp;
    std::string             ecat_iface;
    std::string             pipe_name;
    
    iit::ecat::ec_thread_arg_t ec_thread_arg;

public:

    EC_thread(std::string config_file) {

        name = "EC_thread";
        // non periodic
        period.period = {0,1};

#ifdef __COBALT__
        schedpolicy = SCHED_FIFO;
#else
        schedpolicy = SCHED_FIFO;
#endif
        priority = sched_get_priority_max ( schedpolicy );
        stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

        YAML::Node  root_cfg = YAML::LoadFile ( config_file );
        YAML::Node  board_ctrl = root_cfg["ec_board_ctrl"];
        ecat_iface = board_ctrl["eth_iface"].as<std::string>();
        ec_thread_arg.ecat_cycle_ns = board_ctrl["sync_cycle_time_ns"].as<uint32_t>();
        ec_thread_arg.ecat_cycle_shift_ns = board_ctrl["sync_cycle_offset_ns"].as<uint32_t>();
        ec_thread_arg.sync_point_ns = board_ctrl["sync_point_ns"].as<uint32_t>();

        pipe_name = PIPE_NAME;
    }

    ~EC_thread() {
        iit::ecat::finalize();
        iit::ecat::print_stat ( loop_time );
    }

    virtual void th_init ( void * ) {       
        OutXddp.init(pipe_name);
        iit::ecat::initialize ( ecat_iface.c_str(), true );
        pthread_barrier_wait(&threads_barrier);
        expected_wkc = iit::ecat::operational ( ec_thread_arg );
    }
    virtual void th_loop ( void * ) {
        int wkc = iit::ecat::recv_from_slaves ( timing );
        if ( wkc == expected_wkc ) {
            //printf ( "{EC} loop_time %ld\toffset %ld\trecv_dc_time %ld\n", timing.loop_time, timing.offset, timing.recv_dc_time );
            OutXddp.xddp_write( timing );
            loop_time(timing.loop_time);
        } else {
            // wkc != expected_wkc
            if ( wkc > 0) { 
                printf("Oops wkc differs from expected %d != %d\n", wkc, expected_wkc);
            } else {
                printf("Oops %s\n", strerror(-wkc));
            }
        }
    }
};

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main ( int argc, char * const argv[] ) try {

    std::map<std::string, Thread_hook*> threads;
    
    if ( argc != 2 ) {
        DPRINTF ( "Usage: %s yaml file\n", argv[0] );
        return 0;
    }

    std::string config_file(argv[1]);
    std::ifstream fin ( config_file );
    if ( fin.fail() ) {
        DPRINTF ( "Can not open %s\n", config_file.c_str() );
        assert ( 0 );
    }


    sigset_t set;
    int sig;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    main_common ( &argc, &argv, shutdown );
    
    threads["UI_thread"] = new UI_thread();
    threads["EC_thread"] = new EC_thread(config_file);

    pthread_barrier_init(&threads_barrier, NULL, threads.size());

    threads["EC_thread"]->create(true);
    pthread_barrier_wait(&threads_barrier);
    threads["UI_thread"]->create(false,3);

#ifdef __COBALT__
    // here I want to catch CTRL-C 
     __real_sigwait(&set, &sig);
#else
     sigwait(&set, &sig);  
#endif

    for ( auto const &t : threads) {
        t.second->stop();
        t.second->join();
        delete t.second;
    }
    
    std::cout << "Exit main" << std::endl;

    return 0;

} catch ( std::exception& e ) {

    std::cout << "Main catch .... " <<  e.what() << std::endl;

}


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
