//#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <exception>
#include <iostream>

#include <iit/ecat/ec_master_iface.h>
#include <iit/ecat/advr/pipes.h>
#include <iit/advr/thread_util.h>

extern void main_common ( int *argcp, char *const **argvp, __sighandler_t sig_handler );
extern void set_main_sched_policy ( int );

static int main_loop = 1;

void shutdown ( int sig __attribute__ ( ( unused ) ) ) {
    main_loop = 0;
    DPRINTF ( "got signal .... Shutdown\n" );
}


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

    UI_thread(std::string _pipe_name):pipe_name(_pipe_name) {

        name = "UI_thread";
        // periodic
        period.period = {0,100};

        schedpolicy = SCHED_OTHER;
        priority = sched_get_priority_max ( schedpolicy ) /2;
        stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    }

    ~UI_thread() {
        close(xddp_fd);
        //iit::ecat::print_stat ( loop_time );
    }

    virtual void th_init ( void * ) {
        int retry = 10;
        std::string pipe ( pipe_prefix + pipe_name );
        while ( retry-- && xddp_fd <= 0 ) {
            sleep(1);
            std::cout << "... try opening " << pipe << std::endl;
            xddp_fd = open ( pipe.c_str(), O_RDONLY |O_NONBLOCK );
        }
        if (xddp_fd <= 0) { exit(0); }
    }
    virtual void th_loop ( void * ) {
        
        int nbytes = read ( xddp_fd, ( void* ) &timing, sizeof ( timing ) );
        if ( nbytes > 0 ) {
            std::cout << "[UI] ec_timing_t " <<  timing.loop_time <<" "<< timing.offset <<" "<< timing.recv_dc_time << std::endl;
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
public:

    EC_thread(std::string _ecat_iface, std::string _pipe_name)
    :ecat_iface(_ecat_iface),pipe_name(_pipe_name) {

        name = "EC_thread";
        // non periodic
        period.period = {0,1};

#ifdef __COBALT__
        schedpolicy = SCHED_FIFO;
#else
        schedpolicy = SCHED_OTHER;
#endif
        priority = sched_get_priority_max ( schedpolicy );
        stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    }

    ~EC_thread() {
        iit::ecat::finalize();
        iit::ecat::print_stat ( loop_time );
    }

    virtual void th_init ( void * ) {
        uint32_t sync_cycle_time_ns = 1e6;
        //uint32_t sync_cycle_time_ns = 1e9;
        uint32_t sync_cycle_offset_ns = 0; //1e9;
        
        OutXddp.init(pipe_name);
        iit::ecat::initialize ( ecat_iface.c_str() );
        expected_wkc = iit::ecat::operational ( sync_cycle_time_ns, sync_cycle_offset_ns );
    }
    virtual void th_loop ( void * ) {
        int wkc = iit::ecat::recv_from_slaves ( &timing );
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
        DPRINTF ( "Usage: %s eth_name\n", argv[0] );
        return 0;
    }

    std::string iface = argv[1]; 

    main_common ( &argc, &argv, shutdown );
    
    threads["UI_thread"] = new UI_thread(std::string("ec_timing"));
    threads["EC_thread"] = new EC_thread(iface, std::string("ec_timing"));
    
    threads["EC_thread"]->create();
    threads["UI_thread"]->create();

    while ( main_loop ) {

        sleep(1);
    }

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
