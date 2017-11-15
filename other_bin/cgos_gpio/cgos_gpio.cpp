#include <assert.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <exception>
#include <iostream>
#include <map>
#include <iomanip>

#include <iit/advr/thread_util.h>

#include <Cgos.h>


#ifdef __XENO_PIPE__
static const std::string pipe_prefix ( "/proc/xenomai/registry/rtipc/xddp/" );
#else
static const std::string pipe_prefix ( "/tmp/" );
#endif

extern void main_common ( int *argcp, char *const **argvp, __sighandler_t sig_handler );
extern void set_main_sched_policy ( int );



class GPIO_poller : public Thread_hook {

    std::string             pipe_name;
    // board handle
    HCGOS                   hCgos;
    uint32_t                gpioInputValues;
        
public:

    GPIO_poller(std::string _pipe_name):pipe_name(_pipe_name) {

        name = "GPIO_poller";
        // periodic
        period.period = {0,100000};

        schedpolicy = SCHED_OTHER;
        priority = sched_get_priority_max ( schedpolicy ) / 4;
        stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    }

    ~GPIO_poller() {

        // close board
        if (hCgos) CgosBoardClose(hCgos);
        // remove DLL
        CgosLibUninitialize();
    }

    virtual void th_init ( void * ) {

        if ( ! CgosLibInitialize() ) {
            if ( ! CgosLibInstall(1) ) {
                throw std::runtime_error("Oops ... the driver could not be installed. Check your rights");
            }
            // the driver has been installed
            if ( ! CgosLibInitialize() ) {
                throw std::runtime_error("Oops ... the driver still could not be opened, a reboot might be required");
            }
        }
        
        // CgosLibInitialize successful
        
        // open the board
        if ( ! CgosBoardOpen(0,0,0,&hCgos) ) {
            throw std::runtime_error("Oops ... could not open a board");
        }
        
    }

    virtual void th_loop ( void * ) {
        
       CgosIORead( hCgos, 0, &gpioInputValues );
       std::cout << std::showbase 
                 << std::internal
                 << std::setfill('0')
                 << std::hex
                 << gpioInputValues
                 << std::endl;
        
    }
};





////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main ( int argc, char * const argv[] ) try {

    std::map<std::string, Thread_hook*> threads;
    
    sigset_t set;
    int sig;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
    
    main_common (&argc, &argv, 0 );

    threads["GPIO_poller"] = new GPIO_poller(std::string("GPIO_poller"));
    threads["GPIO_poller"]->create(false);

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
