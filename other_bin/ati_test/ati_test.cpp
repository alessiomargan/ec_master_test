//#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <exception>
#include <iostream>
#include <cmath>
#include <random>

#include <iit/ecat/utils.h>
#include <iit/ecat/advr/pipes.h>
#include <iit/advr/thread_util.h>
#include <iit/advr/ati_iface.h>
#include <iit/advr/zmq_pub_th.h>

extern void main_common ( int *argcp, char *const **argvp, __sighandler_t sig_handler );
extern void set_main_sched_policy ( int );



////////////////////////////////////////////////////
// 
////////////////////////////////////////////////////
class ATI_thread : public Thread_hook {

    iit::ecat::stat_t       loop_time;
    uint64_t                tNow, dt;
    int                     xddp_fd;
    std::string             pipe_name;
    
    /////////////////////////////////////////
    
    iit::advr::Ati_Sens * ati;
    
    YAML::Node ati_config;
    
    boost::circular_buffer<iit::advr::ati_log_t> sens_log;
    iit::advr::ati_log_t   sample;
        
public:

    ATI_thread( std::string config_yaml ) {

        name = "ati_thread";
        // periodic
        period.period = {0,1};

        schedpolicy = SCHED_OTHER;
        priority = sched_get_priority_max ( schedpolicy ) /2;
        stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

        std::ifstream fin ( config_yaml );
        if ( fin.fail() ) {
            DPRINTF ( "Can not open %s\n", config_yaml.c_str() );
            assert ( 0 );
        }

        ati_config = YAML::LoadFile ( config_yaml )["ati_config"];
        sens_log.set_capacity ( 1000000 );
    
        ati = new iit::advr::Ati_Sens ( );
        ati->config( true );

    }

    ~ATI_thread() {

        delete ati;

        if ( ati_config["dump_log"].as<bool>() ) {
            std::string filename = "/tmp/sens.txt";
            iit::ecat::dump_buffer ( filename, sens_log );
        }

        std::cout << "~" << typeid ( this ).name() << std::endl;
    }

    virtual void th_init ( void * ) {

    }

    virtual void th_loop ( void * ) {
        
        //ati->get_last_sample ( sample );
        //for ( int i=0; i < 6; i++ ) sample.ft[i] = sample.ft[i] / 1000 ;
        //sens_log.push_back ( sample );
        
        //sample.fprint(stderr);
    }
};


////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main ( int argc, char * const argv[] ) try {

    std::map<std::string, Thread_hook*> threads;
    
    threads["ati_thread"]   = new ATI_thread( argv[1] );
    //threads["ZMQ_pub"]      = new ZMQ_Pub_thread( argv[1] );

    sigset_t set;
    int sig;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
    
    main_common (&argc, &argv, 0 );
    
    threads["ati_thread"]->create(false);

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
