#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <exception>

#include <ec_boards_impedance_test.h>

extern void main_common ( int *argcp, char *const **argvp, __sighandler_t sig_handler );


////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main ( int argc, char * const argv[] ) try {

    std::map<std::string, Thread_hook*> threads;

    if ( argc != 2 ) {
        printf ( "Usage: %s config.yaml\n", argv[0] );
        return 0;
    }

    sigset_t set;
    int sig;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
    
    main_common (&argc, &argv, 0 );

    threads["impedance_test"] = new EC_boards_impedance_test ( argv[1] );
    threads["impedance_test"]->create ( true, 2 );

#if 0
    // ZMQ_pub wait for pipe creation
    while ( ! dynamic_cast<Ec_Thread_Boards_base*> ( threads["EC_boards_joint_joy"] )->init_OK() ) {
        sleep ( 1 );
    }
    threads["ZMQ_pub"] = new iit::ZMQ_Pub_thread();
    threads["ZMQ_pub"]->create ( false,3 );
#endif

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
