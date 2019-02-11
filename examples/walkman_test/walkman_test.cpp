#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <exception>

#include <ec_boards_walkman_test.h>
#include <iit/advr/zmq_pub_th.h>

extern void main_common ( int *argcp, char *const **argvp, __sighandler_t sig_handler );

static int main_loop = 1;

void shutdown ( int sig __attribute__ ( ( unused ) ) ) {
    main_loop = 0;
    printf ( "got signal .... Shutdown\n" );
}

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main ( int argc, char * const argv[] ) try {

    std::map<std::string, Thread_hook*> threads;

    if ( argc != 2 ) {
        printf ( "Usage: %s config.yaml\n", argv[0] );
        return 0;
    }

    sigset_t set, zmq_start_set;
    int sig;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
        
    main_common (&argc, &argv, 0 );

    pthread_barrier_init(&threads_barrier, NULL, 2);

    threads["Walkman_test"] = new EC_boards_walkman_test ( argv[1] );
    threads["Walkman_test"]->create ( true, 2 );

    pthread_barrier_wait(&threads_barrier);
    threads["ZMQ_pub"] = new ZMQ_Pub_thread( argv[1] );
    threads["ZMQ_pub"]->create ( false,3 );
    
#ifdef __COBALT__
    __real_sigwait(&set, &sig);
#else
    sigwait(&set, &sig);  
#endif
     
    for ( auto const& item : threads ) {
        item.second->stop();
        item.second->join();
        delete item.second;
    }

    std::cout << "Exit main" << std::endl;

    return 0;

} catch ( std::exception& e ) {

    std::cout << "Main catch .... " <<  e.what() << std::endl;

}


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
