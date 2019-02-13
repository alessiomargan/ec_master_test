#include <cerrno>
#include <cassert>
#include <csignal>

#include <exception>
#include <iostream>

#include <ec_boards_basic.h>
#include <iit/advr/zmq_pub_th.h>
#include <iit/advr/zmq_rep_th.h>

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

    threads["boards_basic"] = new Ec_Boards_basic ( argv[1] );
    threads["ZMQ_pub"] =      new ZMQ_Pub_thread( argv[1] );
    threads["ZMQ_rep"] =      new ZMQ_Rep_thread( argv[1], threads["boards_basic"]);
    /*
     * The barrier is opened when COUNT waiters arrived.
     * we want to wait just Ec_Boards_base when reach OP
     */ 
    pthread_barrier_init(&threads_barrier, NULL, 1+1 );
    
    threads["boards_basic"]->create ( true );
    pthread_barrier_wait(&threads_barrier);
    threads["ZMQ_rep"]->create ( false, 3 );
    threads["ZMQ_pub"]->create ( false, 3 );
    
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
