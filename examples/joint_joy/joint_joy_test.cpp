#include <cerrno>
#include <cassert>
#include <csignal>
#include <exception>

#include <ec_boards_joint_joy.h>
#include <iit/advr/zmq_publisher.h>

std::map<std::string, Thread_hook*> threads;

extern void main_common ( int *argcp, char *const **argvp, __sighandler_t sig_handler );

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main ( int argc, char * const argv[] ) try {

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

    pthread_barrier_init(&threads_barrier, NULL, 2);

    threads["boards_basic"] = new EC_boards_joint_joy ( argv[1] );
    threads["boards_basic"]->create ( true );

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
