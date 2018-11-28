#include <cerrno>
#include <cassert>
#include <csignal>

#include <exception>
#include <iostream>

#include <ec_board_calib.h>
#include <iit/ecat/advr/esc.h>
//#include <iit/advr/zmq_publisher.h>

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

    threads["boards_calib"] = new Ec_Board_Calib ( argv[1] );
    pthread_barrier_init(&threads_barrier, NULL, threads.size());
    threads["boards_calib"]->create ( true );
    pthread_barrier_wait(&threads_barrier);

    sigwait(&set, &sig);  
     
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