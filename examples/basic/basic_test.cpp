#include <cerrno>
#include <cassert>
#include <csignal>

#include <exception>
#include <iostream>

#include <ec_boards_basic.h>
#include <iit/advr/zmq_pub_th.h>
#include <iit/advr/zmq_rep_th.h>

extern void main_common ( int *argcp, char *const **argvp, __sighandler_t sig_handler );

static int basic_loop = 1;

static void basic_sighandler(int signum) {
    
    std::cout << "Handling signal " << signum << std::endl;
    basic_loop = 0;
}

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main ( int argc, char * const argv[] ) try {

    ThreadsMap threads;
    int cpu_num = 2;
    
    if ( argc != 2 ) {
        std::cout << "Usage: " << argv[0] << " config.yaml" << std::endl;
        return 0;
    }

    main_common (&argc, &argv, basic_sighandler );

    ZMQ_Rep_thread zmq_rep( argv[1], &threads);
    
    threads["boards_basic"] = new Ec_Boards_basic ( argv[1] );
    threads["ZMQ_pub"] =      new ZMQ_Pub_thread  ( argv[1] );
    //threads["ZMQ_rep"] =      new ZMQ_Rep_thread  ( argv[1], &threads);
    zmq_rep.th_init(0);
    
    /*
     * The barrier is opened when COUNT waiters arrived.
     * we want to wait just Ec_Boards_base when reach OP
     */ 
    pthread_barrier_init(&threads_barrier, NULL, 1+1 );
    threads["boards_basic"]->create ( true, cpu_num );
    pthread_barrier_wait(&threads_barrier);
    threads["ZMQ_pub"]->create ( false, ++cpu_num );
    //threads["ZMQ_rep"]->create ( false, ++cpu_num );
    
    try {
        while ( basic_loop ) {
            zmq_rep.th_loop(0);
        }
    } catch ( std::exception& e ) {
        // ZMQ specific exception
        std::cout << "catching .... " <<  e.what() << std::endl;
    }

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
