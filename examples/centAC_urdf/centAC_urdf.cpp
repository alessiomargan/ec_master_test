#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <exception>

#include <ec_boards_centAC_urdf.h>
//#include <iit/advr/zmq_publisher.h>

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

    main_common ( &argc, &argv, shutdown );

    threads["CentAC_urdf"] = new EC_boards_centAC_urdf ( argv[1] );
    threads["CentAC_urdf"]->create ( true, 2 );

#if 0
    // ZMQ_pub wait for pipe creation
    while ( ! dynamic_cast<Ec_Thread_Boards_base*> ( threads["EC_boards_joint_joy"] )->init_OK() ) {
        sleep ( 1 );
    }
    threads["ZMQ_pub"] = new iit::ZMQ_Pub_thread();
    threads["ZMQ_pub"]->create ( false,3 );
#endif

    while ( main_loop ) {
        sleep ( 1 );
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
