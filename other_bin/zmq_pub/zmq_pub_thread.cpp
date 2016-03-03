#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <exception>

#include <iit/advr/zmq_publisher.h>

extern void main_common ( __sighandler_t sig_handler );

static int main_loop = 1;

static void shutdown ( int sig __attribute__ ( ( unused ) ) ) {
    main_loop = 0;
    printf ( "got signal .... Shutdown\n" );
}

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main ( int argc, char *argv[] ) try {

    std::map<std::string, Thread_hook*> threads;

    main_common ( shutdown );

    threads["ZMQ_pub"] = new iit::ZMQ_Pub_thread();
    threads["ZMQ_pub"]->create ( false,3 );

    while ( main_loop ) {
        sleep ( 1 );
    }

    for ( auto it = threads.begin(); it != threads.end(); it++ ) {
        it->second->stop();
        it->second->join();
        delete it->second;
    }

    std::cout << "Exit main" << std::endl;

    return 0;

} catch ( std::exception& e ) {

    std::cout << "Main catch .... " <<  e.what() << std::endl;

}


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
