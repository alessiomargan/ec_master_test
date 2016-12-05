#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <exception>

#include <ec_boards_centAC_impedance.h>

extern void main_common ( __sighandler_t sig_handler );

static int main_loop = 1;

void shutdown ( int sig __attribute__ ( ( unused ) ) ) {
    main_loop = 0;
    printf ( "got signal .... Shutdown\n" );
}

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main ( int argc, char *argv[] ) try {

    std::map<std::string, Thread_hook*> threads;

    if ( argc != 2 ) {
        printf ( "Usage: %s config.yaml\n", argv[0] );
        return 0;
    }

    main_common ( shutdown );

    threads["CentAC_test"] = new EC_boards_centAC_impedance ( argv[1] );
    threads["CentAC_test"]->create ( true, 2 );

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
