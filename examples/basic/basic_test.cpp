#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>

#include <exception>
#include <iostream>

#include <ec_boards_basic.h>

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

    main_common (&argc, &argv, shutdown );

    threads["boards_basic"] = new Ec_Boards_basic ( argv[1] );
    threads["boards_basic"]->create ( true );

    while ( main_loop ) {
        sleep ( 1 );
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
