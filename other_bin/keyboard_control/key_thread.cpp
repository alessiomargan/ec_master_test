#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <iostream>
#include <sstream>

#include <unistd.h>
#include <termios.h>


#ifdef __XENO_PIPE__
static const std::string pipe_prefix ( "/proc/xenomai/registry/rtipc/xddp/" );
#else
static const std::string pipe_prefix ( "/tmp/" );
#endif

extern void main_common ( __sighandler_t sig_handler );
extern void set_main_sched_policy ( int );

static int main_loop = 1;

static void shutdown ( int sig __attribute__ ( ( unused ) ) ) {
    main_loop = 0;
    printf ( "got signal .... Shutdown\n" );
}

static void set_signal_handler ( void ) {
    signal ( SIGINT, shutdown );
    signal ( SIGTERM, shutdown );
    signal ( SIGKILL, shutdown );
}

void * js_nrt_thread ( void * ) {
    int xddp_sock, nbytes, fd_in;
    char c;
    std::string line;

    std::string pipe_name ( "EC_board_key_input" );
    std::string pipe ( pipe_prefix + pipe_name );
    xddp_sock = open ( pipe.c_str(), O_WRONLY );

    if ( xddp_sock < 0 ) {
        printf ( "%s : %s\n", pipe.c_str(), strerror ( errno ) );
    } else {
        std::cout << "Using "<< pipe << std::endl;
    }

    while ( main_loop ) {

        std::getline ( std::cin, line );
        nbytes = write ( xddp_sock, ( void* ) line.c_str(), line.length() );


        if ( nbytes < 0 ) {
            perror ( "write" );
            break;
        }

    }

    close ( fd_in );
    close ( xddp_sock );

    return 0;
}


int main ( void ) {

    struct sched_param param;
    int policy = SCHED_OTHER;

    main_common ( shutdown );
    set_main_sched_policy ( sched_get_priority_max ( policy ) );

    js_nrt_thread ( 0 );

    return 0;
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
