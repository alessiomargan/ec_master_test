//#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <exception>
#include <iostream>

#include <iit/ecat/ec_master_iface.h>

extern void main_common ( __sighandler_t sig_handler );

static int main_loop = 1;

void shutdown ( int sig __attribute__ ( ( unused ) ) ) {
    main_loop = 0;
    DPRINTF ( "got signal .... Shutdown\n" );
}

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main ( int argc, char *argv[] ) try {

    uint32_t sync_cycle_time_ns = 1e6;
    uint32_t sync_cycle_offset_ns = 0; //1e9;

    iit::ecat::ec_timing_t timing;

    if ( argc != 2 ) {
        DPRINTF ( "Usage: %s eth_name\n", argv[0] );
        return 0;
    }

    main_common ( shutdown );

    iit::ecat::initialize ( argv[1] );

    iit::ecat::operational ( sync_cycle_time_ns, sync_cycle_offset_ns );

    while ( main_loop ) {

        iit::ecat::recv_from_slaves ( &timing );
        DPRINTF ( "loop_time %ld\toffset %ld\trecv_dc_time %ld", timing.loop_time, timing.offset, timing.recv_dc_time );
        DPRINTF ( "\33[2K\r" );
    }

    iit::ecat::finalize();

    std::cout << "Exit main" << std::endl;

    return 0;

} catch ( std::exception& e ) {

    std::cout << "Main catch .... " <<  e.what() << std::endl;

}


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
