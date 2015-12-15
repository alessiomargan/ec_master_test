//#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <exception>
#include <iostream>

#include <iit/ecat/ec_master_iface.h>

extern void main_common(__sighandler_t sig_handler);

static int main_loop = 1;

void shutdown(int sig __attribute__((unused)))
{
    main_loop = 0;
    printf("got signal .... Shutdown\n");
}

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main(int argc, char *argv[]) try {

    uint64_t sync_cycle_time_ns = 1e6;
    uint64_t sync_cycle_offset_ns = 0;
    
    if ( argc != 2) {
	printf("Usage: %s eth_name\n", argv[0]);
        return 0;
    }

    main_common(shutdown);
    
    iit::ecat::initialize(argv[1]);
    
    iit::ecat::operational(&sync_cycle_time_ns, &sync_cycle_offset_ns);
    
    while (main_loop ) { sleep(3); }

    iit::ecat::finalize();

    std::cout << "Exit main" << std::endl;

    return 0;

} catch (std::exception& e) {

    std::cout << "Main catch .... " <<  e.what() << std::endl;

}


