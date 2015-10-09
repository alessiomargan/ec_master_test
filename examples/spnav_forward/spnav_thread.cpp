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

#include <spnav.h>

#define DEFAULT_PIPE_NAME "EC_board_nav_input"

#ifdef __XENO__
    static const std::string pipe_prefix("/proc/xenomai/registry/rtipc/xddp/");
#else
    static const std::string pipe_prefix("/tmp/");
#endif
 
static int main_loop = 1;
static int xddp_sock = 0;

static void shutdown(int sig __attribute__((unused)))
{
    main_loop = 0;
    printf("got signal .... Shutdown\n");
    spnav_close();
    close(xddp_sock);
    exit(0);
}

static void set_signal_handler(void)
{
    signal(SIGINT, shutdown);
    signal(SIGTERM, shutdown);
    signal(SIGKILL, shutdown);
}

static void print_spnav_ev(spnav_event &sev) {

    if(sev.type == SPNAV_EVENT_MOTION) {
	    printf("got motion event: t(%d, %d, %d) ", sev.motion.x, sev.motion.y, sev.motion.z);
	    printf("r(%d, %d, %d)\n", sev.motion.rx, sev.motion.ry, sev.motion.rz);
    } else {	/* SPNAV_EVENT_BUTTON */
	    printf("got button %s event b(%d)\n", sev.button.press ? "press" : "release", sev.button.bnum);
    }
}    

    
void * spnav_nrt_thread(void * arg)
{
    int nbytes, fd_in;
    char c;
    std::string line;
    spnav_event sev;
    
    std::string pipe_name((char*)arg);
    std::string pipe(pipe_prefix + pipe_name);
    xddp_sock = open(pipe.c_str(), O_WRONLY);

    if (xddp_sock < 0 ) {
        printf ("%s : %s\n", pipe.c_str(), strerror(errno));
    } else {
	std::cout << "Using "<< pipe << std::endl;
    }

    if(spnav_open()==-1) {
	std::cout << "FAIL open spnav " << std::endl;
	return 0;
    }
    
    /* spnav_wait_event() and spnav_poll_event(), will silently ignore any non-spnav X11 events.
	*
	* If you need to handle other X11 events you will have to use a regular XNextEvent() loop,
	* and pass any ClientMessage events to spnav_x11_event, which will return the event type or
	* zero if it's not an spnav event (see spnav.h).
	*/
    while(spnav_wait_event(&sev)) {
	print_spnav_ev(sev);
	if ( xddp_sock > 0 ) {
	    nbytes = write(xddp_sock, (void*)&sev, sizeof(sev));
	}
    }

    spnav_close();
    close(xddp_sock);
    
    return 0;
}


int main(int argc, char *argv[]) try {

    struct sched_param param;
    int policy = SCHED_OTHER;
    char * pipe_arg = 0;
    
    if ( argc < 2 ) {
	pipe_arg = (char*)DEFAULT_PIPE_NAME;
    } else if ( argc == 2 ) {
	pipe_arg = argv[1];    
    }

    printf("Using pipe name %s\n", pipe_arg);
    
    set_signal_handler();

    param.sched_priority = sched_get_priority_max(policy);
    if(sched_setscheduler(0, policy, &param) == -1) {
            perror("sched_setscheduler failed");
            exit(-1);
    }

    spnav_nrt_thread(pipe_arg);
    
    return 0;
    
} catch (std::exception& e) {

    std::cout << "Main catch .... " <<  e.what() << std::endl;

}