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


#include <linux/joystick.h>
#define JOY_DEV "/dev/input/js0"

#ifdef __XENO_PIPE__
    static const std::string pipe_prefix("/proc/xenomai/registry/rtipc/xddp/");
#else
    static const std::string pipe_prefix("/tmp/");
#endif
 
extern void main_common(__sighandler_t sig_handler);
extern void set_main_sched_policy(int);

static int main_loop = 1;

static void shutdown(int sig __attribute__((unused)))
{
    main_loop = 0;
    printf("got signal .... Shutdown\n");
}
    
static void print_js_ev(struct js_event &js_ev) {

    std::cout << "ev : type "<<
    std::hex << (int)js_ev.type <<
    " cmd  " <<
    std::dec << (int)js_ev.number <<
    " value " <<
    std::dec << (int)js_ev.value <<
    std::endl;
    
}
    
void * js_nrt_thread(void *)
{
    int xddp_sock, nbytes, fd_in;
    char c;
    std::string line;
    struct js_event js_ev;

     std::string pipe_name("EC_board_js_input");
     std::string pipe(pipe_prefix + pipe_name);
     xddp_sock = open(pipe.c_str(), O_WRONLY);

    if (xddp_sock < 0 ) {
        printf ("%s : %s\n", pipe.c_str(), strerror(errno));
    } else {
	std::cout << "Using "<< pipe << std::endl;
    }

    std::cout << "try open input device "<< JOY_DEV << std::endl;
#ifdef NOBLOCK
    if ( (fd_in = open(JOY_DEV, O_RDONLY | O_NONBLOCK)) < 0 ) {
#else
    if ( (fd_in = open(JOY_DEV, O_RDONLY)) < 0 ) {
#endif
	std::cout << "FAIL open  "<< JOY_DEV << std::endl;
	return 0;
    }
    
#ifndef NOBLOCK
    fd_set rfds;
    struct timeval tv;
    int retval;
#endif
    
    while ( main_loop ) {

        //std::getline(std::cin, line);
	//nbytes = write(xddp_sock, (void*)line.c_str(), line.length());
        
	//js_ev.value += 1;
        //nbytes = write(xddp_sock, (void*)&js_ev, sizeof(js_ev));

#ifdef NOBLOCK
	while ( read(fd_in, (void*)&js_ev, sizeof(js_ev)) > 0) {
	    if ( ! (js_ev.type & JS_EVENT_INIT) ) {
		print_js_ev(js_ev);
		if ( xddp_sock > 0 ) {
		    nbytes = write(xddp_sock, (void*)&js_ev, sizeof(js_ev));
		}
	    }
	}
        usleep(1e3);
#else
	FD_ZERO(&rfds);
	FD_SET(fd_in, &rfds);
	tv.tv_sec = 3;
	tv.tv_usec = 0;
 	if ( select(fd_in+1, &rfds, NULL, NULL, &tv) > 0 ) {
	    nbytes = read(fd_in, (void*)&js_ev, sizeof(js_ev));
	    if ( ! (js_ev.type & JS_EVENT_INIT) ) {
		print_js_ev(js_ev);
		if ( xddp_sock > 0 ) {
		    nbytes = write(xddp_sock, (void*)&js_ev, sizeof(js_ev));
		}
	    }
	} else {
	    //std::cout << "select timeout "<< JOY_DEV << std::endl;
	}
#endif
	
        if (nbytes < 0) {
            perror("write");
            break;
        }
        
    }

    close(fd_in);
    close(xddp_sock);
    
    return 0;
}


int main(void) {

    int policy = SCHED_OTHER;
    
    main_common(shutdown);
    set_main_sched_policy(sched_get_priority_max(policy));
    
    js_nrt_thread(0);
    
    return 0;
}
