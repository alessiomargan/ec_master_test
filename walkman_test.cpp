#include <signal.h>
#include <sys/mman.h>
#include <execinfo.h>
#include <stdio.h>
#include <stdint.h>

#ifdef __XENO__
    #include <rtdk.h>
#endif

#include <memory>

#include <iit/ecat/advr/ec_boards_iface.h>

using namespace iit::ecat::advr;

static int run_loop = 1;

static void warn_upon_switch(int sig __attribute__((unused)))
{
    // handle rt to nrt contex switch
    void *bt[3];
    int nentries;

    /* Dump a backtrace of the frame which caused the switch to
       secondary mode: */
    nentries = backtrace(bt,sizeof(bt)/sizeof(bt[0]));
    // dump backtrace 
    backtrace_symbols_fd(bt,nentries,fileno(stdout));
}

static void shutdown(int sig __attribute__((unused)))
{
    run_loop = 0;
    DPRINTF("got signal .... Shutdown\n");
}

static void set_signal_handler(void)
{
    signal(SIGINT, shutdown);
    signal(SIGINT, shutdown);
    signal(SIGKILL, shutdown);
#ifdef __XENO__
    // call pthread_set_mode_np(0, PTHREAD_WARNSW) to cause a SIGXCPU
    // signal to be sent when the calling thread involontary switches to secondary mode
    signal(SIGXCPU, warn_upon_switch);
#endif
}

///////////////////////////////////////////////////////////////////////////////

using namespace iit::ecat;

Ec_Boards_ctrl * ec_boards_ctrl; 


int main(int argc, char **argv)
{
    int ret;

    set_signal_handler();

#ifdef __XENO__
    
    int policy = SCHED_FIFO;
    struct sched_param  schedparam;
    schedparam.sched_priority = sched_get_priority_max(policy);
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &schedparam);

    /* Prevent any memory-swapping for this program */
    ret = mlockall(MCL_CURRENT | MCL_FUTURE);
    if ( ret < 0 ) {
        printf("mlockall failed (ret=%d) %s\n", ret, strerror(ret));
        return 0;
    }
    /*
     * This is a real-time compatible printf() package from
     * Xenomai's RT Development Kit (RTDK), that does NOT cause
     * any transition to secondary (i.e. non real-time) mode when
     * writing output.
     */
    rt_print_auto_init(1);
#endif

    if ( argc != 2 ) {
    printf("Usage: %s ifname\nifname = {eth0,rteth0} for example\n", argv[0]);
        return 0;
    }

    Ec_Boards_ctrl * ec_boards_ctrl = new Ec_Boards_ctrl(argv[1]); 

    if ( ec_boards_ctrl->init() <= 0) {
        delete ec_boards_ctrl;
        return 0;
    }

    ec_boards_ctrl->configure_boards();

    if ( ec_boards_ctrl->set_operative() <= 0) {
        delete ec_boards_ctrl;
        return 0;
    }
    
    int cnt = 0;
    uint16_t  cmd = CTRL_POWER_MOD_OFF;
    while ( run_loop ) {

        ec_boards_ctrl->recv_from_slaves();
        
        if ( (cnt % 1000) == 0) {
            if (cmd == CTRL_POWER_MOD_OFF) {
                cmd = CTRL_POWER_MOD_ON;
            } else {
                cmd = CTRL_POWER_MOD_OFF;
            }
            ec_boards_ctrl->set_ctrl_status(3,cmd);
        }
        cnt++;

        ec_boards_ctrl->send_to_slaves();
   
    }

    delete ec_boards_ctrl;

    return 0;
}
