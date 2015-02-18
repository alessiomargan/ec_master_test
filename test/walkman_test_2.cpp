#include <signal.h>
#include <sys/mman.h>
#include <execinfo.h>
#include <stdio.h>
#include <stdint.h>

#ifdef __XENO__
    #include <rtdk.h>
#endif

#include <boost/circular_buffer.hpp>
#include <boost/tokenizer.hpp>

#include <iit/ecat/advr/ec_boards_iface.h>
#include <ati_iface.h>

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
        printf("Usage: %s ifname trajectory file\nifname = {eth0,rteth0} for example\n", argv[0]);
        return 0;
    }

    ///////////////////////////////////////////////////////////////////////

    Ec_Boards_ctrl * ec_boards_ctrl;

    ec_boards_ctrl = new Ec_Boards_ctrl(std::string(argv[1])); 

    if ( ec_boards_ctrl->init() != EC_BOARD_OK ) {
        std::cout << "Error in boards init()... cannot proceed!" << std::endl;      
        delete ec_boards_ctrl;
        return 0;
    }
    ec_boards_ctrl->configure_boards();

    Rid2PosMap  rid2pos = ec_boards_ctrl->get_Rid2PosMap();

    uint64_t    start = get_time_ns();
    uint64_t    dt;
    double time = 0;
    int sPos = 0;
#if 1
    std::vector<int> rIDs = {
        41, 51, // hip  
        42, 52,
        43, 53,
        44, 54,
        45, 55,
        46, 56,
    };
#else
    //std::vector<int> rIDs = {
    //    42, 43, 44, 45, 46,
    //    52, 53, 54, 55, 56
    //};
    std::vector<int> rIDs = {
        //42, 52,
        //43, 53,
        //44, 54,
        //45, 55,
        46, 56,
    };
#endif
    
    std::map<int,int> sgn;
    sgn[41] = 1;
    sgn[42] = 1;
    sgn[43] = 1;
    sgn[44] = 1;
    sgn[45] = -1;
    sgn[46] = -1;
    sgn[51] = 1;
    sgn[52] = 1;
    sgn[53] = -1;
    sgn[54] = -1;
    sgn[55] = 1;
    sgn[56] = -1; //1;
    std::map<int,float> off;
    off[41] = 0;
    off[42] = 0;
    off[43] = 0;
    off[44] = 0;
    off[45] = DEG2RAD(-20);
    off[46] = 0;
    off[51] = 0;
    off[52] = 0;
    off[53] = 0;
    off[54] = 0;
    off[55] = DEG2RAD(-20);
    off[56] = 0;

#define MID_POS(m,M)    (m+(M-m)/2)
    std::map<int,float> home;
    home[41] = MID_POS(2.28,3.75);
    home[42] = MID_POS(1.60,4.02);
    home[43] = MID_POS(1.06,4.02);
    home[44] = 4.31; //MID_POS(,);
    home[45] = MID_POS(2.07,4.26);
    home[46] = M_PI; //MID_POS(,);

    home[51] = MID_POS(2.63,3.96);
    home[52] = MID_POS(2.27,4.73);
    home[53] = MID_POS(2.09,5.23);
    home[54] = MID_POS(0.80,3.15);
    home[55] = MID_POS(2.07,4.19);
    home[56] = MID_POS(2.36,3.90);

    for ( auto it = rIDs.begin(); it != rIDs.end(); it++ ) {
        std::cout << *it << " " << home[*it] << std::endl; 
    }


    float P,I,D;

    // set all motor to stay where they are ....
    for ( auto it = rIDs.begin(); it != rIDs.end(); it++ ) {

        sPos = rid2pos[*it];
        
        Motor * moto = ec_boards_ctrl->slave_as_Motor(sPos);
        assert(moto);
        moto->set_off_sgn(off[*it],sgn[*it]);
      
        if ( *it == 42 || *it == 46 || *it == 52 || *it == 56 ) {
            // medium
            P = 60.0;
            I = 0.0;
            D = 1.0;
        } else if ( *it == 41 || *it == 51 ) {
            // big hip yaw
            P = 800.0;
            I = 0.0;
            D = 12.0;
        } else if ( *it == 43 || *it == 53 ) {
            // big hip pitch
            P = 700.0;
            I = 0.0;
            D = 10.0;
        } else if ( *it == 44 || *it == 54 ) {
            // big knee
            P = 500.0;
            I = 0.0;
            D = 2.0;
        } else {
            // big
            P = 500.0;
            I = 0.0;
            D = 2.0;
        }
        
        moto->start_log(true);
        moto->readSDO("position", home[*it]);
        moto->start(CTRL_SET_POS_MODE, P, I, D);

    } // end for

#if 0
    float pos, pos_ref;

    while ( run_loop ) {

        osal_usleep(1000);

        time += 0.0002;

        for ( auto it = rIDs.begin(); it != rIDs.end(); it++ ) {
            sPos = rid2pos[*it];
            Motor * moto = ec_boards_ctrl->slave_as_Motor(sPos);
            assert(moto);

            pos_ref = home[*it] + 0.2 * sinf(2*M_PI*time);
            moto->set_SDO("pos_ref", pos_ref);
            moto->get_SDO("pos_ref", pos_ref);
            moto->get_SDO("position", pos);
            //DPRINTF("%d home %f pos %f pos_ref %f\n", *it, home[*it] ,pos, pos_ref);
        }
    }
#else

    if ( ec_boards_ctrl->set_operative() <= 0 ) {
        std::cout << "Error in boards set_operative()... cannot proceed!" << std::endl; 
        delete ec_boards_ctrl;
        return 0;
    }

    while (run_loop) {

        if ( ! ec_boards_ctrl->recv_from_slaves() ) {
            break;
        }
        ec_boards_ctrl->send_to_slaves();
    }


#endif


    for ( auto it = rIDs.begin(); it != rIDs.end(); it++ ) {
        sPos = rid2pos[*it];
        Motor * moto = ec_boards_ctrl->slave_as_Motor(sPos);
        assert(moto);
        moto->stop();
    }

    /////////////////////////////////////////////////////////////////

    delete ec_boards_ctrl;


    return 0;
}
