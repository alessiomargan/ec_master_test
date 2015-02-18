#include <signal.h>
#include <sys/mman.h>
#include <execinfo.h>
#include <stdio.h>
#include <stdint.h>
#include <yaml-cpp/yaml.h>

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


    if ( argc != 2) {
    printf("Usage: %s config.yaml\n", argv[0]);
        return 0;
    }


    Ec_Boards_ctrl * ec_boards_ctrl;

    ///////////////////////////////////////////////////////////////////////////

    ec_boards_ctrl = new Ec_Boards_ctrl(argv[1]); 

    if ( ec_boards_ctrl->init() != EC_BOARD_OK) {
        std::cout << "Error in boards init()... cannot proceed!" << std::endl;		
        delete ec_boards_ctrl;
        return 0;
    }

    ec_boards_ctrl->configure_boards();

    Rid2PosMap  rid2pos = ec_boards_ctrl->get_Rid2PosMap();

#if 1
    Motor * moto = ec_boards_ctrl->slave_as_Motor(1);
    assert(moto);
    moto->set_off_sgn(0,1);
    float home;
    moto->readSDO("position", home);
    DPRINTF(">>>>>>>>>>> home %f\n", home);
    moto->writeSDO<float>("tor_offs", 0.0);


    if ( moto->start(CTRL_SET_POS_LNK_ERR, 13.0, 0.0035, 3.0) != EC_BOARD_OK ) {
    //if ( moto->start(CTRL_SET_MIX_POS_MODE_2) != EC_BOARD_OK ) {
        DPRINTF("Motor not started\n");
        delete ec_boards_ctrl;
        return 0;
    }

    if ( ec_boards_ctrl->set_operative() <= 0 ) {
        std::cout << "Error in boards set_operative()... cannot proceed!" << std::endl; 
        delete ec_boards_ctrl;
        return 0;
    }

    sleep(3);

    moto->start_log(true);

    uint64_t dt;
    double time = 0.0;
    McEscPdoTypes::pdo_rx mc_pdo_rx;
    McEscPdoTypes::pdo_tx mc_pdo_tx;

    mc_pdo_tx.PosGainP = 13.0;
    mc_pdo_tx.PosGainI = 0.0035; 
    mc_pdo_tx.PosGainD = 3.0;
    mc_pdo_tx.tor_offs = 0.0;
    mc_pdo_tx.pos_ref = 0;
    ec_boards_ctrl->setTxPDO(1, mc_pdo_tx);

    ec_boards_ctrl->send_to_slaves();

    while (run_loop) {

        if ( ! ec_boards_ctrl->recv_from_slaves() ) {
            break;
        }

        time += 0.0003;
        ec_boards_ctrl->getRxPDO(1, mc_pdo_rx);
        mc_pdo_tx.pos_ref = home + 1 * sinf(2*M_PI*time);
        //ec_boards_ctrl->setTxPDO(1, mc_pdo_tx);
        moto->set_posRef(home + 1.0 * sinf(2*M_PI*time));

        //DPRINTF("GO %f\n", mc_pdo_tx.pos_ref);

        ec_boards_ctrl->send_to_slaves();
    }

    moto->stop();

#else

    HubIoESC * hub_io = dynamic_cast<HubIoESC*>(ec_boards_ctrl->slave_as_EscWrapper(2));
    uint16_t reg_0x1000;

    while (run_loop) {

        hub_io->read_io_reg(reg_0x1000);
        DPRINTF("%d\n",reg_0x1000);
        sleep(1);

    }

#endif

    delete ec_boards_ctrl;

    return 1;
}
