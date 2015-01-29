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

#define DEG2RAD(X)  ((float)X*M_PI)/180.0


extern Rid2PosMap  rid2pos;


static void load_trj(std::string filename, std::vector<std::vector<float>> &trj) {

    std::string     line;
    std::fstream    file(filename, std::ios::in);

    typedef boost::tokenizer<boost::char_separator<char>> Tokenizer;
    boost::char_separator<char> sep(",");

    if (file) {
        while (getline(file, line)) {
            Tokenizer info(line, sep); // tokenize the line of data
            std::vector<float> values;
            for (Tokenizer::iterator it = info.begin(); it != info.end(); ++it) {
                // convert data into double value, and store
                values.push_back(atof(it->c_str()));
            }
            // store array of values
            trj.push_back(values);
        }
    } else {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
    }
}


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

    ///////////////////////////////////////////////////////////////////////

    Ec_Boards_ctrl * ec_boards_ctrl;

    ec_boards_ctrl = new Ec_Boards_ctrl(argv[1]); 

    if ( ec_boards_ctrl->init() <= 0 ) {
        std::cout << "Error in boards init()... cannot proceed!" << std::endl;      
        delete ec_boards_ctrl;
        return 0;
    }
    ec_boards_ctrl->configure_boards();

    if ( ec_boards_ctrl->set_operative() <= 0 ) {
        std::cout << "Error in boards set_operative()... cannot proceed!" << std::endl; 
        delete ec_boards_ctrl;
        return 0;
    }


    int cnt = 0;
    McEscPdoTypes::pdo_rx mc_pdo_rx;
    McEscPdoTypes::pdo_tx mc_pdo_tx;

    Ft6EscPdoTypes::pdo_rx ft_pdo_rx;
    Ft6EscPdoTypes::pdo_tx ft_pdo_tx;

    uint16_t  cmd = CTRL_POWER_MOD_OFF;

    uint64_t    start = get_time_ns();
    uint64_t    dt;
    double time = 0;
    int sPos = 0;
#if 0
    std::vector<int> rIDs = {
        41, 42, 43, 44, 45, 46,
        51, 52, 53, 54, 55, 56
    };
#else
    std::vector<int> rIDs = {
        42, 43, 44, 45, 46,
        52, 53, 54, 55, 56
    };
#endif
    
    std::map<int,int> rid2col;
    rid2col[41] = 0;
    rid2col[42] = 1;
    rid2col[43] = 2;
    rid2col[44] = 3;
    rid2col[45] = 4;
    rid2col[46] = 5;
    rid2col[51] = 6;
    rid2col[52] = 7;
    rid2col[53] = 8;
    rid2col[54] = 9;
    rid2col[55] = 10;
    rid2col[56] = 11;

    std::map<int,int> syn;
    syn[41] = 1;
    syn[42] = 1;
    syn[43] = 1;
    syn[44] = 1;
    syn[45] = -1;
    syn[46] = -1;
    syn[51] = 1;
    syn[52] = 1;
    syn[53] = -1;
    syn[54] = -1;
    syn[55] = 1;
    syn[56] = 1;
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
    off[55] = DEG2RAD(-20);;
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

    std::vector<std::vector<float>> trj;
    load_trj("walk_5cm_5steps.csv", trj);

#if 0
    std::vector<std::vector<float>>::const_iterator it = trj.begin();
    while ( it != trj.end()) {
        std::vector<float> row(*it);
        std::vector<float>::const_iterator rit = row.begin();
        while ( rit != row.end()) {
            std::cout << (*rit) << "\t";
            rit ++;
        }
        std::cout << std::endl;
        it ++;
    }
#endif
    
    std::vector<std::vector<float>>::const_iterator it = trj.begin();
    std::vector<float> row(*it);
    std::vector<float>::const_iterator rit = row.begin();
    while ( rit != row.end()) {
        std::cout << (*rit) << "\t";
        rit ++;
    }
    std::cout << std::endl;
    it ++;
    row = *it;
    rit = row.begin();
    home[41] = M_PI - (syn[41]*off[41]) + (syn[41]*(*rit)); rit++;
    home[42] = M_PI - (syn[42]*off[42]) + (syn[42]*(*rit)); rit++;
    home[43] = M_PI - (syn[43]*off[43]) + (syn[43]*(*rit)); rit++;
    home[44] = M_PI - (syn[44]*off[44]) + (syn[44]*(*rit)); rit++;
    home[45] = M_PI - (syn[45]*off[45]) + (syn[45]*(*rit)); rit++;
    home[46] = M_PI - (syn[46]*off[46]) + (syn[46]*(*rit)); rit++;
    home[51] = M_PI - (syn[51]*off[51]) + (syn[51]*(*rit)); rit++;
    home[52] = M_PI - (syn[52]*off[52]) + (syn[52]*(*rit)); rit++;
    home[53] = M_PI - (syn[53]*off[53]) + (syn[53]*(*rit)); rit++;
    home[54] = M_PI - (syn[54]*off[54]) + (syn[54]*(*rit)); rit++;
    home[55] = M_PI - (syn[55]*off[55]) + (syn[55]*(*rit)); rit++;
    home[56] = M_PI - (syn[56]*off[56]) + (syn[56]*(*rit)); rit++;

    for ( auto it = rIDs.begin(); it != rIDs.end(); it++ ) {
        std::cout << *it << " " << home[*it] << std::endl; 
    }

    // set all motor to stay where they are ....
    for ( auto it = rIDs.begin(); it != rIDs.end(); it++ ) {

        sPos = rid2pos[*it];

        ec_boards_ctrl->set_ctrl_status(sPos, CTRL_SET_DIRECT_MODE);
        ec_boards_ctrl->set_ctrl_status(sPos, CTRL_POWER_MOD_ON);

        ec_boards_ctrl->recv_from_slaves();

        ec_boards_ctrl->getRxPDO(sPos, mc_pdo_rx);
        mc_pdo_tx.pos_ref = mc_pdo_rx.position;
        if ( *it == 42 || *it == 46 || *it == 52 || *it == 56 ) {
            // medium
            mc_pdo_tx.PosGainP = 30;
            mc_pdo_tx.PosGainI = 0.0;
            mc_pdo_tx.PosGainD = 0.2;
        } else {

            // big
            mc_pdo_tx.PosGainP = 200;
            mc_pdo_tx.PosGainI = 0.0;
            mc_pdo_tx.PosGainD = 3;
        }
        ec_boards_ctrl->setTxPDO(sPos, mc_pdo_tx);

        ec_boards_ctrl->send_to_slaves();

        ec_boards_ctrl->set_ctrl_status(sPos, CTRL_SET_POS_MODE);

        // go to
        while ( run_loop ) {

            // rx from all
            ec_boards_ctrl->recv_from_slaves();
            // rID rx pdo
            ec_boards_ctrl->getRxPDO(sPos, mc_pdo_rx);
            if ( fabs(mc_pdo_rx.position - home[*it]) > 0.1 ) {
                if ( mc_pdo_rx.position > home[*it] ) {
                    mc_pdo_tx.pos_ref -= 0.0005;
                } else {
                    mc_pdo_tx.pos_ref += 0.0005;
                }
                DPRINTF("%d GO to %f --> %f\n", *it, home[*it], mc_pdo_tx.pos_ref);
                ec_boards_ctrl->setTxPDO(sPos, mc_pdo_tx);

                ec_boards_ctrl->send_to_slaves();

                osal_usleep(1000);

            } else {
                break;
            }
        }

    }


    std::vector<std::vector<float>>::const_iterator trj_it = trj.begin();
    trj_it++;
    trj_it++;

 
    while ( run_loop ) {

        // TO REMOVE
        //osal_usleep(2000);

        ec_boards_ctrl->recv_from_slaves();

        if ( trj_it != trj.end() ) {

            row = *trj_it;
            rit = row.begin();

            for ( auto it = rIDs.begin(); it != rIDs.end(); it++ ) {

                sPos = rid2pos[*it];
                ec_boards_ctrl->getRxPDO(sPos, mc_pdo_rx);
                mc_pdo_tx.pos_ref = M_PI - (syn[*it]*off[*it]) + (syn[*it]*(row[rid2col[*it]]));
                DPRINTF("GO %f\n", mc_pdo_tx.pos_ref);
                ec_boards_ctrl->setTxPDO(sPos, mc_pdo_tx);

            }
            trj_it++;
            DPRINTF("\n\n");

        } else {
            //trj_it = trj.begin();
            //trj_it++;
            break;
        }

        ///////////////////////////////////////////////////////////////////////
        //dt = get_time_ns() - start;
        //time += 0.0001;
        //mc_pdo_tx.pos_ref = home[*it] + 0.2 * sinf(2*M_PI*time);
        //DPRINTF("GO %f\n", mc_pdo_tx.pos_ref);
        //ec_boards_ctrl->setTxPDO(sPos, mc_pdo_tx);
        ///////////////////////////////////////////////////////////////////////
        
        ec_boards_ctrl->send_to_slaves();

        if ( (cnt % 10) == 0 ) {
            //ec_boards_ctrl->check_sanity(sPos);
        }
        cnt++;
    }

    for ( auto it = rIDs.begin(); it != rIDs.end(); it++ ) {
        ec_boards_ctrl->set_ctrl_status(rid2pos[*it],CTRL_POWER_MOD_OFF);
    }

    /////////////////////////////////////////////////////////////////

    delete ec_boards_ctrl;


    return 0;
}
