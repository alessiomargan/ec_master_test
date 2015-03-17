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

#define FT_ECAT_POS 3

using namespace iit::ecat::advr;

static int run_loop = 1;


typedef struct {
    uint64_t    ts;
    float       ati[6];
    float       iit[6];
    float       dummy[6];
    void sprint(char *buff, size_t size) {
        snprintf(buff, size, "%lld\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t0\t0\t0\t0\t0\t0\n", ts,
// big sensor
//                 iit[0],iit[1],iit[2],iit[3],iit[4],iit[5],
//                 -ati[0],ati[1],ati[2],-ati[3],ati[4],ati[5]);
// small sensor
                 iit[0],iit[1],iit[2],iit[3],iit[4],iit[5],
                 ati[0],ati[1],ati[2],ati[3],ati[4],ati[5]);
    }
    void fprint(FILE *fp) {
        fprintf(fp, "%lld\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t0\t0\t0\t0\t0\t0\n", ts,
// big sensor
//                iit[0],iit[1],iit[2],iit[3],iit[4],iit[5],
//                -ati[0],ati[1],ati[2],-ati[3],ati[4],ati[5]);
// small sensor
                iit[0],iit[1],iit[2],iit[3],iit[4],iit[5],
                ati[0],ati[1],ati[2],ati[3],ati[4],ati[5]);

    }
} sens_data_t ; 


void load_matrix(std::string filename, std::vector<std::vector<float>> &cal_matrix) {

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
            cal_matrix.push_back(values);
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

    boost::circular_buffer<sens_data_t> sens_log;
    sens_log.set_capacity(LOG_SIZE);

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

    Ati_Sens * ati = new Ati_Sens(true);

    ///////////////////////////////////////////////////////////////////////

    std::vector<std::vector<float>> cal_matrix;
    load_matrix("RightFootCalibMatrix.txt", cal_matrix);
    //load_matrix("LeftFootCalibMatrix.txt", cal_matrix);
    //load_matrix("RightWristCalibMatrix.txt", cal_matrix);
    //load_matrix("LeftWristCalibMatrix.txt", cal_matrix);
    //load_matrix("ones.txt", cal_matrix);

    std::vector<std::vector<float>>::const_iterator it = cal_matrix.begin();
    while ( it != cal_matrix.end()) {
        std::vector<float> row(*it);
        std::vector<float>::const_iterator rit = row.begin();
        while ( rit != row.end()) {
            std::cout << (*rit) << "\t";
            rit ++;
        }
        std::cout << std::endl;
        it ++;
    }
    //return 0;

    ///////////////////////////////////////////////////////////////////////

    Ec_Boards_ctrl * ec_boards_ctrl;

    ec_boards_ctrl = new Ec_Boards_ctrl(argv[1]); 

    if ( ec_boards_ctrl->init() != EC_BOARD_OK) {
        std::cout << "Error in boards init()... cannot proceed!" << std::endl;		
        delete ec_boards_ctrl;
        return 0;
    }

    ec_boards_ctrl->configure_boards();

    Rid2PosMap  rid2pos = ec_boards_ctrl->get_Rid2PosMap();

    int cnt = 0;

    
    ec_boards_ctrl->set_flash_cmd(FT_ECAT_POS, CTRL_REMOVE_TORQUE_OFFS);

    //ec_boards_ctrl->set_cal_matrix(FT_ECAT_POS, cal_matrix);
    //ec_boards_ctrl->set_flash_cmd(FT_ECAT_POS, 0x0012);

    Ft6EscPdoTypes::pdo_rx ft_pdo_rx;
    Ft6EscPdoTypes::pdo_tx ft_pdo_tx;

    ati_log_t   sample;
    sens_data_t sens_data;

    if ( ec_boards_ctrl->set_operative() <= 0 ) {
        std::cout << "Error in boards set_operative()... cannot proceed!" << std::endl; 
        delete ec_boards_ctrl;
        return 0;
    }

    //sleep(3);

    uint64_t    start = get_time_ns();

    while ( run_loop ) {

        ec_boards_ctrl->recv_from_slaves();
        ati->get_last_sample(sample);

        if ( (cnt % 10) == 0 ) {
            //ec_boards_ctrl->check_sanity();
        }
        cnt++;

        ///////////////////////////////////////////////////////////////////////

        ec_boards_ctrl->getRxPDO(FT_ECAT_POS, ft_pdo_rx);
        //DPRINTF("IIT\n");
        //ft_pdo_rx.fprint(stderr); 
        //DPRINTF("ATI\n");
        //sample.fprint(stderr);
        //DPRINTF("\n");

        sens_data.ts = get_time_ns() - start;
        memcpy((void*)&sens_data.ati, &sample.ft, sizeof(float)*6);
        memcpy((void*)&sens_data.iit, &ft_pdo_rx.force_X, sizeof(float)*6);
        sens_log.push_back(sens_data);
        sens_data.fprint(stderr);

        ///////////////////////////////////////////////////////////////////////

        ec_boards_ctrl->send_to_slaves();

    }

    delete ec_boards_ctrl;
    delete ati;

    /////////////////////////////////////////////////////////////////

    std::string filename = "/tmp/sens.txt";
    dump_buffer(filename, sens_log);

    return 0;
}
