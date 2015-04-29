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

    Rid2PosMap  rid2pos = ec_boards_ctrl->get_Rid2PosMap();

    const YAML::Node doc = ec_boards_ctrl->get_config_YAML_Node();
    const YAML::Node firmware_update = doc["firmware_update"];

    std::vector<int> slave_list;
    bool use_rId = true;
    
    if ( firmware_update["slave_rId_list"] ) {
        slave_list = firmware_update["slave_rId_list"].as<std::vector<int>>();
    } else if ( firmware_update["slave_pos_list"] ) {
        slave_list = firmware_update["slave_pos_list"].as<std::vector<int>>();
        use_rId = false;
    } else {
        std::cout << "NO slave list found !!" << std::endl;
        delete ec_boards_ctrl;
        return 1;
    }
    
    std::string fw_path;
    std::string bin_file;
    int passwd;
    EscWrapper * esc;
    uint16_t   bType;
    int sPos;

    fw_path = firmware_update["fw_path"].as<std::string>();

    for (auto it = slave_list.begin(); it != slave_list.end(); it++) {
        if ( use_rId ) {
            // use rId
            try { sPos = rid2pos.at(*it); }
            catch (const std::out_of_range& oor) { DPRINTF("%s\n", oor.what()); }
        } else {
            // use slave pos
            sPos = *it;
        }
        esc = ec_boards_ctrl->slave_as_EscWrapper(sPos);
        if ( !esc ) {
            esc = ec_boards_ctrl->slave_as_Zombie(sPos);
            DPRINTF("..... try with Z0mb13 %d\n", sPos);
        }
        if( esc ) {
            YAML::Node motor_type;
            bType = esc->get_ESC_type();
            switch ( bType ) {
                case HI_PWR_AC_MC :
                    motor_type = firmware_update["big_motor"];
                    break;
                case HI_PWR_DC_MC :
                    motor_type = firmware_update["medium_motor"];
                    break;
                case LO_PWR_DC_MC :
                    motor_type = firmware_update["small_motor"];
                    break;
                case FT6 :
                    motor_type = firmware_update["force_torque_6"];
                    break;
                default :
                    break;
            }

            if ( motor_type ) {
                bin_file    = motor_type["bin_file"].as<std::string>();
                passwd      = motor_type["passwd"].as<int>();
                DPRINTF("%d %s 0x%04X \n", *it, (fw_path+bin_file).c_str(), passwd);
                ret = ec_boards_ctrl->update_board_firmware(sPos, fw_path+bin_file, passwd);
                if ( ! ret  ) {
                    DPRINTF("FAIL update slave pos %d\n" ,sPos);
                }
            } else {
                DPRINTF("Unknown esc type %d\n" ,bType);
            }

        }
    }
    
    delete ec_boards_ctrl;

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    ec_boards_ctrl = new Ec_Boards_ctrl(argv[1]); 

    if ( ec_boards_ctrl->init() != EC_BOARD_OK) {
        std::cout << "Error in boards init()... cannot proceed!" << std::endl;		
        delete ec_boards_ctrl;
        return 0;
    }
    
    delete ec_boards_ctrl;

    return 1;
}
