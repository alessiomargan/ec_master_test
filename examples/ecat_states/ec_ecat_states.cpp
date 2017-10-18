#include <ec_ecat_states.h>
#include <iit/ecat/advr/lxm32i_esc.h>

using namespace iit::ecat::advr;

Ec_Ecat_states::Ec_Ecat_states ( const char* config_yaml ) : Ec_Thread_Boards_base ( config_yaml ) {

    name = "EC_ecat_states";
    // non periodic
    period.period = {0,1};

#ifdef __COBALT__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max ( schedpolicy ) - 10 ;
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    // open pipe ... xeno xddp or fifo
    inXddp.init ( "EC_board_input" );

}

Ec_Ecat_states::~Ec_Ecat_states() {
    std::cout << "~" << typeid ( this ).name() << std::endl;
}

void Ec_Ecat_states::init_preOP ( void ) {

#if 0
    DPRINTF("+++++++++++++++++++++\n");
    Ec_Boards_ctrl::shutdown(false);
    // wait boards boot up
    sleep(1);
    // init Ec_Boards_ctrl
    if ( Ec_Boards_ctrl::init() != iit::ecat::advr::EC_BOARD_OK ) {
        throw "something wrong";
    }
#endif

}

void Ec_Ecat_states::init_OP ( void ) {

    LXM32iESC * moto = slave_as<LXM32iESC>(1); 
    assert ( moto );
    //moto->test_motor();
    
    //throw std::runtime_error("... esco !!");

    
}

template<class C>
int Ec_Ecat_states::user_input ( C &user_cmd ) {

    static int  bytes_cnt;
    int         bytes;

    if ( ( bytes = inXddp.xddp_read ( user_cmd ) ) <= 0 ) {
        return bytes;
    }

    bytes_cnt += bytes;
    //DPRINTF ( ">> %d %d\n",bytes, bytes_cnt );

    return bytes;
}

int Ec_Ecat_states::user_loop ( void ) {

    static auto cazzo = 0;
    static auto sp_it = slave_as<LXM32iESC>(1)->set_points.begin();
    
    static LXM32iEscPdoTypes::pdo_rx last_rx_pdo;
    auto tmp_rx_pdo = slave_as<LXM32iESC>(1)->getRxPDO();
    auto tmp_tx_pdo = slave_as<LXM32iESC>(1)->getTxPDO();

    if ( tmp_rx_pdo._DCOMstatus.all != last_rx_pdo._DCOMstatus.all ) {
    
        last_rx_pdo = tmp_rx_pdo;
        tmp_rx_pdo.dump(std::cout, "\n"); std::cout << "\n\n";
    }
    
    if ( tmp_rx_pdo._DCOMopmd_act == PPOS ) {

        if ( cazzo ) {
        
            tmp_tx_pdo.DCOMcontrol.dcom_control.b4_ = 0x1;
            //tmp_tx_pdo.PPp_target *= -1;
            if ( sp_it != slave_as<LXM32iESC>(1)->set_points.end() ) {
                tmp_tx_pdo.PPp_target = *sp_it; 
                sp_it ++;
                cazzo = 0;
            } else {
                sp_it = slave_as<LXM32iESC>(1)->set_points.begin();
                cazzo = 0;
            }
        }
        
        if ( tmp_rx_pdo._DCOMstatus.dcom_status.b10_target_reached &&
             tmp_rx_pdo._DCOMstatus.dcom_status.b12_operating_mode_specific &&
             tmp_rx_pdo._DCOMstatus.dcom_status.b14_x_end ) {
            
            std::cout << "... target reached " << tmp_rx_pdo._p_act << "\n";
            tmp_tx_pdo.DCOMcontrol.dcom_control.b4_ = 0x0;
            cazzo = 1;
        }
    }
    
    char cmd;
    if ( user_input( cmd ) == sizeof(cmd) ) {
        
        std::cout << cmd << "\n";
        tmp_rx_pdo.dump(std::cout, "\n"); std::cout << "\n\n";

        if ( tmp_rx_pdo._DCOMstatus.dcom_status.b3_Fault ) {
            tmp_tx_pdo.DCOMcontrol.dcom_control.b7_FaultReset = 1;
            DPRINTF("[%s]: reset fault 0x%X\n", __FUNCTION__, tmp_rx_pdo._LastError);
        } else {
            tmp_tx_pdo.DCOMcontrol.dcom_control.b7_FaultReset = 0;                            
        }

        // 
        switch (cmd) {
            case 'a' :
                tmp_tx_pdo.DCOMcontrol.all = 0x0;
                break;
            case 'b' :
                //tmp_tx_pdo.DCOMcontrol.all = 0x6;
                tmp_tx_pdo.DCOMcontrol.dcom_control.b0_SwitchOn = 0x0;
                tmp_tx_pdo.DCOMcontrol.dcom_control.b1_EnableVoltage = 0x1;
                tmp_tx_pdo.DCOMcontrol.dcom_control.b2_QuickStop = 0x1;
                tmp_tx_pdo.DCOMcontrol.dcom_control.b3_EnableOperation = 0x0;
                break;
            case 'c' :
                //tmp_tx_pdo.DCOMcontrol.all = 0xF;
                tmp_tx_pdo.DCOMcontrol.dcom_control.b0_SwitchOn = 0x1;
                tmp_tx_pdo.DCOMcontrol.dcom_control.b1_EnableVoltage = 0x1;
                tmp_tx_pdo.DCOMcontrol.dcom_control.b2_QuickStop = 0x1;
                tmp_tx_pdo.DCOMcontrol.dcom_control.b3_EnableOperation = 0x1;
                break;
            case 'j' :
                tmp_tx_pdo.DCOMopmode = JOG;
                tmp_tx_pdo.JOGactivate = POS_SLOW;
                break;
            case 'h' :
                // position 
                tmp_tx_pdo.DCOMopmode = HOMING;
                // start homing
                tmp_tx_pdo.DCOMcontrol.dcom_control.b4_ = 0x1;
                break;
            case 'p' :
                // position 
                tmp_tx_pdo.DCOMopmode = PPOS;
                tmp_tx_pdo.DCOMcontrol.dcom_control.b4_ = 0x1;
                // starts a movement to a target position
                // Target values transmitted during a movement become immediately effective and are executed at the target.
                // The movement is not stopped at the current target position
                //tmp_tx_pdo.DCOMcontrol.dcom_control.b5_ = 0x0;
                // Target values transmitted during a movement become immediately effective and are immediately executed.
                tmp_tx_pdo.DCOMcontrol.dcom_control.b5_ = 0x1;                
                // relative movement
                tmp_tx_pdo.DCOMcontrol.dcom_control.b6_ = 0x1;
                tmp_tx_pdo.PPp_target = *sp_it; 
                // absolute movement
                //tmp_tx_pdo.DCOMcontrol.dcom_control.b6_ = 0x0;
                //tmp_tx_pdo.PPp_target = *sp_it;
                sp_it ++;
                break;
            case 'v' :
                // velocity
                tmp_tx_pdo.DCOMopmode = PVEL;
                tmp_tx_pdo.PVv_target = 50;
                break;
            case 't' :
                // torque
                tmp_tx_pdo.DCOMopmode = PTOR;
                tmp_tx_pdo.PTtq_target = 30;
                break;
            case 'x' :
                if ( tmp_tx_pdo.JOGactivate == POS_SLOW ) {
                    tmp_tx_pdo.JOGactivate = NEG_SLOW;                    
                } else {
                    tmp_tx_pdo.JOGactivate = POS_SLOW;
                }
                tmp_tx_pdo.PVv_target *= -1;
                tmp_tx_pdo.PTtq_target *= -1;
                break;
            case 'X' :
                tmp_tx_pdo.PVv_target += 10;
                tmp_tx_pdo.PTtq_target += 10;
                break;
                
            default :
                break;
        }
    }
    
    //
    slave_as<LXM32iESC>(1)->setTxPDO(tmp_tx_pdo);

    
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on