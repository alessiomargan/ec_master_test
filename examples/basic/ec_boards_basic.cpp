#include <ec_boards_basic.h>

#include <iit/advr/coman_robot_id.h>
#include <iit/advr/walkman_robot_id.h>
#include <iit/advr/centauro_robot_id.h>

#define MID_POS(m,M)    (m+(M-m)/2)

using namespace iit::ecat::advr;

Ec_Boards_basic::Ec_Boards_basic ( const char* config_yaml ) : Ec_Thread_Boards_base ( config_yaml ) {

    name = "EC_boards_basic";
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

Ec_Boards_basic::~Ec_Boards_basic() {
    
    std::cout << "~" << typeid ( this ).name() << std::endl;
}

void Ec_Boards_basic::init_preOP ( void ) {

    
}

void Ec_Boards_basic::init_OP ( void ) {

    std::vector<int> motor_rid = centauro::robot_mcs_ids;
    //std::vector<int> motor_rid = std::initializer_list<int> { };
#if 0
    std::vector<int> motor_rid = std::initializer_list<int> {
        centauro::WAIST_Y,
        centauro::RA_SH_1,
        centauro::RA_SH_2,
        centauro::RA_SH_3,
        centauro::RA_EL,
        centauro::RA_WR_1,
        centauro::RA_WR_2,
        centauro::RA_WR_3,
        centauro::LA_SH_1,
        centauro::LA_SH_2,
        centauro::LA_SH_3,
        centauro::LA_EL,
        centauro::LA_WR_1,
        centauro::LA_WR_2,
        centauro::LA_WR_3,
    };
#endif
    get_esc_map_byclass ( motors,  motor_rid );

    int slave_pos;
    Motor * moto;
    for ( auto const& item : motors ) {
        slave_pos = item.first;
        moto = item.second;
        // avoid segfault if no CentAcESC
        if ( dynamic_cast<CentAcESC*>(moto) ) {
            set_ctrl_status_X ( dynamic_cast<CentAcESC*>(moto), CTRL_POWER_MOD_ON );
            set_ctrl_status_X ( dynamic_cast<CentAcESC*>(moto), CTRL_FAN_ON );
        }
        
    }

#if 0
    // get first motor ....
    CentAcESC * moto = slave_as<CentAcESC>(1); 
    if ( moto ) {
        moto->run_torque_calibration( );
    }
#endif

}

template<class C>
int Ec_Boards_basic::user_input ( C &user_cmd ) {

    static int  bytes_cnt;
    int         bytes;
    input_t     cmd;

    if ( ( bytes = inXddp.xddp_read ( user_cmd ) ) <= 0 ) {
        return bytes;
    }

    bytes_cnt += bytes;
    DPRINTF ( ">> %d %d\n",bytes, bytes_cnt );
    //DPRINTF(">> %d\n",cmd.value);

    return bytes;
}

int Ec_Boards_basic::user_loop ( void ) {

    char what = 0;
    user_input ( what );
    
    if ( what == 'a' ) {

        int slave_pos;
        Motor * moto;
        for ( auto const& item : motors ) {
            slave_pos = item.first;
            moto = item.second;
            set_ctrl_status_X ( dynamic_cast<CentAcESC*>(moto), CTRL_POWER_MOD_ON );
        }
        
    }
   
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on