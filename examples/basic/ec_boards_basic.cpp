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
    schedpolicy = SCHED_RR;
#endif
    priority = sched_get_priority_max ( schedpolicy );
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    // open pipe ... xeno xddp or fifo
    inXddp.init ( "EC_board_input" );

}

Ec_Boards_basic::~Ec_Boards_basic() {
    
    std::cout << "~" << typeid ( this ).name() << std::endl;
}

void Ec_Boards_basic::init_preOP ( void ) {

    int slave_pos, rId;
    Motor * moto;

#if 0    
    for ( auto const& item : motors ) {
        slave_pos = item.first;
        rId = pos2Rid(slave_pos);
        moto = item.second;

        if ( dynamic_cast<CentAcESC*>(moto) ) {
            if ( std::set<int>({11,12}).count(rId) == 1 ) {
                dynamic_cast<CentAcESC*>(moto)->set_zero_position(M_PI+DEG2RAD(-45) );
            } else if ( std::set<int>({21,22}).count(rId) == 1 ) {
                dynamic_cast<CentAcESC*>(moto)->set_zero_position(M_PI+DEG2RAD(+45) );             
            } else {
                dynamic_cast<CentAcESC*>(moto)->set_zero_position(M_PI);
            }
            
        }
    }
#endif
#if 0
    // get first motor ....
    CentAcESC * treeAct = slave_as<CentAcESC>(rid2Pos(1)); 
    if ( treeAct ) {
        
        if ( treeAct->run_torque_calibration( ) ) {
            DPRINTF ( ">> [%d] Fail torque calibration\n",rid2Pos(1) );
        }    
    } 
#endif

    
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
    //get_esc_map_byclass ( motors,  motor_rid );

    int slave_pos;
    Motor * moto;
    float pos_ref;
    for ( auto const& item : motors ) {
        slave_pos = item.first;
        moto = item.second;
        
        //auto motor_pdo_rx = moto->getRxPDO();
        //pos_ref = motor_pdo_rx.link_pos + 0.01;
        //moto->set_posRef( pos_ref );
            
        // avoid segfault if no CentAcESC
        if ( dynamic_cast<CentAcESC*>(moto) ) { 
            set_ctrl_status_X ( dynamic_cast<CentAcESC*>(moto), CTRL_POWER_MOD_ON );
            set_ctrl_status_X ( dynamic_cast<CentAcESC*>(moto), CTRL_FAN_ON );
        }
        
        
    }

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

    return bytes;
}

int Ec_Boards_basic::user_loop ( void ) {

    //auto const rd_sdos = { "motorEncBadReadPPM","linkEncBadReadPPM","torque_read" };
    auto const rd_sdos = { "torque_read" };
    std::vector<float> rd_sdo_values(rd_sdos.size());     
    
    char what = 0;
    user_input ( what );
    
    if ( what == 'a' ) {
        int slave_pos;
        Motor * moto;
        for ( auto const& item : motors ) {
            slave_pos = item.first;
            moto = item.second;
            auto cm = dynamic_cast<CentAcESC*>(moto);
            if ( ! cm )
                continue;
            
            if ( cm ) { 
                //set_ctrl_status_X ( cm, CTRL_POWER_MOD_ON );
            }
            
            auto tmp_it = rd_sdo_values.begin();
            for ( auto const sdo_name :  rd_sdos ) {
                cm->readSDO_byname ( sdo_name, *tmp_it );
                tmp_it++;
            }
            DPRINTF ( ">> [%d] %f %f %f\n", pos2Rid( slave_pos ), rd_sdo_values[0], rd_sdo_values[1], rd_sdo_values[2] );
        }
    }        
}
   


// kate: indent-mode cstyle; indent-width 4; replace-tabs on