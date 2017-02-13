#include <ec_boards_coman_impedance.h>
#include <iit/advr/coman_robot_id.h>

#define MID_POS(m,M)    (m+(M-m)/2)


namespace iit {
namespace ecat {
namespace advr {
namespace coman {  

    std::map<int, float> robot_ids_imp_pos_deg = {

        {WAIST_Y, 0.0}, {WAIST_P, 0.0}, {WAIST_R, 0.0},

        {RL_H_R, 0.0}, {RL_H_Y, 0.0},   {RL_H_P, -50.0},
        {RL_K,  70.0}, {RL_A_P, 10.0}, {RL_A_R,  0.0},

        {LL_H_R, 0.0}, {LL_H_Y,  0.0},   {LL_H_P, -50.0},
        {LL_K,  70.0}, {LL_A_P,  10.0}, {LL_A_R,    0.0},

        // home
        {RA_SH_1, 30.0}, {RA_SH_2, 70.0}, {RA_SH_3, 0.0}, {RA_EL, -90.0},
        {RA_WR_1, 0.0}, {RA_WR_2, 0.0}, {RA_WR_3, 0.0}, {RA_HA, 0.0},

        {LA_SH_1, 30.0}, {LA_SH_2,-70.0}, {LA_SH_3, 0.0}, {LA_EL, -90.0},
        {LA_WR_1, 0.0}, {LA_WR_2, 0.0}, {LA_WR_3, 0.0}, {LA_HA, 0.0}

    };
}}}}

using namespace iit::ecat::advr;

static const std::vector<double> Xt_3s = std::initializer_list<double> { 0, 3 };
static const std::vector<double> Xt_5s = std::initializer_list<double> { 0, 5 };

Ec_Boards_coman_impedance::Ec_Boards_coman_impedance ( const char* config_yaml ) : Ec_Thread_Boards_base ( config_yaml ) {

    name = "Ec_Boards_coman_impedance";
    // non periodic
    period.period = {0,1};

#ifdef __COBALT__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max ( schedpolicy );
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    // open pipe ... xeno xddp or fifo
    inXddp.init ( "EC_board_input" );

}

Ec_Boards_coman_impedance::~Ec_Boards_coman_impedance() {

}

void Ec_Boards_coman_impedance::init_preOP ( void ) {

    Motor * moto;
    LpESC * lp_moto;
    
    std::vector<double> Ys;

    int slave_pos;
    float min_pos, max_pos;
    int16_t torque;

    std::vector<int> pos_rid = coman::robot_mcs_ids;
    std::vector<int> no_control = std::initializer_list<int> {
        coman::RA_HA,
        coman::LA_HA,
    };
    std::vector<int> imp_rid = std::initializer_list<int> {
        // right arm
        coman::RA_SH_1,
        coman::RA_SH_2,
        coman::RA_SH_3,
        coman::RA_EL,
        coman::RA_WR_1,
        // left arm
        coman::LA_SH_1,
        coman::LA_SH_2,
        coman::LA_SH_3,
        coman::LA_EL,
        coman::LA_WR_1,
    };

    remove_rids_intersection(pos_rid, no_control);
    
    remove_rids_intersection(pos_rid, imp_rid);

    get_esc_map_byclass ( motors_ctrl_pos,  pos_rid );
    for ( auto const& item : motors_ctrl_pos ) {

        slave_pos = item.first;
        moto = item.second;
        //////////////////////////////////////////////////
        moto->readSDO ( "link_pos", start_pos[slave_pos] );
        // home
        home[slave_pos] = DEG2RAD ( coman::robot_ids_home_pos_deg[pos2Rid(slave_pos)] );
        test_pos[slave_pos] = DEG2RAD ( coman::robot_ids_imp_pos_deg[pos2Rid(slave_pos)] );
        DPRINTF ( "Joint_id %d start %f home %f test_pos %f\n", pos2Rid ( slave_pos ), start_pos[slave_pos], home[slave_pos], test_pos[slave_pos] );
        Ys =  std::initializer_list<double> { start_pos[slave_pos], home[slave_pos] };
        spline_start2home[slave_pos].set_points ( Xt_5s, Ys );
        Ys = std::initializer_list<double> { home[slave_pos], test_pos[slave_pos] };
        spline_home2test_pos[slave_pos].set_points ( Xt_5s, Ys );
        Ys = std::initializer_list<double> { test_pos[slave_pos], home[slave_pos] };
        spline_test_pos2home[slave_pos].set_points ( Xt_5s, Ys );

        //////////////////////////////////////////////////
        // start controller :
        moto->start ( CTRL_SET_POS_MODE);
    }

   get_esc_map_byclass ( motors_ctrl_imp,  imp_rid );
    for ( auto const& item : motors_ctrl_imp ) {

        slave_pos = item.first;
        moto = item.second;
        //moto->readSDO("torque", torque);    
        //DPRINTF ( ">>> before cmd torque %d\n", torque );
        // reset torque offset
        //set_flash_cmd(slave_pos, 0x00CD);
        //moto->readSDO("torque", torque);    
        //DPRINTF ( ">>> after cmd torque %d\n", torque );
        ///////////////////////////////////////////////////
        // start controller :
        // - read actual joint position and set as pos_ref
        // - use impedance gains defined in config_yaml file or default value stored in flash  
        moto->start ( CTRL_SET_IMPED_MODE );
    }

    DPRINTF ( ">>> wait xddp terminal ....\n" );
    char c; while ( termInXddp.xddp_read ( c ) <= 0 ) { osal_usleep(100); }
    
    if ( motors_ctrl_pos.size() > 0 ) {
        q_spln.push ( &spline_start2home );
    }
}

void Ec_Boards_coman_impedance::init_OP ( void ) {

    if ( ! q_spln.empty() ) {
        running_spline = q_spln.front();
        last_run_spline = running_spline;
        advr::reset_trj ( *running_spline );
    }
    
    DPRINTF ( "End Init_OP\n" );

}

int Ec_Boards_coman_impedance::user_loop ( void ) {

    static const float spline_error = 0.07;
    static const float imp_spline_error = -1.0;

    int what;
    user_input ( what );
  
    if ( ! q_spln.empty() ) {
        running_spline = q_spln.front();
        if ( running_spline ) {
            // !@#%@$#%^^# ... tune error
            if ( go_there ( motors_ctrl_pos, *running_spline, spline_error, true) ) 
            {
                // running spline has finish !!
                last_run_spline = running_spline;
                q_spln.pop();
                if ( ! q_spln.empty() ) {
                    running_spline = q_spln.front();
                    smooth_splines_trj ( motors_ctrl_pos, *running_spline, *last_run_spline );
                    advr::reset_trj ( *running_spline );
                }
            }
        } else {
            DPRINTF ( "Error NULL running spline ... pop it\n" );
            q_spln.pop();
        }
        
    } else {
        
        running_spline = last_run_spline = 0;
#if 1            
        if ( motors_ctrl_pos.size() > 0 ) {
            // add splines ....
            q_spln.push ( &spline_home2test_pos );
            q_spln.push ( &spline_test_pos2home );
            // !!! since queue was empty reset the first spline
            running_spline = q_spln.front();
            last_run_spline = running_spline;
            advr::reset_trj ( *running_spline );
        }
#endif

    }
}

template<class C>
int Ec_Boards_coman_impedance::user_input ( C &user_cmd ) {

    static int  bytes_cnt;
    int     bytes;
    char    cmd;

    if ( ( bytes = inXddp.xddp_read ( cmd ) ) <= 0 ) {
        return bytes;
    }

    bytes_cnt += bytes;
    DPRINTF ( ">> %d %d\n",bytes, bytes_cnt );

    return bytes;
}


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
