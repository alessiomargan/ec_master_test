/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/
/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_BOARDS_JOINT_JOY_H__
#define __EC_BOARDS_JOINT_JOY_H__

#include <iit/advr/ec_boards_base.h>
#include <iit/advr/trajectory.h>

/**
 */

class EC_boards_joint_joy : public Ec_Thread_Boards_base {
public:

    enum user_state_t : int {
        HOMING	= 1,
        MOVING,
        STEP_1,
        STEP_2,
        TRJ_1,
        TRJ_2,
        ANY2HOME,
        //
        IDLE,
    };

    EC_boards_joint_joy ( const char * config_yaml );
    virtual ~EC_boards_joint_joy();

    template<class C>
    int user_input ( C &user_cmd );
    int user_loop ( void );

private :

    virtual void init_preOP ( void );
    virtual void init_OP ( void );

    std::map<int,float> step_1;
    std::map<int,float> step_2;

    advr::Trj_ptr_map spline1_trj;
    advr::Trj_ptr_map spline2_trj;
    advr::Trj_ptr_map spline_start2home;
    advr::Trj_ptr_map spline_any2home;

    XDDP_pipe jsInXddp, navInXddp;

    user_state_t user_state;
};




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
