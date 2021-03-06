/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/
/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_BOARDS_COMAN_TEST_H__
#define __EC_BOARDS_COMAN_TEST_H__

#include <iit/advr/ec_boards_base.h>
#include <iit/advr/trajectory.h>

/**
 */

class EC_boards_coman_test : public Ec_Thread_Boards_base {
public:

    enum user_state_t : int {
        HOMING 	= 1,
        HOME2MID,
        MID2HOME,
        ANY2HOME,
        MOVING,
        //
        IDLE,
    };
    enum home_state_t : int {
        LEFT_LEG_HOME	= 1,
        LEFT_ARM_HOME,
        RIGHT_LEG_HOME,
        RIGHT_ARM_HOME,
        WAIST_HOME,
        TEST_HOME,
        END_HOME,
    };

    EC_boards_coman_test ( const char * config_yaml );
    virtual ~EC_boards_coman_test();

    template<class C>
    int xddp_input ( C &user_cmd );
    int user_loop ( void );

private :

    virtual void init_preOP ( void );
    virtual void init_OP ( void );

    std::map<int,float> step_2;
    std::map<int,float> test_pos;
    

    advr::Trj_ptr_map spline_start2home;
    advr::Trj_ptr_map spline_home2test;
    advr::Trj_ptr_map spline_test2home;
    advr::Trj_ptr_map spline_any2home;

    XDDP_pipe jsInXddp, navInXddp, imuInXddp;

    user_state_t user_state;
    home_state_t home_state;

    std::map<int, iit::ecat::advr::Motor*> 	left_leg;
    std::map<int, iit::ecat::advr::Motor*> 	right_leg;
    std::map<int, iit::ecat::advr::Motor*> 	left_arm;;
    std::map<int, iit::ecat::advr::Motor*> 	right_arm;
    std::map<int, iit::ecat::advr::Motor*> 	waist;

    std::map<int, iit::ecat::advr::Motor*>  motors_to_start;
    
};




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
