/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/
/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_BOARDS_WALKMAN_TEST_H__
#define __EC_BOARDS_WALKMAN_TEST_H__

#include <iit/advr/ec_boards_base.h>
#include <iit/advr/trajectory.h>

/**
 */

class EC_boards_walkman_test : public Ec_Thread_Boards_base {
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

    EC_boards_walkman_test ( const char * config_yaml );
    virtual ~EC_boards_walkman_test();

    template<class C>
    int xddp_input ( C &user_cmd );
    int user_loop ( void );

private :

    virtual void init_preOP ( void );
    virtual void init_OP ( void );
    
    void move_head(float, float);
    void move_hands(float, float);
    
    std::map<int,float> test_pos;

    advr::Trj_ptr_map trj_start2home;
    advr::Trj_ptr_map trj_home2test_pos2home;
    advr::Trj_ptr_map trj_any2home;

    advr::ImpTrj_ptr_map imp_trj;

    XDDP_pipe jsInXddp, navInXddp, imuInXddp;

    std::map<int, iit::ecat::advr::Motor*>  head;
    std::map<int, iit::ecat::advr::Motor*> 	left_arm;;
    std::map<int, iit::ecat::advr::Motor*> 	right_arm;
    std::map<int, iit::ecat::advr::Motor*> 	hands;
    std::map<int, iit::ecat::advr::Motor*>  waist;
    std::map<int, iit::ecat::advr::Motor*>  left_leg;
    std::map<int, iit::ecat::advr::Motor*>  right_leg;

    std::map<int, iit::ecat::advr::Motor*>  motors2ctrl;
    std::map<int, iit::ecat::advr::Motor*>  motors2move;
    std::map<int, iit::ecat::advr::Motor*>  motors2torque;
};




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 