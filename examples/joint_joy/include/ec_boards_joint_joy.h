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


/**
 */


class EC_boards_joint_joy : public Ec_Thread_Boards_base
{
public:
    
    enum user_state_t : int
    { 
	HOMING	= 1,
	MOVING,
	STEP_1,
	STEP_2
    };
   
    EC_boards_joint_joy(const char * config_yaml);
    virtual ~EC_boards_joint_joy();

    template<class C>
    int user_input(C &user_cmd);
    int user_loop(void);

private :
    
    virtual void init_preOP(void);
    virtual void init_OP(void);
//     bool go_there(std::map<int, iit::ecat::advr::Motor*> motor_set,
// 		  std::map<int,float> target_pos,
// 		  float eps);
    
    std::map<int,float> step_1;
    std::map<int,float> step_2;
    
    XDDP_pipe jsInXddp, navInXddp;
    
    user_state_t user_state;
};




#endif
