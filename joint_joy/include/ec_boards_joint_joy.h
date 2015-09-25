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

#include <ec_boards_base.h>

#include <linux/joystick.h>
#include <spnav.h>

/**
 */

//typedef struct js_event	input_t;
typedef spnav_event 	input_t;

typedef XDDP_pipe<input_t,input_t> 	InXddp;

class EC_boards_joint_joy :
    public Ec_Thread_Boards_base,
    public InXddp
{
public:
    
    EC_boards_joint_joy(const char * config_yaml);
    virtual ~EC_boards_joint_joy();

    template<class C>
    int user_input(C &user_cmd);
    int user_loop(void);

private :
    
    virtual void init_preOP(void);
    virtual void init_OP(void);
    
    std::map<int, iit::ecat::advr::Motor*> motors;
    
    std::map<int,float> home;
    std::map<int,float> start_pos;

};




#endif
