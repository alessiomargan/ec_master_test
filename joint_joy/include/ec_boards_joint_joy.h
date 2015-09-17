/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)
   
*/

/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_BOARDS_BASIC_H__
#define __EC_BOARDS_BASIC_H__

#include <linux/joystick.h>

#include <utils.h>
#include <thread_util.h>

#include <iit/ecat/advr/ec_boards_iface.h>
#include <iit/ecat/advr/pipes.h>

/**
 */

typedef struct js_event 		input_t;
typedef XDDP_pipe<input_t,input_t> 	InXddp;

class EC_boards_joint_joy :
    public Thread_hook,
    public iit::ecat::advr::Ec_Boards_ctrl,
    public InXddp
{
public:
    
    EC_boards_joint_joy(const char * config_yaml);
    virtual ~EC_boards_joint_joy();

    virtual void th_init(void *);
    virtual void th_loop(void *);

    template<class C>
    int user_input(C &user_cmd);
    int user_loop(void);

private :
    
    virtual void homing(void);
    
    iit::ecat::stat_t  s_loop;
    uint64_t start_time, tNow, tPre;
    
    iit::ecat::advr::Rid2PosMap	rid2pos;
    
    std::map<int,float> home_rId;
    std::map<int,float> start_pos_rId;

};




#endif
