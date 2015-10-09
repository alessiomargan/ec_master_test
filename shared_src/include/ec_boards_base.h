/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)
   
*/

/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_THREAD_BOARDS_BASE_H__
#define __EC_THREAD_BOARDS_BASE_H__


#include <utils.h>
#include <thread_util.h>

#include <iit/ecat/advr/ec_boards_iface.h>

/**
 */

enum user_state : int
{ 
    // waist
    HOMING	= 1,
    MOVING,
};


class Ec_Thread_Boards_base :
    public Thread_hook,
    public iit::ecat::advr::Ec_Boards_ctrl
{
public:
    
    Ec_Thread_Boards_base(const char * config_yaml) :Ec_Boards_ctrl(config_yaml) {}
    virtual ~Ec_Thread_Boards_base() {
	 iit::ecat::print_stat(s_loop);
    }

    virtual void th_init(void *);
    virtual void th_loop(void *);
    
    virtual int user_loop(void) = 0;

protected :
    
    virtual void init_preOP(void) = 0;
    virtual void init_OP(void) = 0;
    
    iit::ecat::stat_t  s_loop;
    uint64_t start_time, tNow, tPre;
    
    iit::ecat::advr::Rid2PosMap	rid2pos;
    iit::ecat::advr::Pos2RidMap	pos2rid;
    
    std::map<int,float> home;
    std::map<int,float> start_pos;

};


inline void Ec_Thread_Boards_base::th_init(void *) {
    
    // init Ec_Boards_ctrl
    if ( Ec_Boards_ctrl::init() != iit::ecat::advr::EC_BOARD_OK) {
	throw "something wrong";
    }
    // get Robot_Id map 
    rid2pos = get_Rid2PosMap();
    pos2rid = get_Pos2RidMap();
    
    init_preOP();
    
    if ( set_operative() <= 0 ) {
	throw "something else wrong";
    }
    
    start_time = iit::ecat::get_time_ns();
    tNow, tPre = start_time;
    
    init_OP();
	
}

inline void Ec_Thread_Boards_base::th_loop(void *) {
	
    tNow = get_time_ns();
    s_loop(tNow - tPre);
    tPre = tNow;
    
    try {
	
	if ( recv_from_slaves() != iit::ecat::advr::EC_BOARD_OK ) {
	    // TODO
	    DPRINTF("recv_from_slaves FAIL !\n");
	    return;
	}
	    
	user_loop();

	send_to_slaves();	
	
    } catch (iit::ecat::EscWrpError &e) {
	    std::cout << e.what() << std::endl;
    }
    
}


#endif
