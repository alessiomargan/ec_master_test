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


#include <iit/advr/thread_util.h>

#include <iit/ecat/advr/ec_boards_iface.h>
#include <iit/advr/pipes.h>

/**
 */

		  
class Ec_Thread_Boards_base :
    public Thread_hook,
    public iit::ecat::advr::Ec_Boards_ctrl
{
public:
    
    Ec_Thread_Boards_base(const char * config_yaml) :Ec_Boards_ctrl(config_yaml) {}
    virtual ~Ec_Thread_Boards_base() {
	 iit::ecat::print_stat(s_loop);
    }

    bool init_OK() { return init_done; };

    virtual void th_init(void *);
    virtual void th_loop(void *);
    
    virtual int user_loop(void) = 0;

protected :
    
    virtual void init_preOP(void) = 0;
    virtual void init_OP(void) = 0;
    
    void xddps_init(void);
    void xddps_loop(void);
    
    iit::ecat::stat_t  s_loop;
    uint64_t start_time, tNow, tPre;
    
    iit::ecat::advr::Rid2PosMap	rid2pos;
    iit::ecat::advr::Pos2RidMap	pos2rid;
    
    std::map<int,float> home;
    std::map<int,float> start_pos;

    std::map<int,XDDP_pipe*> xddps;

    std::map<int, iit::ecat::advr::Motor*> 	motors;
    std::map<int, iit::ecat::advr::Ft6ESC*> 	fts;
    std::map<int, iit::ecat::advr::PowESC*> 	pows;
    std::map<int, iit::ecat::advr::PowComanESC*>powCmns;
    std::map<int, iit::ecat::advr::TestESC*> 	tests;

private:
    
    bool init_done;
    
};



#endif
