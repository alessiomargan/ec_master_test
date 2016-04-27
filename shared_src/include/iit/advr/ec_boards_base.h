/*
   Copyright (C) Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_THREAD_BOARDS_BASE_H__
#define __EC_THREAD_BOARDS_BASE_H__

#include <iit/ecat/advr/ec_boards_iface.h>

#include <iit/advr/thread_util.h>
#include <iit/advr/pipes.h>
#include <iit/advr/trajectory.h>

#include <queue>

#define ECAT_PTHREAD_STACK_SIZE (16*1024*1024) // 16MB

/**
 */


class Ec_Thread_Boards_base :
    public Thread_hook,
    public iit::ecat::advr::Ec_Boards_ctrl {
public:

    virtual ~Ec_Thread_Boards_base();
    Ec_Thread_Boards_base ( const char * config_yaml ) : Ec_Boards_ctrl ( config_yaml ) {
            termInXddp.init ( "terminal" );
    }

    bool init_OK() {
        return init_done;
    };

    virtual void th_init ( void * );
    virtual void th_loop ( void * );

    virtual int user_loop ( void ) = 0;

protected :

    virtual void init_preOP ( void ) = 0;
    virtual void init_OP ( void ) = 0;

    iit::ecat::stat_t  s_loop;
    uint64_t start_time, tNow, tPre;

    std::map<int,float> home;
    std::map<int,float> start_pos;

    std::queue<advr::Spline_map *> q_spln;
    advr::Spline_map * running_spline;
    advr::Spline_map * last_run_spline;

    void xddps_init ( void );
    void xddps_loop ( void );
    std::map<int,XDDP_pipe*> xddps;

    XDDP_pipe termInXddp;

    std::map<int, iit::ecat::advr::Motor*> motors;
    std::map<int, iit::ecat::advr::Ft6ESC*> 	fts;
    std::map<int, iit::ecat::advr::FootSensorESC*> foot_sensors;
    std::map<int, iit::ecat::advr::PowESC*> 	pows;
    std::map<int, iit::ecat::advr::PowComanESC*>powCmns;
    std::map<int, iit::ecat::advr::TestESC*> 	tests;

    bool go_there ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                    const std::map<int,float> &target_pos,
                    float eps, bool debug = false );

    //template <typename T>
    bool go_there ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                    //const std::map<int,advr::trajectory<T>> &spline_map_trj,
                    const advr::Spline_map &spline_map_trj,
                    float eps, bool debug = false );


    void get_trj_for_end_points ( advr::Spline_map &new_spline_trj,
                                  std::map<int,float> &end_points,
                                  float secs );

    void smooth_splines_trj ( advr::Spline_map &new_spline_trj,
                              const advr::Spline_map &old_spline_trj,
                              double smooth_time=0.5 );

    void set_any2home ( advr::Spline_map &new_spline_trj, advr::Spline_map &old_spline_trj );


private:

    bool init_done;
    iit::ecat::ec_timing_t timing;
    
};




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
