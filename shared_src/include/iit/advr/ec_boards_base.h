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
#include <iit/ecat/advr/pipes.h>
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
    Ec_Thread_Boards_base ( const char * config_yaml );

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

    std::queue<advr::Trj_ptr_map *> trj_queue;
    advr::Trj_ptr_map * running_trj;
    advr::Trj_ptr_map * last_run_trj;

    XDDP_pipe termInXddp;
    XDDP_pipe debugOutXddp;
    
    std::map<int, iit::ecat::advr::Motor*>          motors;
    std::map<int, iit::ecat::advr::Ft6ESC*>         fts;
    std::map<int, iit::ecat::advr::FootSensorESC*>  foot_sensors;
    std::map<int, iit::ecat::advr::PowESC*>         pows;
    std::map<int, iit::ecat::advr::PowComanESC*>    powCmns;
    std::map<int, iit::ecat::advr::TestESC*>        tests;

    void remove_rids_intersection(std::vector<int> &, const std::vector<int> &);
    
    bool go_there ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                    const std::map<int,float> &target_pos,
                    float eps, bool debug = false );
    bool go_there ( const std::map<int,float> &target_pos,
                    float eps, bool debug = false );

    bool go_there ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                    const advr::Trj_ptr_map &spline_map_trj,
                    float eps, bool debug = false );
    bool go_there ( const advr::Trj_ptr_map &spline_map_trj,
                    float eps, bool debug = false );


    void get_trj_for_end_points ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                                  advr::Trj_ptr_map &new_spline_trj,
                                  std::map<int,float> &end_points,
                                  float secs );
    void get_trj_for_end_points ( advr::Trj_ptr_map &new_spline_trj,
                                  std::map<int,float> &end_points,
                                  float secs );

    void smooth_splines_trj ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                              advr::Trj_ptr_map &new_spline_trj,
                              const advr::Trj_ptr_map &old_spline_trj,
                              double smooth_time=0.5 );
    void smooth_splines_trj ( advr::Trj_ptr_map &new_spline_trj,
                              const advr::Trj_ptr_map &old_spline_trj,
                              double smooth_time=0.5 );

    void set_any2home ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                        advr::Trj_ptr_map &new_spline_trj,
                        advr::Trj_ptr_map &old_spline_trj );
    void set_any2home ( advr::Trj_ptr_map &new_spline_trj,
                        advr::Trj_ptr_map &old_spline_trj );

private:

    iit::ecat::ec_timing_t timing;
    
};




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
