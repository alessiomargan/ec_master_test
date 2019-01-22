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

#include <protobuf/ec_boards_base_input.pb.h>

#include <queue>

#define ECAT_PTHREAD_STACK_SIZE (16*1024*1024) // 16MB

/**
 */


class Ec_Thread_Boards_base :
    public Thread_hook,
    public iit::ecat::advr::Ec_Boards_ctrl {
public:

    virtual ~Ec_Thread_Boards_base();
    Ec_Thread_Boards_base ( std::string config_yaml );

    virtual void th_init ( void * );
    virtual void th_loop ( void * );

    virtual int user_loop ( void ) = 0;
    virtual int repl_loop ( void );

protected :

    virtual void init_preOP ( void ) = 0;
    virtual void init_OP ( void ) = 0;

    virtual void stop_motors ( void );
    
    iit::ecat::stat_t  s_loop;
    uint64_t start_time, tNow, tPre;

    std::map<int,float> home;
    std::map<int,float> start_pos;

    std::deque<advr::Trj_ptr_map> trj_queue;

    uint32_t    emergency_active;
    
    XDDP_pipe   emergencyInXddp;
    XDDP_pipe   replInXddp;
    XDDP_pipe   debugOutXddp;
    
    std::map<int, iit::ecat::advr::Motor*>          motors;
    std::map<int, iit::ecat::advr::Ft6ESC*>         fts;
    iit::ecat::advr::FootSensor_16x8_SlavesMap      foot_16x8;
    iit::ecat::advr::FootSensor_10x5_SlavesMap      foot_10x5;
    iit::ecat::advr::SkinSensor_8x3_SlavesMap       skin_8x3;
    std::map<int, iit::ecat::advr::PowESC*>         pows;
    std::map<int, iit::ecat::advr::PowF28M36ESC*>   powF28M36s;
    std::map<int, iit::ecat::advr::PowComanESC*>    powCmns;
    std::map<int, iit::ecat::advr::ImuVnESC*>       imus;
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

    bool set_impedance_refs ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                              const advr::ImpTrj_ptr_map &imp_trj_map,
                              float eps, bool debug );


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

    iit::ecat::ec_timing_t  timing;
    
    // YAML_Node()["ec_boards_base"]["app_mode"] = "run_mode OR "config_mode"
    bool run_mode;


};


inline void Ec_Thread_Boards_base::stop_motors ( void )  {
    iit::ecat::advr::Motor * motor;
    get_esc_map_byclass ( motors );
    for ( auto const& item : motors ) {
        motor = item.second;
        motor->stop();
    }
}

template<typename TK, typename TV>
std::vector<TK> extract_keys(std::map<TK, TV> const& input_map) {
    std::vector<TK> retval;
    for (auto const& element : input_map) {
        retval.push_back(element.first);
    }
    return retval;
}

template<typename TK, typename TV>
std::vector<TV> extract_values(std::map<TK, TV> const& input_map) {
    std::vector<TV> retval;
    for (auto const& element : input_map) {
        retval.push_back(element.second);
    }
    return retval;
}




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
