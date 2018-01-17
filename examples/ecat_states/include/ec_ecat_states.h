/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_ECAT_STATES_H__
#define __EC_ECAT_STATES_H__

#include <iit/advr/ec_boards_base.h>
/**
 */


class Ec_Ecat_states : public Ec_Thread_Boards_base {
public:

    Ec_Ecat_states ( const char * config_yaml );
    virtual ~Ec_Ecat_states();

    template<class C>
    int user_input ( C &user_cmd );
    virtual int user_loop ( void );
    
    bool go_there ( const std::map<int, iit::ecat::advr::LXM32iESC*> &motor_set,
                    const advr::Trj_ptr_map &trj_map,
                    float eps, bool debug );
    bool go_there ( const std::map<int, iit::ecat::advr::LXM32iESC*> &motor_set,
                    const std::map<int,float> &target_pos,
                    float eps, bool debug );

private :

    virtual void init_preOP ( void );
    virtual void init_OP ( void );

    XDDP_pipe inXddp;
    
    std::map<int, iit::ecat::advr::LXM32iESC*> lxm32i;
    advr::Trj_name_ptr_map trj_map;
    std::vector<std::map<int,float>> trj_points;
    std::map<int,float> first_pos;
    std::vector<std::map<int,float>>::iterator tp_it;
};




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
