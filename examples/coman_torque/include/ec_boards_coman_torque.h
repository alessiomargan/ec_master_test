/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_BOARDS_COMAN_TORQUE_H__
#define __EC_BOARDS_COMAN_TORQUE_H__

#include <iit/advr/ec_boards_base.h>
/**
 */

typedef struct js_event 		input_t;

class Ec_Boards_coman_torque : public Ec_Thread_Boards_base {
public:

    Ec_Boards_coman_torque ( const char * config_yaml );
    virtual ~Ec_Boards_coman_torque();

    template<class C>
    int user_input ( C &user_cmd );
    virtual int user_loop ( void );

private :

    virtual void init_preOP ( void );
    virtual void init_OP ( void );

    XDDP_pipe inXddp;

    std::map<int, iit::ecat::advr::Motor*>  motors_ctrl_tor;
    std::map<int, iit::ecat::advr::Motor*>  motors_ctrl_pos;

    advr::Trj_ptr_map spline_start2home;
    
};




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
