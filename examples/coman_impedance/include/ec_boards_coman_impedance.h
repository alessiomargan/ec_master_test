/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_BOARDS_COMAN_IMPEDANCE_H__
#define __EC_BOARDS_COMAN_IMPEDANCE_H__

#include <iit/advr/ec_boards_base.h>
/**
 */

typedef struct js_event 		input_t;

class Ec_Boards_coman_impedance : public Ec_Thread_Boards_base {
public:

    Ec_Boards_coman_impedance ( const char * config_yaml );
    virtual ~Ec_Boards_coman_impedance();

    template<class C>
    int user_input ( C &user_cmd );
    virtual int user_loop ( void );

private :

    virtual void init_preOP ( void );
    virtual void init_OP ( void );

    XDDP_pipe inXddp;

    std::map<int, iit::ecat::advr::Motor*>  motors_ctrl_imp;
    std::map<int, iit::ecat::advr::Motor*>  motors_ctrl_pos;
    std::map<int, iit::ecat::advr::Motor*>  motors_moving;

    std::map<int,float> test_pos;
    
    advr::Spline_map spline_start2home;
    advr::Spline_map spline_home2test_pos;
    advr::Spline_map spline_test_pos2home;
    
};




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
