/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_BOARDS_SINE_H__
#define __EC_BOARDS_SINE_H__

#include <linux/joystick.h>
#include <iit/advr/ec_boards_base.h>
/**
 */

typedef struct js_event input_t;
typedef XDDP_pipe	InXddp;

class Ec_Boards_sine :
    public Ec_Thread_Boards_base {
public:

    Ec_Boards_sine ( const char * config_yaml );
    virtual ~Ec_Boards_sine();

    template<class C>
    int user_input ( C &user_cmd );
    virtual int user_loop ( void );

private :

    virtual void init_preOP ( void );
    virtual void init_OP ( void );

    XDDP_pipe inXddp;
    
    std::map<int, iit::ecat::advr::Motor*>  motors_to_start;
    advr::Trj_name_ptr_map trj_map;
    advr::Trj_vector_ptr_map ptr_map_vec;
    
};




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
