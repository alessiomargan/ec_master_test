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

private :

    virtual void init_preOP ( void );
    virtual void init_OP ( void );

    XDDP_pipe inXddp;
};




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
