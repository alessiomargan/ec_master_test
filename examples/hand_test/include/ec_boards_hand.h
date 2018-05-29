/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_BOARDS_HAND_H__
#define __EC_BOARDS_HAND_H__

#include <iit/advr/ec_boards_base.h>
/**
 */

typedef XDDP_pipe	InXddp;

class Ec_Boards_hand :
    public Ec_Thread_Boards_base {
public:

    Ec_Boards_hand ( const char * config_yaml );
    virtual ~Ec_Boards_hand();

    template<class C>
    int user_input ( C &user_cmd );
    virtual int user_loop ( void );

private :

    virtual void init_preOP ( void );
    virtual void init_OP ( void );
    int check_force_finger( void );
    
    //std::map<int, iit::ecat::advr::LpHandESC*>  fingers;
    std::map<int, iit::ecat::advr::HeriHandESC*>  fingers;
    
    XDDP_pipe inXddp;
};




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
