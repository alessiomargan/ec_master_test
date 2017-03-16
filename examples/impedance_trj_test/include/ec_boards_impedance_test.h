/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/
/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_BOARDS_IMPEDANCE_TEST_H__
#define __EC_BOARDS_IMPEDANCE_TEST_H__

#include <iit/advr/ec_boards_base.h>
#include <iit/advr/trajectory.h>

/**
 */

class EC_boards_impedance_test : public Ec_Thread_Boards_base {
public:

    EC_boards_impedance_test ( const char * config_yaml );
    virtual ~EC_boards_impedance_test();

    template<class C>
    int xddp_input ( C &user_cmd );
    int user_loop ( void );

private :

    virtual void init_preOP ( void );
    virtual void init_OP ( void );
    
    
    std::map<int,float> test_pos;

    std::queue<advr::ImpTrj_ptr_map *> imp_trj_queue;

    advr::ImpTrj_ptr_map trj_start2home;
    advr::ImpTrj_ptr_map trj_home2test_pos2home;

    XDDP_pipe jsInXddp;

    std::map<int, iit::ecat::advr::Motor*>  motors2ctrl;
    std::map<int, iit::ecat::advr::Motor*>  motors2move;
    std::map<int, iit::ecat::advr::Motor*>  motors2torque;
};




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
