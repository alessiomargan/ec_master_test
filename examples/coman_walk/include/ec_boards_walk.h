/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_BOARDS_COMAN_WALK_H__
#define __EC_BOARDS_COMAN_WALK_H__

#include <linux/joystick.h>

#include <iit/advr/ec_boards_base.h>
#include <iit/advr/trajectory.h>


/**
 */

typedef struct js_event 		input_t;
// typedef XDDP_pipe<input_t,input_t> 	InXddp;
//typedef XDDP_pipe<char,char>  InXddp;

class EC_boards_walk : public Ec_Thread_Boards_base {
public:

    EC_boards_walk ( const char * config_yaml );
    virtual ~EC_boards_walk();

    template<class C>
    int user_input ( C &user_cmd );
    int user_loop ( void );

private :

    virtual void init_preOP ( void );
    virtual void init_OP ( void );

    int user_loop_walk ( void );
    int user_loop_test_joint ( void );

    iit::ecat::advr::Rid2PosMap	rid2pos;

    // key is slave_pos
    std::map<int,float> home;
    // key is slave_pos
    std::map<int,float> start_pos;

    iit::ecat::advr::Ft6ESC * leftFoot, * rightFoot;

    XDDP_pipe jsInXddp;
    XDDP_pipe keyInXddp;
    XDDP_pipe imuInXddp;

    double dt;

    advr::Spline_map spline_start2home;
};




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
