/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)
   
*/

/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_BOARDS_BASIC_H__
#define __EC_BOARDS_BASIC_H__

#include <linux/joystick.h>
#include <iit/advr/ec_boards_base.h>
/**
 */

typedef struct js_event 		input_t;

class Ec_Boards_basic : public Ec_Thread_Boards_base
{
public:
    
    Ec_Boards_basic(const char * config_yaml);
    virtual ~Ec_Boards_basic();

    template<class C>
    int user_input(C &user_cmd);
    virtual int user_loop(void);

private :
    
    virtual void init_preOP(void);
    virtual void init_OP(void);
    
     XDDP_pipe inXddp;
};




#endif
