#ifndef __INPUT_DEVICE_H__
#define __INPUT_DEVICE_H__

#include <thread_util.h>

class Joy_Thread :
    public Thread_hook {
public:

    Joy_Thread ( const char * config_yaml ) {
        name = "EC_boards_joint_joy";
        period.period = {0,0};
        schedpolicy = SCHED_OTHER;
        priority = sched_get_priority_max ( schedpolicy );

    }

    virtual ~Joy_Thread() {

    }

    virtual void th_init ( void * ) {

    }

    virtual void th_loop ( void * ) {

    }

protected:

    int xddp_sock;

}

#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
