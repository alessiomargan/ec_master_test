////////////////////////////////////////////////////////////
// thread util
//
// First created:  summer 2009, by A.Margan
//
// Revisions:
//
////////////////////////////////////////////////////////////

#ifndef __THREAD_UTIL_H__
#define __THREAD_UTIL_H__

#include <pthread.h>
#include <linux/sched.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <bits/local_lim.h>
#include <exception>
#include <typeinfo>
#include <iostream>
#include <sys/timerfd.h>

#include <iit/ecat/utils.h>

#define handle_error_en(en, msg) \
    do { errno = en; perror(msg); exit(EXIT_FAILURE); } while (0)

typedef struct {
    struct timeval task_time;
    struct timeval period;
} task_period_t;


class Thread_hook;

typedef Thread_hook* Thread_hook_Ptr;

void * periodic_thread ( Thread_hook_Ptr );
void * non_periodic_thread ( Thread_hook_Ptr );
void * nrt_thread ( Thread_hook_Ptr );


inline void tsnorm ( struct timespec *ts ) {
    while ( ts->tv_nsec >= NSEC_PER_SEC ) {
        ts->tv_nsec -= NSEC_PER_SEC;
        ts->tv_sec++;
    }
}

extern pthread_barrier_t threads_barrier;

class Thread_hook {

public:

    virtual ~Thread_hook();

    void create ( int rt, int cpu_nr );
    void stop ( void );
    void join ( void );

    int is_non_periodic();

    virtual void th_init ( void * ) = 0;
    virtual void th_loop ( void * ) = 0;

    static void * th_helper ( void * );
    //static void * rt_th_helper ( void * );

protected:

    int _run_loop;

    const char *    name;
    task_period_t   period;

    pthread_t       thread_id;
    // pthread attribute
    int             schedpolicy;
    int             priority;
    int             stacksize;

    int             fd_timer;
    
    friend void * periodic_thread ( Thread_hook_Ptr );
    friend void * non_periodic_thread ( Thread_hook_Ptr );
    //friend void * nrt_thread ( Thread_hook_Ptr );

};

inline Thread_hook::~Thread_hook() {

    std::cout << "~" << typeid ( this ).name() << " " << std::string(name) <<std::endl;
}

inline int Thread_hook::is_non_periodic() {

    return ( period.period.tv_sec == 0 && period.period.tv_usec == 1 );
}

inline void * Thread_hook::th_helper ( void *kls )  {

        if ( ( ( Thread_hook_Ptr ) kls )->is_non_periodic() ) {
            return non_periodic_thread ( ( Thread_hook_Ptr ) kls );
        }
        return periodic_thread ( ( Thread_hook_Ptr ) kls );

}


inline void Thread_hook::stop() {
    _run_loop = 0;
}

inline void Thread_hook::join() {
    //pthread_cancel(thread_id);
    pthread_join ( thread_id, 0 );
}

inline void Thread_hook::create ( int rt=true, int cpu_nr=-1 ) {

    int ret;
    pthread_attr_t      attr;
    struct sched_param  schedparam;
    cpu_set_t           cpu_set;
    size_t 		dflt_stacksize;
    _run_loop = 1;

    CPU_ZERO ( &cpu_set );
    CPU_SET ( cpu_nr,&cpu_set );

    ret = pthread_attr_init(&attr);
    if (ret != 0) handle_error_en(ret, "pthread_attr_init");
    
    ret = pthread_attr_setinheritsched ( &attr, PTHREAD_EXPLICIT_SCHED );
    if (ret != 0) handle_error_en(ret, "pthread_attr_setinheritsched");

    ret = pthread_attr_setschedpolicy ( &attr, schedpolicy );
    if (ret != 0) handle_error_en(ret, "pthread_attr_setschedpolicy");

    schedparam.sched_priority = priority;
    ret = pthread_attr_setschedparam ( &attr, &schedparam );
    if (ret != 0) handle_error_en(ret, "pthread_attr_setschedparam");

    pthread_attr_getstacksize ( &attr, &dflt_stacksize );
    DPRINTF ( "default stack size %ld\n", dflt_stacksize );
    if ( stacksize > 0 ) {
        ret = pthread_attr_setstacksize ( &attr, stacksize );
        if (ret != 0) handle_error_en(ret, "pthread_attr_setstacksize");
    
    }
    
    ret = pthread_attr_setdetachstate ( &attr, PTHREAD_CREATE_JOINABLE );
    if (ret != 0) handle_error_en(ret, "pthread_attr_setdetachstate");

    if ( cpu_nr >= 0 ) {
        ret = pthread_attr_setaffinity_np ( &attr, sizeof ( cpu_set ), &cpu_set );
        if (ret != 0) handle_error_en(ret, "pthread_attr_setaffinity_np");
    }
    
    ret = pthread_create ( &thread_id, &attr, &th_helper, this );

    pthread_attr_destroy ( &attr );

    if ( ret != 0 ) {
        DPRINTF ( "%s %d %s", __FILE__, __LINE__, name );
        handle_error_en(ret, "pthread_create");
    }

}

#endif

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
