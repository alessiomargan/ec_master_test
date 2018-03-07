#include <cassert>
#include <csignal>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <exception>
#include <algorithm>
#include <iostream>

#include <iit/ecat/utils.h>
#include <iit/advr/thread_util.h>

#include <protobuf/ec_boards_base_input.pb.h>

#include <iit/advr/cxxopts.hpp>

#define THREADS_NUM 128

extern void main_common ( int *argcp, char *const **argvp, __sighandler_t sig_handler );
extern void set_main_sched_policy ( int );

static int main_loop = 1;

void shutdown ( int sig __attribute__ ( ( unused ) ) ) {
    main_loop = 0;
    DPRINTF ( "got signal .... Shutdown\n" );
}

pthread_barrier_t threads_barrier;

////////////////////////////////////////////////////
// 
////////////////////////////////////////////////////
class RT_thread : public Thread_hook {

    uint64_t                tNow, dt, prev;
    iit::ecat::stat_t       loop_time;
    int                     th_idx;
    std::string             th_name;
        
public:

    RT_thread(int idx) : th_idx(idx)
    {
        th_name = std::string("RT_thrd_") + std::to_string(th_idx);
        name = th_name.c_str();
        // non periodic
        period.period = {0,(idx+1)*50};

#ifdef __COBALT__
        schedpolicy = SCHED_FIFO;
#else
        schedpolicy = SCHED_FIFO;
#endif
        priority = sched_get_priority_max ( schedpolicy );
        stacksize = PTHREAD_STACK_MIN; //0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    }

    ~RT_thread() {
        
        print_zzz();
    }

    virtual void th_init ( void * ) {

        prev = 0;
        pthread_barrier_wait(&threads_barrier);
    }
    
    virtual void th_loop ( void * ) {
        
        tNow = iit::ecat::get_time_ns();
        if ( ! prev ) {
            prev = tNow;
            return;
        }
        
        dt = tNow - prev;
        loop_time(dt);
        prev = tNow;
        
        if ( (b_acc::count(loop_time) % 1000) == 0 ) {
        
            print_zzz();
        }
    }
    
    void print_zzz(void) {
        DPRINTF("thread %s\n", th_name.c_str());
        iit::ecat::print_stat(loop_time);
        DPRINTF("\n");
    }
};



////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main ( int argc, char * argv[] ) try {

    std::string th_name;
    std::map<std::string, Thread_hook*> threads;
    cxxopts::Options options(argv[0], " - wizardry setup");
    int threads_num = THREADS_NUM;
    
    try
    {
        options.add_options()
            ("n, threads_num", "threads_num", cxxopts::value<int>());
        options.parse(argc, argv);

        threads_num = options["threads_num"].as<int>();
    }
    catch (const cxxopts::OptionException& e)
    {
        std::cout << "error parsing options: " << e.what() << std::endl;
        exit(1);
    }
    
    int nr_cpu = sysconf(_SC_NPROCESSORS_ONLN); 
    
    sigset_t set;
    int sig;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
    
    main_common (&argc, (char*const**)&argv, 0 );

    pthread_barrier_init(&threads_barrier, NULL, threads_num  + 1 );

    for ( int idx=0; idx<threads_num; idx++ ) {
        th_name = std::string("RT_thrd_") + std::to_string(idx);
        std::cout << th_name << std::endl;
        threads[th_name] = new RT_thread(idx);
        threads[th_name]->create( true, (idx%nr_cpu) );
    }
    
    
    std::cout << "threads barrier ... wait 3 sec" << std::endl;
    sleep(3);
    pthread_barrier_wait(&threads_barrier);

    #ifdef __COBALT__
    // here I want to catch CTRL-C 
     __real_sigwait(&set, &sig);
#else
     sigwait(&set, &sig);  
#endif

    for ( auto const &t : threads) {
        t.second->stop();
        t.second->join();
        delete t.second;
    }
    
    std::cout << "Exit main" << std::endl;

    return 0;

} catch ( std::exception& e ) {

    std::cout << "Main catch .... " <<  e.what() << std::endl;

}


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
