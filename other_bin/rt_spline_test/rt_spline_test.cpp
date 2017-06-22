//#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <exception>
#include <iostream>
#include <cmath>
#include <random>

#include <iit/ecat/utils.h>
#include <iit/ecat/advr/pipes.h>
#include <iit/advr/thread_util.h>
#include <iit/advr/spline.h>
#include <iit/advr/trajectory.h>
#include <protobuf/vector2d.pb.h>

extern void main_common ( int *argcp, char *const **argvp, __sighandler_t sig_handler );
extern void set_main_sched_policy ( int );


std::random_device rd;     // only used once to initialise (seed) engine
std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
//std::uniform_int_distribution<int> uni(0,10); // guaranteed unbiased
std::uniform_real_distribution<double> uni(0,10);

////////////////////////////////////////////////////
// 
////////////////////////////////////////////////////
class RT_thread : public Thread_hook {

    uint64_t                tNow, dt;
    iit::ecat::stat_t       loop_time;
    XDDP_pipe               OutXddp;
    std::string             pipe_name;
    advr::Trj_ptr           trj;
    gazebo::msgs::Vector2d  vector;
    std::string             pbStr;    
    
public:

    RT_thread(std::string _pipe_name) : pipe_name(_pipe_name)
    {

        name = "RT_thread";
        // non periodic
        period.period = {0,250};

#ifdef __COBALT__
        schedpolicy = SCHED_FIFO;
#else
        schedpolicy = SCHED_OTHER;
#endif
        priority = sched_get_priority_max ( schedpolicy );
        stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    }

    ~RT_thread() {
    }

    virtual void th_init ( void * ) {

        OutXddp.init(pipe_name);
        
        std::vector<double> Ys =  std::initializer_list<double> { 0, M_PI/4, M_PI/2, M_PI };
        std::vector<double> Xs =  std::initializer_list<double> { 0, 1, 2, 3 };
        trj = std::make_shared<advr::Spline_trajectory> ( Xs, Ys );
    }
    
    virtual void th_loop ( void * ) {
    
        std::vector<double> Ys;
        std::vector<double> Xs;
    
        if ( trj->ended() ) {
            
            Xs = std::initializer_list<double> { 0, 1, 2, 3, 4, 5 };
            Ys = std::initializer_list<double> { 0, uni(rng), uni(rng), uni(rng), uni(rng), uni(rng) };
            trj = std::make_shared<advr::Spline_trajectory> ( Xs, Ys );
            //trj->set_points ( Xs ,Ys );
            
        }
        
        auto v = (*trj)();
        uint64_t sT;
        trj->get_start_time(sT);
        //DPRINTF(">> %f\n", v);
        //OutXddp.xddp_write( v );
        
        vector.set_x( (double)(iit::ecat::get_time_ns() - sT)/1000000000 );
        vector.set_y(v);
        
        ////////////////////////////////////////////////////////////////////
        // https://developers.google.com/protocol-buffers/docs/techniques
        
        vector.SerializeToString( &pbStr );
        uint32_t msg_size = pbStr.length();
        int nbytes = write ( OutXddp.get_fd(), ( void* ) &msg_size, sizeof( msg_size ) );
        nbytes = write ( OutXddp.get_fd(), ( void* ) pbStr.c_str() , pbStr.length() );
        
    }
};

////////////////////////////////////////////////////
// 
////////////////////////////////////////////////////
class UI_thread : public Thread_hook {

    iit::ecat::stat_t       loop_time;
    uint64_t                tNow, dt;
    int                     xddp_fd;
    std::string             pipe_name;
    
    gazebo::msgs::Vector2d  vector;
    uint8_t  msg_buff[1024];
    
public:

    UI_thread(std::string _pipe_name):pipe_name(_pipe_name) {

        name = "UI_thread";
        // periodic
        period.period = {0,100};

        schedpolicy = SCHED_OTHER;
        priority = sched_get_priority_max ( schedpolicy ) /2;
        stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    }

    ~UI_thread() {
        close(xddp_fd);
        iit::ecat::print_stat ( loop_time );
    }

    virtual void th_init ( void * ) {
        int retry = 10;
        std::string pipe ( pipe_prefix + pipe_name );
        while ( retry -- ) {
            xddp_fd = open ( pipe.c_str(), O_RDONLY |O_NONBLOCK );
            if ( xddp_fd <= 0 ) {
                std::cout << retry << ": " << pipe << std::endl;
                usleep(100000);
            } else {
                break;
            }
        }
        assert (xddp_fd > 0);
    }
    virtual void th_loop ( void * ) {
        
        uint32_t msg_size;
        
        int nbytes = read ( xddp_fd, ( void* ) &msg_size, sizeof ( msg_size ) );
        if ( nbytes > 0 ) {
            nbytes = read ( xddp_fd, ( void* ) msg_buff, msg_size );
            vector.ParseFromArray(msg_buff, msg_size);
            std::cout << vector.x() << " " << vector.y() << std::endl;
        }

        
    }
};


////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main ( int argc, char * const argv[] ) try {

    std::map<std::string, Thread_hook*> threads;
    
    threads["UI_thread"] = new UI_thread(std::string("spline_test"));
    threads["RT_thread"] = new RT_thread(std::string("spline_test"));
    
    sigset_t set;
    int sig;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
    
    main_common (&argc, &argv, 0 );
    
    threads["RT_thread"]->create(true);
    threads["UI_thread"]->create(false);

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
