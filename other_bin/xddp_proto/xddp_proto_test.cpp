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

#include <protobuf/ec_boards_base_input.pb.h>

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
    XDDP_pipe               outXddp, inXddp;
    std::string             pipe_name;
    
    iit::advr::Ec_board_base_input  pb_msg;
    uint8_t                         pb_buf[1024];    
    std::string                     pb_str;
    
public:

    RT_thread(std::string _pipe_name) : pipe_name(_pipe_name)
    {

        name = "RT_thread";
        // periodic
        period.period = {0,50000};

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

        outXddp.init(pipe_name+"_OUT");
        inXddp.init(pipe_name+"_IN");
        
    }
    
    virtual void th_loop ( void * ) {
    
        int msg_size, nbytes;
        static uint16_t cnt;
        cnt++;
        ///////////////////////////////////////////////////////////////////////
//         // read from NRT
//         nbytes = inXddp.xddp_read( msg_size );
//         if ( nbytes > 0 ) {
//             nbytes = inXddp.xddp_read ( pb_buf, msg_size );
//             pb_msg.ParseFromArray(pb_buf, msg_size);
//             std::cout << pb_msg.type() << std::endl;
//         }
        
        ///////////////////////////////////////////////////////////////////////
        // write to NRT
        pb_msg.set_type(iit::advr::Ec_board_base_input_Type_SET_GAINS);
        pb_msg.mutable_gains()->set_type(iit::advr::Gains_Type_POSITION);
        pb_msg.mutable_gains()->set_kp(cnt);
        pb_msg.mutable_gains()->set_ki(cnt);
        pb_msg.mutable_gains()->set_kd(cnt);
        pb_msg.SerializeToString( &pb_str);
        msg_size = pb_str.length();
        nbytes  = outXddp.xddp_write ( ( void* )&msg_size, sizeof( msg_size ) );
        nbytes += outXddp.xddp_write ( ( void* )pb_str.c_str(), pb_str.length() );
        
        DPRINTF("[RT] write %d bytes to NRT %s%s\n", nbytes, pipe_name.c_str(), "_OUT");
    }
};

////////////////////////////////////////////////////
// 
////////////////////////////////////////////////////
class UI_thread : public Thread_hook {

    iit::ecat::stat_t       loop_time;
    uint64_t                tNow, dt;
    int                     inXddp_fd, outXddp_fd;
    std::string             pipe_name;
    
    iit::advr::Ec_board_base_input  pb_msg;
    uint8_t                         pb_buf[1024];    
    std::string                     pb_str;
    
public:

    UI_thread(std::string _pipe_name):pipe_name(_pipe_name) {

        name = "UI_thread";
        // non periodic
        period.period = {0,1};

        schedpolicy = SCHED_OTHER;
        priority = sched_get_priority_max ( schedpolicy ) /2;
        stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    }

    ~UI_thread() {
        close(inXddp_fd);
        iit::ecat::print_stat ( loop_time );
    }

    virtual void th_init ( void * ) {
        
        int retry = 10;
        std::string pipe ( pipe_prefix + pipe_name +"_OUT");
        while ( retry -- ) {
            inXddp_fd = open ( pipe.c_str(), O_RDONLY |O_NONBLOCK );
            if ( inXddp_fd <= 0 ) {
                std::cout << retry << ": " << pipe << std::endl;
                usleep(10000);
            } else {
                break;
            }
        }
        assert (inXddp_fd > 0);
    }
    virtual void th_loop ( void * ) {
        
        uint32_t msg_size;

        ///////////////////////////////////////////////////////////////////////
        // read from RT

        int nbytes = read ( inXddp_fd, ( void* ) &msg_size, sizeof ( msg_size ) );
        if ( nbytes > 0 ) {
            nbytes = read ( inXddp_fd, ( void* ) pb_buf, msg_size );
            pb_msg.ParseFromArray(pb_buf, msg_size);
            std::cout << pb_msg.DebugString() <<  std::endl;
        }

        
    }
};


////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main ( int argc, char * const argv[] ) try {

    std::map<std::string, Thread_hook*> threads;
    
    sigset_t set;
    int sig;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
    
    main_common (&argc, &argv, 0 );

    threads["UI_thread"] = new UI_thread(std::string("xddp_proto"));
    threads["RT_thread"] = new RT_thread(std::string("xddp_proto"));
    
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
