/*

  Copyright (C) 2015 Italian Institute of Technology

  Developer:
      Alessio Margan (2019-, alessio.margan@iit.it)

*/

#ifndef __ZMQ_REP_H__
#define __ZMQ_REP_H__

#include <zmq.hpp>

#include <yaml-cpp/yaml.h>
#include <iit/advr/thread_util.h>
#include <protobuf/ecat_pdo.pb.h>

#define PB_BUFF_SIZE    2048


extern zmq::context_t zmq_ctx;

///////////////////////////////////////////////////////////////////////
///
/// REPL_loop_thread
///
///////////////////////////////////////////////////////////////////////

class ZMQ_Rep_thread : public Thread_hook {

public:

    ZMQ_Rep_thread( std::string config, void *_ec_th_base );
    ~ZMQ_Rep_thread() {
        delete rep_sock;
    }

    virtual void th_init ( void * );
    virtual void th_loop ( void * );

private:

    YAML::Node              yaml_cfg;
    zmq::socket_t *         rep_sock;
    Ec_Thread_Boards_base * ec_th_base;
    int repl_in_fd, repl_out_fd;
};


ZMQ_Rep_thread::ZMQ_Rep_thread( std::string config, void *_ec_th_base ) {

    name = "ZMQ_Rep_thread";
    // non periodic
    period.period = {0,1};

    schedpolicy = SCHED_OTHER;
    priority = sched_get_priority_max ( schedpolicy );
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
    
    std::ifstream fin ( config );
    if ( fin.fail() ) {
        DPRINTF ( "[0MQ Rep] Can not open %s\n", config.c_str() );
        assert ( 0 );
    }
    yaml_cfg = YAML::LoadFile ( config );
    
    ec_th_base = static_cast<Ec_Thread_Boards_base*>(_ec_th_base);
}

inline void ZMQ_Rep_thread::th_init( void * _) {
    
    int recv_timeout_ms = 500;
    const std::string uri("tcp://*:5555");
    rep_sock = new zmq::socket_t(zmq_ctx, ZMQ_REP);
    rep_sock->setsockopt ( ZMQ_RCVTIMEO, &recv_timeout_ms, sizeof ( recv_timeout_ms ) );
    rep_sock->bind( uri );
    DPRINTF ( "[0MQ Rep] bind to %s\n", uri.c_str() );
    //rep_sock->connect( uri );
    //DPRINTF ( "[0MQ Rep] connect to %s\n", uri.c_str() );
    
    std::string repl_in_path("/tmp/nrt_pipes/repl_in");
    std::cout << "[0MQ Rep] Opening xddp " << repl_in_path << std::endl;
    repl_in_fd = open ( repl_in_path.c_str(), O_WRONLY );
    if ( repl_in_fd < 0 ) {
        printf ( "repl_in_fd %d - %s %s : %s\n", repl_in_fd, __FILE__, __FUNCTION__, strerror ( errno ) );
    }

    std::string repl_out_path("/tmp/nrt_pipes/repl_out");
    std::cout << "[0MQ Rep] Opening xddp " << repl_out_path << std::endl;
    repl_out_fd = open ( repl_out_path.c_str(), O_RDONLY );
    if ( repl_out_fd < 0 ) {
        printf ( "repl_out_fd %d - %s %s : %s\n", repl_out_fd, __FILE__, __FUNCTION__, strerror ( errno ) );
    }
                    
}

inline void ZMQ_Rep_thread::th_loop( void * _) {

    uint32_t reply_size, msg_size, nbytes;
    std::string reply("Hello World !"); 
    zmq::message_t z_msg;
    
    if ( rep_sock->recv(&z_msg) ) {
        msg_size = z_msg.size();
        DPRINTF ( "[0MQ Rep] recv %d bytes\n", msg_size );
        
        // 
        write( repl_in_fd, (void*)&msg_size, sizeof(msg_size) );
        write( repl_in_fd, (void*)z_msg.data(), msg_size );
        
        //
        nbytes = read( repl_out_fd, (void*)&reply_size, sizeof(reply_size) );
        if ( nbytes > 0 ) {
            reply.resize(reply_size);
            nbytes = read( repl_out_fd, (void*)reply.c_str(), reply_size );
        }
        
        rep_sock->send((void*)reply.c_str(), reply.length());
    }
}

#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
