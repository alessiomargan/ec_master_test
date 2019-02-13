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
                    
}

inline void ZMQ_Rep_thread::th_loop( void * _) {

    std::string reply("Hello World !"); 
    zmq::message_t z_msg;
    
    if ( rep_sock->recv(&z_msg) ) {
        DPRINTF ( "[0MQ Rep] recv %s\n", (const char *)z_msg.data() );
        //ec_th_base->get_esc
        rep_sock->send((void*)reply.c_str(), reply.length());
    }
}

#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
