/*

  Copyright (C) 2015 Italian Institute of Technology

  Developer:
      Alessio Margan (2019-, alessio.margan@iit.it)

*/

#ifndef __ZMQ_REP_H__
#define __ZMQ_REP_H__

#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <yaml-cpp/yaml.h>

#include <iit/advr/thread_util.h>
#include <protobuf/ecat_pdo.pb.h>
#include <protobuf/repl_cmd.pb.h>
#include <iit/advr/ec_boards_base.h>

#define PB_BUFF_SIZE    2048


extern zmq::context_t zmq_ctx;

///////////////////////////////////////////////////////////////////////
///
/// REPL_loop_thread
///
///////////////////////////////////////////////////////////////////////

class ZMQ_Rep_thread : public Thread_hook {

public:

    //ZMQ_Rep_thread( std::string config, void *_ec_th_base );
    ZMQ_Rep_thread( std::string _config, void *_threads );
    ~ZMQ_Rep_thread() {
        delete rep_sock;
    }

    virtual void th_init ( void * );
    virtual void th_loop ( void * );

private:

    std::string                 config;
    YAML::Node                  yaml_cfg;
    zmq::socket_t *             rep_sock;
    std::vector<zmq::message_t> multi_zmsg;
    
    std::string uri, pipe_prefix, repl_in_path, repl_out_path;
    
    ThreadsMap * threads;
    int repl_in_fd, repl_out_fd;

    void open_repl_pipes(void);
    void close_repl_pipes(void);
    
    // 
    void stop_master(iit::advr::Cmd_reply &);
    void start_master(iit::advr::Cmd_reply &);
    void get_slaves_descr(iit::advr::Cmd_reply &);

};


ZMQ_Rep_thread::ZMQ_Rep_thread( std::string _config, void *_threads ):config(_config) {

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
    
    threads = static_cast<ThreadsMap*>(_threads);
}

inline void ZMQ_Rep_thread::open_repl_pipes(void) {

    repl_in_path = std::string(pipe_prefix+"repl_in");
    std::cout << "[0MQ Rep] Opening xddp " << repl_in_path << std::endl;
    repl_in_fd = open ( repl_in_path.c_str(), O_WRONLY );
    if ( repl_in_fd < 0 ) {
        printf ( "repl_in_fd %d - %s %s : %s\n", repl_in_fd, __FILE__, __FUNCTION__, strerror ( errno ) );
    }

    repl_out_path = std::string(pipe_prefix+"repl_out");
    std::cout << "[0MQ Rep] Opening xddp " << repl_out_path << std::endl;
    repl_out_fd = open ( repl_out_path.c_str(), O_RDONLY );
    if ( repl_out_fd < 0 ) {
        printf ( "repl_out_fd %d - %s %s : %s\n", repl_out_fd, __FILE__, __FUNCTION__, strerror ( errno ) );
    }
    
}

inline void ZMQ_Rep_thread::close_repl_pipes(void) {

        close(repl_in_fd);
        close(repl_out_fd);
}

inline void ZMQ_Rep_thread::th_init( void * _) {
    
    // default values
    int recv_timeout_ms = 500;
    uri = std::string("tcp://*:5555");
    pipe_prefix = std::string("/proc/xenomai/registry/rtipc/xddp/");

    try {
        const YAML::Node zmq_rep = yaml_cfg["zmq_rep"];
        recv_timeout_ms = zmq_rep["zmq_rcvtimeo_ms"].as<int>();
        uri = zmq_rep["uri"].as<std::string>();
        pipe_prefix = zmq_rep["pipe_prefix"].as<std::string>();
        
    } catch (YAML::Exception &e ) {
        std::cout << "[0MQ Rep] " << e.what() << std::endl;
    }
    
    rep_sock = new zmq::socket_t(zmq_ctx, ZMQ_REP);
    rep_sock->setsockopt ( ZMQ_RCVTIMEO, &recv_timeout_ms, sizeof ( recv_timeout_ms ) );
    rep_sock->bind( uri );
    DPRINTF ( "[0MQ Rep] bind to %s, zmq_rcvtimeo_ms %d\n", uri.c_str(), recv_timeout_ms );
    //rep_sock->connect( uri );
    //DPRINTF ( "[0MQ Rep] connect to %s\n", uri.c_str() );
    
    open_repl_pipes();
}

inline void ZMQ_Rep_thread::th_loop( void * _) {

    uint32_t reply_size, msg_size, nbytes;
    std::string             reply_str; 
    std::string             msg_header;
    zmq::message_t          zmsg;
    zmq::multipart_t        multi_zmsg;
    iit::advr::Cmd_reply    pb_reply;
    iit::advr::Repl_cmd     pb_msg;
        
    int msg_parts = 0;
        
    // receive a multipart message : header + protobuf content
#if 0
    do {
        
        if ( ! rep_sock->recv(&zmsg) ) {
            continue;
        }
        multi_zmsg.addmem(zmsg.data(),zmsg.size());
        msg_parts++;
        msg_size = zmsg.size();
        DPRINTF ( "[0MQ Rep] {%d}recv %d bytes\n", msg_parts, msg_size );
        
    } while (zmsg.more());
#else
    multi_zmsg.recv(*rep_sock);
#endif
    
    if ( multi_zmsg.empty() ) {
        return;
    }
    
    // extract header part as string
    msg_header = multi_zmsg.popstr();
    DPRINTF ( "[0MQ Rep] hdr %s \n", msg_header.c_str() );
    
    /**************************************************************************
     * 
     *************************************************************************/
    if ( msg_header == "ESC_CMD" ) {
    
        zmsg = multi_zmsg.pop();
        msg_size = zmsg.size();
        nbytes = write( repl_in_fd, (void*)&msg_size, sizeof(msg_size) );
        if ( nbytes > 0 ) {
            nbytes = write( repl_in_fd, (void*)zmsg.data(), msg_size );
        } else {
            DPRINTF ( "[0MQ Rep] FAIL write to %s \n", repl_in_path.c_str() );
        }
        //
        nbytes = read( repl_out_fd, (void*)&reply_size, sizeof(reply_size) );
        if ( nbytes > 0 ) {
            reply_str.resize(reply_size);
            nbytes = read( repl_out_fd, (void*)reply_str.c_str(), reply_size );
        } else {
            DPRINTF ( "[0MQ Rep] FAIL read to %s \n", repl_out_path.c_str() );
        }
        
    }
    
    /**************************************************************************
     * 
     *************************************************************************/
    if ( msg_header == "ECAT_MASTER_CMD" ) {

        zmsg = multi_zmsg.pop();
        msg_size = zmsg.size();
        pb_msg.ParseFromArray(zmsg.data(), msg_size);
        DPRINTF("[0MQ Rep] pb_msg\n%s\n", pb_msg.DebugString().c_str());
        pb_reply.set_cmd_type(pb_msg.type());
        pb_reply.set_type(iit::advr::Cmd_reply::NACK);
        pb_reply.set_msg("NOt handled");
        
        switch ( pb_msg.type() ) {
            case iit::advr::CmdType::ECAT_MASTER_CMD :
                switch ( pb_msg.mutable_ecat_master_cmd()->type() ) {
                    case iit::advr::Ecat_Master_cmd::STOP_MASTER :
                        stop_master(pb_reply);
                        break;
                    
                    case iit::advr::Ecat_Master_cmd::START_MASTER :
                        start_master(pb_reply);
                        break;
                    
                    case iit::advr::Ecat_Master_cmd::GET_SLAVES_DESCR :
                        get_slaves_descr(pb_reply);
                        break;
                        
                    default :
                        break;
                }
                break; // case iit::advr::CmdType::ECAT_MASTER_CMD
                
            default :
                break;
        }

        DPRINTF("[0MQ Rep] pb_reply\n%s\n", pb_reply.DebugString().c_str());
        pb_reply.SerializeToString(&reply_str);
        reply_size = reply_str.length();
    
    }

    /**************************************************************************
     * 
     *************************************************************************/
    multi_zmsg.clear();

    // TODO reply with multipart_t mesg
    
    rep_sock->send((void*)reply_str.c_str(), reply_size);

}


inline void ZMQ_Rep_thread::stop_master(iit::advr::Cmd_reply &_pb_reply) {

    try {
        close_repl_pipes();
        for ( auto const& item : *threads ) {
            item.second->stop();
            item.second->join();
            delete item.second;
        _pb_reply.set_type(iit::advr::Cmd_reply::ACK);
        _pb_reply.set_msg("Success");
        }
    } catch (std::exception &e) {
        _pb_reply.set_type(iit::advr::Cmd_reply::NACK);
        _pb_reply.set_msg(e.what());                
    }
    return;
}

inline void ZMQ_Rep_thread::start_master(iit::advr::Cmd_reply &_pb_reply) {

    try {
        ((*threads)["boards_basic"]) = new Ec_Boards_basic (config);
        ((*threads)["ZMQ_pub"]) = new ZMQ_Pub_thread(config);
        pthread_barrier_init(&threads_barrier, NULL, 1+1 );
        ((*threads)["boards_basic"])->create(true, 2);
        pthread_barrier_wait(&threads_barrier);
        ((*threads)["ZMQ_pub"])->create(false, 4);
        open_repl_pipes();
        _pb_reply.set_type(iit::advr::Cmd_reply::ACK);
        _pb_reply.set_msg("Success");
        
    } catch(std::exception &e) {
        _pb_reply.set_type(iit::advr::Cmd_reply::NACK);
        _pb_reply.set_msg(e.what());        
    }
    return; 
}

inline void ZMQ_Rep_thread::get_slaves_descr(iit::advr::Cmd_reply &_pb_reply) {

    try {
        std::string result;
        Ec_Thread_Boards_base * bb = dynamic_cast<Ec_Thread_Boards_base *>((*threads)["boards_basic"]);
        bb->get_slaves_descr(result);
        _pb_reply.set_type(iit::advr::Cmd_reply::ACK);
        _pb_reply.set_msg(result);
        
    } catch(std::exception &e) {
        _pb_reply.set_type(iit::advr::Cmd_reply::NACK);
        _pb_reply.set_msg(e.what()
);        
    }
    return; 

    
}

#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
