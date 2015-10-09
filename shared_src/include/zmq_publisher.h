/*

   Copyright (C) 2015 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)
   
*/

#ifndef __ZMQ_PUBLISHER_H__
#define __ZMQ_PUBLISHER_H__

#include <fcntl.h>
#include <unistd.h>

#include <typeinfo>
#include <string>
#include <iostream>
#include <map>

#include <jsoncpp/json/json.h>
#include <zmq.hpp>
//#include <zmq.h>
//#include <zmq_utils.h>

#ifndef ZMQ_DONTWAIT
    #define ZMQ_DONTWAIT ZMQ_NOBLOCK
#endif
#if ZMQ_VERSION_MAJOR == 2
    #define zmq_msg_send(msg,sock,opt) zmq_send (sock, msg, opt)
    #define zmq_msg_recv(msg,sock,opt) zmq_recv (sock, msg, opt)
    #define zmq_ctx_destroy(context) zmq_term(context)
    #define ZMQ_POLL_MSEC 1000 // zmq_poll is usec
    #define ZMQ_SNDHWM ZMQ_HWM
    #define ZMQ_RCVHWM ZMQ_HWM
#else
    #define ZMQ_POLL_MSEC 1 // zmq_poll is msec
#endif

#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/ft6_esc.h>
#include <iit/ecat/advr/power_board.h>
#include <iit/ecat/advr/test_esc.h>

namespace iit {
namespace ecat {
namespace advr {


class Abs_Publisher;
typedef std::map<int, Abs_Publisher*>  PubMap_t;
typedef std::map<std::string, std::string> jmap_t;

#if __XENO__
    static const std::string pipe_prefix("/proc/xenomai/registry/rtipc/xddp/");
#else
    static const std::string pipe_prefix("/tmp/");
#endif

zmq::context_t zmq_ctx(1);


///////////////////////////////////////////////////////////////////////
///
/// Abs_Publisher
///
///////////////////////////////////////////////////////////////////////

class Abs_Publisher {

public:
    
    Abs_Publisher(std::string uri);
    virtual ~Abs_Publisher();
    
    virtual int open_pipe(std::string pipe_name);
    virtual int publish(void) = 0;

protected:

    int publish_msg();
    
protected:
    
    zmq::socket_t * _z;
    zmq::message_t  _msg_id;
    zmq::message_t  _msg;
    
    std::string pipe;
    int xddp_sock;
    
    char zbuffer[8192];
};


inline Abs_Publisher::Abs_Publisher(std::string uri) {
       
    int opt_linger = 1;
#if ZMQ_VERSION_MAJOR == 2
    uint64_t opt_hwm = 1;
#else
    int opt_hwm = 1;
#endif
        
    _z = new zmq::socket_t(zmq_ctx, ZMQ_PUB);
    _z->setsockopt(ZMQ_LINGER, &opt_linger, sizeof(opt_linger));
    _z->setsockopt(ZMQ_SNDHWM, &opt_hwm, sizeof(opt_hwm));
    _z->bind(uri.c_str());
    
    printf ("publisher bind to %s\n",uri.c_str());

}

inline Abs_Publisher::~Abs_Publisher() {
    delete _z;
}

inline int Abs_Publisher::open_pipe(std::string pipe_name) {
    
    pipe = pipe_name;
    std::string pipe_path = pipe_prefix + pipe_name;

    printf ("Opening xddp_socket %s\n", pipe_path.c_str());
    xddp_sock = open(pipe_path.c_str(), O_RDONLY);

    if (xddp_sock < 0) {
        printf ("%s %s : %s\n", __FILE__, __FUNCTION__, strerror (errno));
        return 1;
    }
    
    return 0;
}

inline int Abs_Publisher::publish_msg() {

    try { 
        //printf("*** send %lu+%lu bytes\n", _msg_id.size() , _msg.size());
        _z->send(_msg_id, ZMQ_SNDMORE);
        _z->send(_msg);
        //printf("***\n");
    } catch (zmq::error_t& e) { // Interrupted system call
        printf(">>> zsend ... catch %s\n", e.what());
        return 1;
    }
    
    return 0;
}


///////////////////////////////////////////////////////////////////////
///
/// Publisher
///
///////////////////////////////////////////////////////////////////////

template<class PubDataTypes>
class Publisher : public Abs_Publisher {

public:
    
    typedef PubDataTypes    pub_data_t;
    
    Publisher(std::string uri) : Abs_Publisher(uri) { }
    virtual ~Publisher() { std::cout << "~" << typeid(this).name() << std::endl; }
    
    virtual int publish(void);

protected:
    
    pub_data_t pub_data;

};


#define TEMPL template<class PubDataTypes>
#define CLASS Publisher<PubDataTypes>
#define SIGNATURE(type) TEMPL inline type CLASS

SIGNATURE(int)::publish(void) {
    
    int nbytes, msg_data_size;
    int expected_bytes = sizeof(pub_data_t);

    nbytes = read(xddp_sock, (void*)&pub_data, expected_bytes);

    if (nbytes != expected_bytes) {
        //printf("zmq rx %d expected %d\n", nbytes, expected_bytes);
        return 1;
    }
    
    // prepare _msg_id
    _msg_id.rebuild(pipe.length());
    memcpy((void*)_msg_id.data(),pipe.data(), pipe.length());
    // prepare _msg
    //////////////////////////////////////////////////////////
    // -- text format 
    msg_data_size = pub_data.sprint(zbuffer,sizeof(zbuffer));
    //////////////////////////////////////////////////////////
    // -- binary format
    
    //////////////////////////////////////////////////////////
    // -- json format
    static Json::FastWriter writer;
    static Json::Value      root;
    static jmap_t           jmap;
    
    jmap.clear();
    pub_data.to_map(jmap);
    
    for (jmap_t::iterator it = jmap.begin(); it != jmap.end(); it++) {
            root[it->first] = ::atof(it->second.c_str()); // std:: it->second; 
    }
    std::string json_string(writer.write(root));
    msg_data_size = json_string.length();
    memcpy((void*)zbuffer, json_string.c_str(), msg_data_size);
    
    // re-build msg
    _msg.rebuild(msg_data_size);
    memcpy((void*)_msg.data(), zbuffer, msg_data_size);
    
    //printf("%s", (char*)_msg.data());
    
    return publish_msg();
    
}

#undef SIGNATURE
#undef CLASS
#undef TEMPL

///////////////////////////////////////////////////////////////////////
///
/// Abs_Publisher
///
///////////////////////////////////////////////////////////////////////

class ZMQ_Pub_thread : public Thread_hook {
    
    typedef Publisher<TestEscPdoTypes::pdo_rx> TestPub;
    typedef Publisher<Ft6EscPdoTypes::pdo_rx> FtPub;
    typedef Publisher<McEscPdoTypes::pdo_rx> McPub;
    typedef Publisher<PowEscPdoTypes::pdo_rx> PwPub;
    
    iit::ecat::stat_t loop_time;
    uint64_t	tNow, dt;
    PubMap_t    zmap;
	
public:

    ZMQ_Pub_thread() {

        name = "ZMQ_Pub_thread";
        period.period = {0,100}; 
        
        schedpolicy = SCHED_OTHER;
        priority = sched_get_priority_max(schedpolicy)/2;
        stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
	
    }

    ~ZMQ_Pub_thread() {

	for ( auto const& item : zmap ) { delete item.second; }
    }

    virtual void th_init(void *) { 

	Abs_Publisher * zpub;

	int base_port = 9000;
	std::string uri("tcp://*:");
	
	base_port++;
	zpub = new McPub(uri+std::to_string(base_port));
	if ( zpub->open_pipe("LpESC_1") == 0 ) { zmap[base_port] = zpub; }
	else { delete zpub; }

#if 0
	base_port++;
	zpub = new TestPub(uri+std::to_string(base_port)); 
	if ( zpub->open_pipe("ESC_test_pos1") == 0 ) { zmap[base_port] = zpub; }
	else { delete zpub; }
	
	base_port++;
	zpub = new FtPub(uri+std::to_string(base_port));
	if ( zpub->open_pipe("Ft6ESC_0") == 0 ) { zmap[base_port] = zpub; }
	else { delete zpub; }
	
	base_port++;
	zpub = new McPub(uri+std::to_string(base_port));
	if ( zpub->open_pipe("HpESC_1000") == 0 ) { zmap[base_port] = zpub; }
	else { delete zpub; }
	
	base_port++;
	zpub = new PwPub(uri+std::to_string(base_port));
	if ( zpub->open_pipe("PowESC_pos2") == 0 ) { zmap[base_port] = zpub; }
	else { delete zpub; }
#endif
    }

    virtual void th_loop(void *) { 

        tNow = get_time_ns();

	for ( auto const& item : zmap ) { item.second->publish(); }

	dt = get_time_ns()-tNow;
        loop_time(dt);

    }

};


}
}
}

#endif
