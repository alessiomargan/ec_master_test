/*

  Copyright (C) 2015 Italian Institute of Technology

  Developer:
      Alessio Margan (2015-, alessio.margan@iit.it)

*/

#ifndef __ZMQ_PUBLISHER_H__
#define __ZMQ_PUBLISHER_H__

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <typeinfo>
#include <string>
#include <iostream>
#include <map>

#include <yaml-cpp/yaml.h>
//#include <jsoncpp/json/json.h>

#include <zmq.hpp>

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

#define PB_BUFF_SIZE    2048
/*
#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/ft6_esc.h>
#include <iit/ecat/advr/pressure_sensor_esc.h>
#include <iit/ecat/advr/imu_Vn_esc.h>
#include <iit/ecat/advr/power_board.h>
#include <iit/ecat/advr/power_coman_board.h>
#include <iit/ecat/advr/mc_hand_esc.h>
#include <iit/ecat/advr/heri_hand_esc.h>
#include <iit/ecat/advr/test_esc.h>
*/

#include <iit/advr/thread_util.h>
#include <iit/advr/ati_iface.h>

#include <protobuf/ecat_pdo.pb.h>



class Abs_Publisher;
typedef std::map<std::string, Abs_Publisher*>   PubMap_t;
typedef std::map<std::string, std::string>      jmap_t;


//extern zmq::context_t zmq_ctx;


///////////////////////////////////////////////////////////////////////
///
/// Abs_Publisher
///
///////////////////////////////////////////////////////////////////////

class Abs_Publisher {

public:
    Abs_Publisher ( std::string uri, std::string zkey );
    virtual ~Abs_Publisher();

    virtual int open_pipe ( std::string pipe_name );
    virtual int publish ( void ) = 0;
   
protected:

    int publish_msg( void );

    zmq::socket_t * _z;
    zmq::message_t  _msg_id;
    zmq::message_t  _msg;

    std::string uri;
    std::string zmsg_id;
    int xddp_sock;

};


// template<typename T>
// std::string json_serializer ( T &t ) {
//     Json::FastWriter writer;
//     Json::Value      root;
//     jmap_t           jmap;
//     t.to_map ( jmap );
//     for ( auto const& item : jmap ) {
//         root[item.first] = ::atof ( item.second.c_str() );
//     }
//     return std::string ( writer.write ( root ) );
// }


///////////////////////////////////////////////////////////////////////
///
/// Templete Publisher
///
///////////////////////////////////////////////////////////////////////
template<typename PubDataTypes>
class Publisher : public Abs_Publisher {

private:

    typedef PubDataTypes    pub_data_t;
    pub_data_t              pub_data;
    
public:    

    Publisher ( std::string uri, std::string zkey ) : Abs_Publisher ( uri, zkey ) { }
    virtual ~Publisher() {
        std::cout << "~" << typeid ( this ).name() << std::endl;
    }

    template<typename T>
    int read_pipe ( T &t ) {

        int nbytes = 0;
        int expected_bytes = sizeof ( t );
        fd_set rfds;
        struct timeval tv;

        FD_ZERO ( &rfds );
        FD_SET ( xddp_sock, &rfds );
        tv.tv_sec = 3;
        tv.tv_usec = 0;

        if ( select ( xddp_sock+1, &rfds, NULL, NULL, &tv ) > 0 ) {

            nbytes = read ( xddp_sock, ( void* ) &t, expected_bytes );
            if ( nbytes != expected_bytes ) {
                printf("[0MQ Pub] rx %d expected %d\n", nbytes, expected_bytes);
                return -1;
            }
        } else {
            return -ETIMEDOUT;
        }

        return nbytes;
    }

    virtual int publish ( void ) {
        
        int retval = read_pipe( pub_data );
        if ( retval <= 0 ) {
            std::cout << "[0MQ Pub] Error " << retval << " read from pipe " << zmsg_id << std::endl;
            return -1;
        }

        return publish ( pub_data );
    }

    int publish ( pub_data_t &pub_data ) {

        int msg_data_size;
        std::string sz_string;

        // prepare _msg_id
        _msg_id.rebuild ( zmsg_id.length() );
        memcpy ( ( void* ) _msg_id.data(),zmsg_id.data(), zmsg_id.length() );

        //////////////////////////////////////////////////////////
        // -- text format
        //msg_data_size = pub_data.sprint ( zbuffer,sizeof ( zbuffer ) );
        //////////////////////////////////////////////////////////
        // -- json format
        //sz_string = json_serializer ( pub_data );
        //////////////////////////////////////////////////////////
        // -- protobuf
        pub_data.pb_toString(&sz_string);

        //////////////////////////////////////////////////////////
        // prepare _msg
        msg_data_size = sz_string.length();
        _msg.rebuild ( msg_data_size );
        memcpy ( ( void* ) _msg.data(), sz_string.c_str(), msg_data_size );
        
        //printf("%s", (char*)_msg.data());
        return publish_msg();
    }

};


class SimplePublisher : public Abs_Publisher {

private:

    uint32_t                fd_timeout_us;
    uint32_t                pb_size;
    uint8_t                 pb_buf[PB_BUFF_SIZE];     
    std::string             pb_str;
    iit::advr::Ec_slave_pdo pb_msg;
    
public:    

    SimplePublisher ( std::string uri, std::string zkey, uint32_t timeout_us ) : Abs_Publisher ( uri, zkey ) {
    
        fd_timeout_us = timeout_us;
    }

    virtual ~SimplePublisher() {
        std::cout << "~" << typeid ( this ).name() << std::endl;
    }

    int read_pipe ( void ) {

        int nbytes = 0;
        fd_set rfds;
        struct timeval tv;
        
        FD_ZERO ( &rfds );
        FD_SET ( xddp_sock, &rfds );
        tv.tv_sec = 0;
        tv.tv_usec = fd_timeout_us;

        if ( select ( xddp_sock+1, &rfds, NULL, NULL, &tv ) > 0 ) {

            nbytes = read ( xddp_sock, (void*)&pb_size, sizeof ( pb_size ) );
            if ( nbytes <= 0 ) {
                return -ENODATA;
            }
            if ( nbytes > PB_BUFF_SIZE ) { 
                return -ENOBUFS;
            }
            
            nbytes = read ( xddp_sock, (void*)pb_buf, pb_size );
            //pb_msg.ParseFromArray(pb_buf, pb_size);
            //std::cout << pb_msg.DebugString() <<  std::endl;
            
        } else {
            return -ETIMEDOUT;
        }

        return nbytes;
    }

    int publish ( void ) {
        
        int retval = read_pipe();
        if ( retval <= 0 ) {
            std::cout << "[0MQ Pub] Error " << retval << " read from pipe " << zmsg_id << std::endl;
            return -1;
        }

        // prepare _msg_id just once
        _msg_id.rebuild ( zmsg_id.length() );
        memcpy ( ( void* ) _msg_id.data(),zmsg_id.data(), zmsg_id.length() );

        //////////////////////////////////////////////////////////
        // prepare _msg
        _msg.rebuild ( pb_size );
        memcpy ( ( void* ) _msg.data(), pb_buf, pb_size );
        
        return publish_msg();
    }


};


///////////////////////////////////////////////////////////////////////
///
/// ZMQ_Pub_thread
///
///////////////////////////////////////////////////////////////////////

class ZMQ_Pub_thread : public Thread_hook {

//     typedef Publisher<iit::ecat::advr::TestEscPdoTypes::pdo_rx>     TestPub;
//     typedef Publisher<iit::ecat::advr::ImuEscPdoTypes::pdo_rx>      ImuPub;
//     typedef Publisher<iit::ecat::advr::Ft6EscPdoTypes::pdo_rx>      FtPub;
//     typedef Publisher<iit::ecat::advr::McEscPdoTypes::pdo_rx>       McPub;
//     typedef Publisher<iit::ecat::advr::PowEscPdoTypes::pdo_rx>      PwPub;
//     typedef Publisher<iit::ecat::advr::PowCmnEscPdoTypes::pdo_rx>   PwCmnPub;
//     typedef Publisher<iit::ecat::advr::McHandEscPdoTypes::pdo_rx>   McHandPub;
//     typedef Publisher<iit::ecat::advr::HeriHandEscPdoTypes::pdo_rx> HeriHandPub;
//     typedef Publisher<iit::advr::ati_log_t> FtAtiPub;
//     template <int _Rows, int _Cols>
//     using PressSensPub = Publisher<typename iit::ecat::advr::PressSensEscPdoTypes<_Rows,_Cols>::pdo_rx>;

    iit::ecat::stat_t loop_time;
    uint64_t    tNow, dt;
    PubMap_t    zmap;
    
    YAML::Node  yaml_cfg;

public:

    ZMQ_Pub_thread( std::string config ) {

        name = "ZMQ_Pub_thread";
        // non periodic
        period.period = {0,1};

        schedpolicy = SCHED_OTHER;
        priority = sched_get_priority_max ( schedpolicy );
        stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
       
        std::ifstream fin ( config );
        if ( fin.fail() ) {
            DPRINTF ( "[0MQ Pub] Can not open %s\n", config.c_str() );
            assert ( 0 );
        }
        yaml_cfg = YAML::LoadFile ( config );
        
    }

    ~ZMQ_Pub_thread() {

        for ( auto const& item : zmap ) {
            delete item.second;
        }

        iit::ecat::print_stat ( loop_time );
    }

    virtual void th_init ( void * );
    virtual void th_loop ( void * );

private:

    void zpub_factory(Abs_Publisher * zpub, const std::string pipe_path, const std::string zkey ) {
        if ( zpub->open_pipe ( pipe_path ) == 0 ) {
            zmap[zkey] = zpub;
        } else {
            delete zpub;
        }
    }
    
    template <typename T>
    void zpub_factory(const std::string uri, const std::string pipe_path, const std::string zkey ) {
        Abs_Publisher * zpub = new T ( uri, zkey );
        zpub_factory(zpub, pipe_path, zkey);
    }


};



#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
