/*

  Copyright (C) 2015 Italian Institute of Technology

  Developer:
      Alessio Margan (2015-, alessio.margan@iit.it)

*/

#include <iit/advr/zmq_publisher.h>
#include <iit/advr/coman_robot_id.h>
#include <iit/advr/walkman_robot_id.h>

using namespace iit::ecat::advr;

zmq::context_t zmq_ctx ( 1 );

#ifdef __XENO_PIPE__
const std::string pipe_prefix ( "/proc/xenomai/registry/rtipc/xddp/" );
#else
const std::string pipe_prefix ( "/tmp/" );
#endif

///////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////

Abs_Publisher::Abs_Publisher ( std::string _uri ) : uri(_uri) {

    int opt_linger = 1;
#if ZMQ_VERSION_MAJOR == 2
    uint64_t opt_hwm = 1;
#else
    int opt_hwm = 1;
#endif

    _z = new zmq::socket_t ( zmq_ctx, ZMQ_PUB );
    _z->setsockopt ( ZMQ_LINGER, &opt_linger, sizeof ( opt_linger ) );
    _z->setsockopt ( ZMQ_SNDHWM, &opt_hwm, sizeof ( opt_hwm ) );
    _z->bind ( uri.c_str() );

    std::cout << "publisher bind to " << uri << std::endl;

}

Abs_Publisher::~Abs_Publisher() {
    _z->unbind(uri.c_str());
    delete _z;
}

int Abs_Publisher::open_pipe ( std::string pipe_name ) {

    pipe = pipe_name;
    std::string pipe_path = pipe_prefix + pipe_name;

    std::cout << "Opening xddp_socket " << pipe_path << std::endl;
    xddp_sock = open ( pipe_path.c_str(), O_RDONLY );

    if ( xddp_sock < 0 ) {
        printf ( "%s %s : %s\n", __FILE__, __FUNCTION__, strerror ( errno ) );
        return 1;
    }

    return 0;
}

int Abs_Publisher::publish_msg() {

    try {
        //printf("*** send %lu+%lu bytes\n", _msg_id.size() , _msg.size());
        _z->send ( _msg_id, ZMQ_SNDMORE );
        _z->send ( _msg );
        //printf("***\n");
    } catch ( zmq::error_t& e ) { // Interrupted system call
        printf ( ">>> zsend ... catch %s\n", e.what() );
        return -1;
    }

    return 0;
}


///////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////

void ZMQ_Pub_thread::th_init ( void* ) {

    Abs_Publisher * zpub;

    int base_port = 9000;
    std::string uri ( "tcp://*:" );

    std::string motor_prefix ( "Motor_id_" );
    std::string ft_prefix ( "Ft_id_" );
    
    ///////////////////////////////////////////////////////////////////////
    // COMAN
    ///////////////////////////////////////////////////////////////////////
    // coman::robot_mcs_ids
    for ( auto const& rid : coman::robot_mcs_ids ) {
        zpub = new McPub ( uri+std::to_string ( base_port+rid ) );
        if ( zpub->open_pipe ( motor_prefix+std::to_string ( rid ) ) == 0 ) {
            zmap[base_port+rid] = zpub;
        } else {
            delete zpub;
        }
    }
    ///////////////////////////////////////////////////////////////////////
    // coman::robot_fts_ids
    for ( auto const& rid : coman::robot_fts_ids ) {
        zpub = new FtPub ( uri+std::to_string ( base_port+rid ) );
        if ( zpub->open_pipe ( ft_prefix+std::to_string ( rid ) ) == 0 ) {
            zmap[base_port+rid] = zpub;
        } else {
            delete zpub;
        }
    }
    zpub = new PwCmnPub ( uri+std::to_string ( 10000 ) );
    if ( zpub->open_pipe ( "PowCmn_pos_1" ) == 0 ) {
        zmap[10000] = zpub;
    } else {
        delete zpub;
    }
#if 1
    ///////////////////////////////////////////////////////////////////////
    // WALKMAN
    ///////////////////////////////////////////////////////////////////////
    // walkman::robot_mcs_ids
    for ( auto const& rid : walkman::robot_mcs_ids ) {
        zpub = new McPub ( uri+std::to_string ( base_port+rid ) );
        if ( zpub->open_pipe ( motor_prefix+std::to_string ( rid ) ) == 0 ) {
            zmap[base_port+rid] = zpub;
        } else {
            delete zpub;
        }
    }
    ///////////////////////////////////////////////////////////////////////
    // walkman::robot_fts_ids
    for ( auto const& rid : walkman::robot_fts_ids ) {
        zpub = new FtPub ( uri+std::to_string ( base_port+rid ) );
        if ( zpub->open_pipe ( ft_prefix+std::to_string ( rid ) ) == 0 ) {
            zmap[base_port+rid] = zpub;
        } else {
            delete zpub;
        }
    }
#endif
    zpub = new PwPub ( uri+std::to_string ( 10001 ) );
    if ( zpub->open_pipe ( "PowWkm_pos_1" ) == 0 ) {
        zmap[10001] = zpub;
    } else {
        delete zpub;
    }

    ///////////////////////////////////////////////////////////////////////
    //
    ///////////////////////////////////////////////////////////////////////
#if 0
    base_port++;
    zpub = new McPub ( uri+std::to_string ( base_port ) );
    if ( zpub->open_pipe ( "Motor_id_1" ) == 0 ) {
        zmap[base_port] = zpub;
    } else {
        delete zpub;
    }
    base_port++;
    zpub = new McPub ( uri+std::to_string ( base_port ) );
    if ( zpub->open_pipe ( "Motor_id_21" ) == 0 ) {
        zmap[base_port] = zpub;
    } else {
        delete zpub;
    }

    base_port++;
    zpub = new TestPub ( uri+std::to_string ( base_port ) );
    if ( zpub->open_pipe ( "Test_pos_4" ) == 0 ) {
        zmap[base_port] = zpub;
    } else {
        delete zpub;
    }

    base_port++;
    zpub = new FtPub ( uri+std::to_string ( base_port ) );
    if ( zpub->open_pipe ( "Ft6ESC_0" ) == 0 ) {
        zmap[base_port] = zpub;
    } else {
        delete zpub;
    }

    base_port++;
    zpub = new McPub ( uri+std::to_string ( base_port ) );
    if ( zpub->open_pipe ( "HpESC_1000" ) == 0 ) {
        zmap[base_port] = zpub;
    } else {
        delete zpub;
    }

    base_port++;
    zpub = new PwPub ( uri+std::to_string ( base_port ) );
    if ( zpub->open_pipe ( "PowESC_pos2" ) == 0 ) {
        zmap[base_port] = zpub;
    } else {
        delete zpub;
    }
#endif
}

void ZMQ_Pub_thread::th_loop ( void * ) {

    tNow = iit::ecat::get_time_ns();

    for ( auto const& item : zmap ) {
        item.second->publish();
    }

    dt = iit::ecat::get_time_ns()-tNow;
    loop_time ( dt );

}


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; // kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
