/*

  Copyright (C) 2015 Italian Institute of Technology

  Developer:
      Alessio Margan (2015-, alessio.margan@iit.it)

*/
#include <experimental/filesystem>

#include <iit/advr/zmq_publisher.h>
#include <iit/advr/coman_robot_id.h>
#include <iit/advr/walkman_robot_id.h>
#include <iit/advr/centauro_robot_id.h>

using namespace iit::ecat::advr;

namespace fs = std::experimental::filesystem;

zmq::context_t zmq_ctx ( 1 );

#ifdef __XENO_PIPE__
static const std::string __pipe_prefix( "/proc/xenomai/registry/rtipc/xddp/" );
#else
static const std::string __pipe_prefix ( "/tmp/" );
#endif

///////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////

Abs_Publisher::Abs_Publisher ( std::string _uri, std::string _zkey ) : uri ( _uri ), zmsg_id( _zkey ) {

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
    //if ( xddp_sock > 0) { _z->unbind(uri.c_str()); }
    //_z->unbind(uri.c_str());
    delete _z;
}

int Abs_Publisher::open_pipe ( std::string pipe_name ) {

    //pipe = pipe_name;
    std::string pipe_path = pipe_name;

    std::cout << "Opening xddp_socket " << pipe_path << std::endl;
    xddp_sock = open ( pipe_path.c_str(), O_RDONLY );

    if ( xddp_sock < 0 ) {
        printf ( "xddp_sock %d - %s %s : %s\n", xddp_sock, __FILE__, __FUNCTION__, strerror ( errno ) );
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

void ZMQ_Pub_thread::th_init ( void * ) {

    Abs_Publisher * zpub;

    int base_port = 5000; 
    std::string uri ( "tcp://*:" );
    std::string robot_prefix ( "void@" );

    try {
        base_port = yaml_cfg["zmq"]["base_port"].as<int>();
    } catch ( YAML::Exception ) {  }
    try {
        uri = yaml_cfg["zmq"]["uri"].as<std::string>();
    } catch ( YAML::Exception ) { }
    try {
        robot_prefix = yaml_cfg["ec_boards_base"]["robot_name"].as<std::string>() + "@";
    } catch ( YAML::Exception ) { }
    
    const std::string motor_prefix  ( "Motor_id_" );
    const std::string ft_prefix     ( "Ft_id_" );
    const std::string psens_prefix  ( "PressSens_id_" );
    const std::string imu_prefix    ( "Imu_id_" );
    const std::string hand_prefix   ( "Hand_id_" );

    
    ///////////////////////////////////////////////////////////////////////
    // DISCOVER in __pipe_prefix path
    ///////////////////////////////////////////////////////////////////////
    
    for( auto& p: fs::directory_iterator(__pipe_prefix) ) {
        
        std::string path = p.path().u8string();
        std::string filename = p.path().filename().u8string();
        //std::cout << path << '\n';
        
        if ( path.find(motor_prefix) != std::string::npos ) {
            zpub_factory<McPub>(uri+std::to_string(base_port++), path, filename);
        } else if ( path.find(ft_prefix) != std::string::npos )  {
            zpub_factory<FtPub>(uri+std::to_string(base_port++), path, filename);
        } else if ( path.find(psens_prefix) != std::string::npos )  {
            zpub_factory<PressSensPub<10,5>>(uri+std::to_string(base_port++), path, filename);
        } else if ( path.find(imu_prefix) != std::string::npos )  {
            zpub_factory<ImuPub>(uri+std::to_string(base_port++), path, filename);
        }
        
    }
    
    ///////////////////////////////////////////////////////////////////////
    // COMAN
    ///////////////////////////////////////////////////////////////////////
#if 0
    base_port = 9000;
    for ( auto const& rid : coman::robot_mcs_ids ) { 
        zpub_factory<McPub>(rid, uri, motor_prefix, base_port);
    }
    for ( auto const& rid : coman::robot_fts_ids ) {
        zpub_factory<FtPub>(rid, uri, ft_prefix, base_port);
    }
    zpub = new PwCmnPub ( uri+std::to_string ( 10000 ) );
    if ( zpub->open_pipe ( "PowCmn_pos_1" ) == 0 ) {
        zmap[10000] = zpub;
    } else {
        delete zpub;
    }
#endif
    ///////////////////////////////////////////////////////////////////////
    // WALKMAN
    ///////////////////////////////////////////////////////////////////////
#if 0
    base_port = 9500;
    for ( auto const& rid : walkman::robot_mcs_ids ) {
        zpub_factory<McPub>(rid, uri, walkman+motor_prefix, base_port);
    }
    for ( auto const& rid : walkman::robot_fts_ids ) {
        zpub_factory<FtPub>(rid, uri, ft_prefix, base_port);
    }
    for ( auto const& rid : walkman::robot_foot_ids ) {
        zpub_factory<PressSensPub<10,5>>(rid, uri, walkman+foot_prefix, base_port);
    }
    zpub = new PwPub ( uri+std::to_string ( 10001 ) );
    if ( zpub->open_pipe ( "PowWkm_pos_1" ) == 0 ) {
        zmap[10001] = zpub;
    } else {
        delete zpub;
    }
#endif    
    ///////////////////////////////////////////////////////////////////////
    // CENTAURO
    ///////////////////////////////////////////////////////////////////////
#if 0
    base_port = 9600;
    for ( auto const &rid : centauro::robot_mcs_ids ) {
        zpub_factory<McPub>(rid, uri, centauro+motor_prefix, base_port);
    }
    for ( auto const& rid : std::initializer_list<int>{1} ) {
        zpub_factory<PressSensPub<8,3>>(rid, uri, norobot+skin_prefix, base_port);
    }
#endif
    ///////////////////////////////////////////////////////////////////////
    //
    ///////////////////////////////////////////////////////////////////////
#if 0
    base_port = 9800;
    for ( auto const& rid : std::initializer_list<int>{1,2,3} ) {
        zpub_factory<McHandPub>(rid, uri, hand_prefix, base_port);
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


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
