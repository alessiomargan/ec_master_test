/*

  Copyright (C) 2015 Italian Institute of Technology

  Developer:
      Alessio Margan (2015-, alessio.margan@iit.it)

*/
#include <experimental/filesystem>
#include <boost/algorithm/string.hpp>

#include <iit/advr/zmq_pub_th.h>

namespace fs = std::experimental::filesystem;

zmq::context_t zmq_ctx ( 1 );

#ifdef __XENO_PIPE__
static const std::string __pipe_prefix( "/proc/xenomai/registry/rtipc/xddp/" );
#else
static const std::string __pipe_prefix ( "/tmp/nrt_pipes" );
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

    std::cout << "[0MQ Pub] publisher bind to " << uri << std::endl;

}

Abs_Publisher::~Abs_Publisher() {
    delete _z;
}

int Abs_Publisher::open_pipe ( std::string pipe_name ) {

    //pipe = pipe_name;
    std::string pipe_path = pipe_name;

    std::cout << "[0MQ Pub] Opening xddp_socket " << pipe_path << std::endl;
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
    uint32_t fd_timeout_us = 999999;
    std::string robot_prefix ( "void@" );

    try {
        base_port = yaml_cfg["zmq_pub"]["base_port"].as<int>();
    } catch ( YAML::Exception ) {  }
    try {
        uri = yaml_cfg["zmq_pub"]["uri"].as<std::string>();
    } catch ( YAML::Exception ) { }
    try {
        fd_timeout_us = yaml_cfg["zmq_pub"]["fd_timeout_us"].as<uint32_t>();
    } catch ( YAML::Exception ) { }
    try {
        robot_prefix = yaml_cfg["ec_boards_base"]["robot_name"].as<std::string>() + "@";
    } catch ( YAML::Exception ) { }
    try {
        std::string application_mode = yaml_cfg["ec_boards_base"]["app_mode"].as<std::string>();
        run_mode = (application_mode == std::string("run_mode"));
        DPRINTF ("%s %s\n", __FUNCTION__, application_mode.c_str());
    } catch ( YAML::Exception &e ) {
        std::cout << e.what() << std::endl;
    }
    
    const std::string motor_prefix  ( "Motor_id_" );
    const std::string ft_prefix     ( "Ft_id_" );
    const std::string psens_prefix  ( "PressSens_id_" );
    const std::string imu_prefix    ( "Imu_id_" );
    const std::string hand_prefix   ( "Hand_id_" );
    const std::string heri_prefix   ( "Heri_id_" );
    const std::string pow_prefix    ( "PowF28M36Esc_" );

    
    ///////////////////////////////////////////////////////////////////////
    // DISCOVER in __pipe_prefix path
    ///////////////////////////////////////////////////////////////////////
    
    for( auto& p: fs::directory_iterator(__pipe_prefix) ) {
        
        std::string path = p.path().u8string();
        std::string filename = p.path().filename().u8string();
        //std::cout << path << '\n';
        std::vector<std::string> filename_tokens;
        int id;
        boost::split(filename_tokens, filename, boost::is_any_of("_"));
        id = std::atoi(filename_tokens.back().c_str());
        if ( ! id ) {
            // 0 ==> atoi error
            continue;
        } 
 
        zpub_factory(new SimplePublisher(uri+std::to_string(base_port+id), filename, fd_timeout_us), path, filename);           
    }

    if ( zmap.size() == 0 ) {
        std::cout << "[0MQ Pub] " << __pipe_prefix << " empty or invalid Id "<< std::endl;
    }
        
}

void ZMQ_Pub_thread::th_loop ( void * ) {

    tNow = iit::ecat::get_time_ns();

    if ( run_mode ) {
        for ( auto const& item : zmap ) {
            item.second->publish();
        }
    } else {
        sleep(1);
    } 
    

    dt = iit::ecat::get_time_ns()-tNow;
    loop_time ( dt );

}


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 