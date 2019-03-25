#include <iit/advr/ati_iface.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
//#include <sys/ioctl.h>
//#include <net/if.h>
#ifdef __COBALT__
    #include <asm/ioctl.h>
    #include <rtdm/rtdm.h>
    //#include <rtnet.h>
    #define RTIOC_TYPE_NETWORK      RTDM_CLASS_NETWORK
    #define RTNET_RTIOC_TIMEOUT     _IOW(RTIOC_TYPE_NETWORK,  0x11, int64_t)
#endif

#include <cstring>
#include <fstream>


using namespace iit::advr;

void Ati_Sens::config ( bool _run_thread,
                     float _countsPerUnit,
                     const std::string& _ip )
{

    run = _run_thread;
    countsPerUnit = _countsPerUnit;
    ati_iface_ip = _ip;

    // create udp socket
    if ( ( udp_sock = socket ( AF_INET, SOCK_DGRAM, IPPROTO_UDP ) ) < 0 ) {
        perror ( "[Ati_Sens] socket cannot be created" );
        assert ( 0 );
    }

    // set socket timeout option
#ifdef NOTUSE__COBALT__
    // This socket control option is used to specify the time-out on the socket before it returns.
    // It is used typically to wait for data on a Read.
    // The time-out specifies the amount of time the function will wait for data before it returns.
    int64_t timeout_ns = 250000000;
    if ( ioctl ( udp_sock, RTNET_RTIOC_TIMEOUT, &timeout_ns ) < 0 )
        DPRINTF ( "ioctl RTNET_RTIOC_TIMEOUT failed\n" );
#else
    struct timeval timeout;
    timeout.tv_sec =  0;
    timeout.tv_usec = 250000;
    if ( setsockopt ( udp_sock, SOL_SOCKET, SO_RCVTIMEO, ( char * ) &timeout,
                      sizeof ( timeout ) ) < 0 )
        DPRINTF ( "[Ati_Sens] setsockopt SO_RCVTIMEO failed\n" );

    if ( setsockopt ( udp_sock, SOL_SOCKET, SO_SNDTIMEO, ( char * ) &timeout,
                      sizeof ( timeout ) ) < 0 )
        DPRINTF ( "[Ati_Sens] setsockopt SO_SNDTIMEO failed\n" );
#endif

    //
    memset ( ( void* ) &dest_addr, 0, sizeof ( dest_addr ) );
    dest_addr.sin_family        = AF_INET;
    dest_addr.sin_port          = htons ( 49152 );
    dest_addr.sin_addr.s_addr   = inet_addr ( ATI_IFACE_IP );

    if ( connect ( udp_sock, ( struct sockaddr * ) &dest_addr, sizeof ( dest_addr ) ) < 0 ) {
        perror ( "cannot connect to  ip/port" );
        close ( udp_sock );
        assert ( 0 );
    }

    ///////////////////////////////////////////////////////////////////////////

    ati_log.set_capacity ( LOG_SIZE );
    start_time = 0;
    if ( run ) {
        start_thread();
    }
}

Ati_Sens::~Ati_Sens ( void ) {

    if ( run ) {

        cmd_t ati_cmd = { htons ( 0x1234 ), htons ( 0x0000 ), 1 };
        send_cmd ( ati_cmd );
        usleep ( 200000 );
        run = false;
        //pthread_cancel(thread_id);
        pthread_join ( thread_id, NULL );
        DPRINTF ( "[Ati_Sens] Join thread\n" );
    }

    close ( udp_sock );
    iit::ecat::dump_buffer ( std::string ( "/tmp/ati_log.txt" ), ati_log );
    DPRINTF ( "[Ati_Sens] ~%s\n", typeid ( this ).name() );

}

int Ati_Sens::send_cmd ( cmd_t &ati_cmd ) {

    return send ( udp_sock, ( void* ) &ati_cmd, sizeof ( ati_cmd ), 0 );
}

void Ati_Sens::get_last_sample ( ati_log_t &sample ) {

    std::unique_lock<std::mutex> ( mtx );
    memcpy ( ( void* ) &sample, &last_sample, sizeof ( ati_log_t ) );
}

float Ati_Sens::getCountsPerUnit() const
{
    std::unique_lock<std::mutex> ( mtx );
    return countsPerUnit;
}

void Ati_Sens::setCountsPerUnit(float value)
{
    std::unique_lock<std::mutex> ( mtx );
    countsPerUnit = value;
}

int Ati_Sens::recv_data() {

    int         size;
    ati_raw_t   data;
    ati_log_t   log_item;


    size = recv ( udp_sock, &data, sizeof ( data ), 0 );
    if ( size < 0 ) {
        DPRINTF ( "[Ati_Sens] udp recv() %s\n", strerror ( errno ) );
        return size;
    }

    // adjust byte order ...
    log_item.rtd_seq = ntohl ( data.rtd_seq );
    log_item.ts = iit::ecat::get_time_ns() - start_time;
    for ( int i = 0; i < 6; i++ ) {
        log_item.ft[i] = ( float ) ( ( int32_t ) ntohl ( data.ft[i] ) ) /countsPerUnit;
    }
    
    if ( ati_log.full() ) {
        DPRINTF ( "!! ati_log full, overwriting sample\n"); 
    }
    ati_log.push_back ( log_item );

#if 0
    DPRINTF ( ">> %d %d 0x%04X \n", ntohl ( data.rtd_seq ), ntohl ( data.ft_seq ), ntohl ( data.status ) );
    for ( int i = 0; i < 6; i++ ) {
        //DPRINTF("%d\t", ntohl(data.ft[i]) );
        DPRINTF ( "%f\t", ( float ) ( ( int32_t ) ntohl ( data.ft[i] ) ) /countsPerUnit );
    }
    DPRINTF ( "\n" );
#endif

    std::unique_lock<std::mutex> ( mtx );
    memcpy ( ( void* ) &last_sample, &log_item, sizeof ( ati_log_t ) );

    return size;
}

void * Ati_Sens::rx_thread ( void *_ ) {

    Ati_Sens * kls = ( Ati_Sens* ) _;
    ati_log_t   log_item;

    // request continuous data ...
    cmd_t ati_cmd = { htons ( 0x1234 ), htons ( 0x0002 ), 0 };
    kls->send_cmd ( ati_cmd );

    sleep ( 1 );

    // bias
    ati_cmd.cmd = htons ( 0x0042 );
    kls->send_cmd ( ati_cmd );

    kls->start_time = iit::ecat::get_time_ns();

#ifdef __COBALT__
    pthread_setmode_np ( 0, PTHREAD_WARNSW, 0 );
#endif
    pthread_setname_np ( pthread_self(), "rx_ati" );

    while ( kls->run ) {

        kls->recv_data();

    }
    
    return 0;
}


void Ati_Sens::start_thread ( void ) {

    pthread_attr_t      attr;
    int                 policy;
    cpu_set_t           cpu_set;
    struct sched_param  schedparam;


#ifdef __COBALT__
    policy = SCHED_FIFO;
#else
    policy = SCHED_OTHER;
#endif

    CPU_ZERO ( &cpu_set );
    CPU_SET ( 2,&cpu_set );

    pthread_attr_init ( &attr );
    pthread_attr_setinheritsched ( &attr, PTHREAD_EXPLICIT_SCHED );
    pthread_attr_setschedpolicy ( &attr, policy );
    schedparam.sched_priority = sched_get_priority_max ( policy );
    pthread_attr_setschedparam ( &attr, &schedparam );
    pthread_attr_setdetachstate ( &attr, PTHREAD_CREATE_JOINABLE );
    pthread_attr_setaffinity_np ( &attr, sizeof ( cpu_set ), &cpu_set );

    DPRINTF ( "[Ati_Sens] Start ati_rx_thread\n" );
    run = true;
    pthread_create ( &thread_id, &attr, rx_thread, ( void* ) this );

}


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
