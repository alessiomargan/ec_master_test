#ifndef __ATI_IFACE_H__
#define __ATI_IFACE_H__

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <boost/circular_buffer.hpp>

#include <mutex>
#include <string>

#include <iit/ecat/utils.h>
#include <protobuf/ecat_pdo.pb.h>

#define LOG_SIZE   10000000

namespace iit {
namespace advr {

typedef struct {
    uint16_t hdr;
    uint16_t cmd;
    uint32_t n_samples;
} cmd_t;

typedef struct {
    uint32_t    rtd_seq;
    uint32_t    ft_seq;
    uint32_t    status;
    int32_t     ft[6];
} ati_raw_t; // 36 bytes

typedef struct {
    uint32_t    rtd_seq;
    uint64_t    ts;
    float       ft[6];
    void sprint ( char *buff, size_t size ) {
        snprintf ( buff, size, "%lu\t%d\t%f\t%f\t%f\t%f\t%f\t%f\n", ts, rtd_seq, ft[0],ft[1],ft[2],ft[3],ft[4],ft[5] );
    }
    void fprint ( FILE *fp ) {
        fprintf ( fp, "%lu\t%d\t%f\t%f\t%f\t%f\t%f\t%f\n", ts, rtd_seq, ft[0],ft[1],ft[2],ft[3],ft[4],ft[5] );
    }
    void pb_toString( std::string * pb_str ) {
        static iit::advr::Ec_slave_pdo pb_rx_pdo;
        //static iit::advr::FT_ati_rx pb_ati;
        static struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        // Header
        pb_rx_pdo.mutable_header()->mutable_stamp()->set_sec(ts.tv_sec);
        pb_rx_pdo.mutable_header()->mutable_stamp()->set_nsec(ts.tv_nsec);
        // Type
        pb_rx_pdo.set_type(iit::advr::Ec_slave_pdo::RX_FT_ATI);
        //    
        pb_rx_pdo.mutable_ft_ati_rx()->set_aforce_x(ft[0]);
        pb_rx_pdo.mutable_ft_ati_rx()->set_aforce_y (ft[1]);
        pb_rx_pdo.mutable_ft_ati_rx()->set_aforce_z (ft[2]);
        pb_rx_pdo.mutable_ft_ati_rx()->set_atorque_x(ft[3]);
        pb_rx_pdo.mutable_ft_ati_rx()->set_atorque_y(ft[4]);
        pb_rx_pdo.mutable_ft_ati_rx()->set_atorque_z(ft[5]);
        pb_rx_pdo.SerializeToString(pb_str);
    }
    
} ati_log_t ; // 36 bytes




class Ati_Sens {


private:
    bool run;
    pthread_t   thread_id;

    std::mutex  mtx;
    ati_log_t   last_sample;

    int recv_data();
    int send_cmd ( cmd_t & );

public:

    Ati_Sens ( bool run = false );
    virtual ~Ati_Sens ( void );

    void get_last_sample ( ati_log_t &sample );

protected:

    void start_thread ( void );
    static void * rx_thread ( void * );

    int udp_sock;
    //struct sockaddr_in local_addr;
    struct sockaddr_in  dest_addr;

    uint64_t    start_time;
    boost::circular_buffer<ati_log_t> ati_log;

};

}
}

#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
