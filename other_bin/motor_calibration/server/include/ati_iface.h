#ifndef __ATI_IFACE_H__
#define __ATI_IFACE_H__

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <boost/circular_buffer.hpp>

#include <mutex>
#include <string>

#include <iit/ecat/utils.h>
#include <iit/advr/thread_util.h>
#include <iit/ecat/advr/pipes.h>

#define LOG_SIZE   1000000
#define ATI_IFACE_IP    "192.168.1.1"

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
} ati_log_t ; // 36 bytes


// countsPerUnit
// small sensor 1000000 big sensor 1000

class Ati_Sens {


private:
    bool run;
    pthread_t   thread_id;

    std::mutex  mtx;
    ati_log_t   last_sample;
    float countsPerUnit;

    int recv_data();
    int send_cmd ( cmd_t & );

public:

    Ati_Sens ( ) {}
    virtual ~Ati_Sens ( void );

    void config( bool run = false, float countsPerUnit_=1000.0,
               const std::string& _ip = ATI_IFACE_IP);

    void start_thread ( void );
    void get_last_sample ( ati_log_t &sample );
    void zeroOffset();

    float getCountsPerUnit() const;
    void setCountsPerUnit(float value);

protected:

    static void * rx_thread ( void * );

    int udp_sock;
    //struct sockaddr_in local_addr;
    struct sockaddr_in  dest_addr;

    uint64_t    start_time;
    boost::circular_buffer<ati_log_t> ati_log;
    std::string ati_iface_ip;

private:
    XDDP_pipe pipeout;

};



#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
