/*
 *
 *  Created on: Dec, 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_LOG_H__
#define __IIT_ECAT_ADVR_LOG_H__

#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <malloc.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>

#ifdef __XENO__
    #include <iit/ecat/advr/rt_ipc.h>
#else
    #include <sys/types.h>
    #include <sys/stat.h>
#endif

#include <boost/circular_buffer.hpp>

// at 1kHz ecat DC clock 3 mins
#define DEFAULT_LOG_SIZE   (3 * 60 * 1e3L)

template<class EscLogTypes>
class PDO_log
{
public:
    typedef EscLogTypes    log_t;

    PDO_log(std::string filename, uint32_t capacity): log_filename(filename) {
        esc_log.set_capacity(capacity);
    }
    virtual ~PDO_log() {
        dump_buffer(log_filename, esc_log);
    }

    void start_log(bool start) {
        _start_log = start;
        _start_log_ts = get_time_ns();
    }

    void push_back(const log_t & item) {
        esc_log.push_back(item);
    }

protected:

    std::string log_filename;
    boost::circular_buffer<EscLogTypes> esc_log;

    bool        _start_log;
    uint64_t    _start_log_ts;

};

/**
 * 
 */

template<class XddpTxTypes, class XddpRxTypes>
class XDDP_pipe {

public:
    typedef XddpTxTypes    xddp_tx_t;
    typedef XddpRxTypes    xddp_rx_t;

    XDDP_pipe(int pool_size = 8192):
        pool_size(pool_size)
    {
        fd = 0;
    }

    void init(const std::string pipe_name) {
        
#ifdef __XENO__
        fd = xddp_bind(pipe_name.c_str(), pool_size);
#else
        pipe_prefix =  "/tmp";
        std::string pipe = pipe_prefix + pipe_name;
        mkfifo(pipe.c_str(), S_IRWXU|S_IRWXG);
        fd = open(pipe.c_str(), O_RDWR | O_NONBLOCK);
#endif
        assert(fd);
    }
    
    virtual ~XDDP_pipe()
    {
        if ( ! fd ) { return; }
        
        close(fd);
#ifndef __XENO__
        std::string pipe = pipe_prefix + pipe_name;
        unlink(pipe.c_str());
#endif
    }

    int xddp_write(xddp_tx_t & tx)
    {
        char buff[pool_size];
        
        if ( ! fd ) { return 0; }
        tx.sprint(buff, sizeof(buff));
        return ::write(fd, buff, strlen(buff));
    }

    int xddp_read(void *buffer, ssize_t buff_size)
    {
         if ( ! fd ) { return 0; }
        /////////////////////////////////////////////////////////
        // NON-BLOCKING, read buff_size byte from pipe or cross domain socket
#if __XENO__
        return recvfrom(fd, buffer, buff_size, MSG_DONTWAIT, NULL, 0);
#else
        // NON-BLOCKING
        return ::read(fd, buffer, buff_size);
#endif
    }


protected:
    int fd;
    int pool_size;
    std::string pipe_prefix;
    std::string pipe_name;
};



#endif
