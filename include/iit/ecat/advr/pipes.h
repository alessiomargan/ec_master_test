/*
 *
 *  Created on: Apr, 2015
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_PIPE_H__
#define __IIT_ECAT_ADVR_PIPE_H__

#ifdef __XENO__
    #include <iit/ecat/advr/rt_ipc.h>
#else
    #include <sys/types.h>
    #include <sys/stat.h>
#endif

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
        pipe_prefix =  "/tmp/";
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