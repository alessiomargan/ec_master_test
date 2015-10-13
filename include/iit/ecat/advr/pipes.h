/*
 *
 *  Created on: Apr, 2015
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_PIPE_H__
#define __IIT_ECAT_ADVR_PIPE_H__

#include <string>

#if __XENO__
    static const std::string pipe_prefix("/proc/xenomai/registry/rtipc/xddp/");
#else
    static const std::string pipe_prefix("/tmp/");
#endif


#ifdef __XENO__
    #include <iit/ecat/advr/rt_ipc.h>
#else
    #include <sys/types.h>
    #include <sys/stat.h>
#if 0
    static int xddp_bind(const char * label, size_t local_poolsz) {
    
        int socket_fd;
        struct sockaddr_un server_address; 
        struct sockaddr_un client_address; 
        int bytes_received, bytes_sent, address_length;
        int integer_buffer;
        socklen_t address_length = sizeof(struct sockaddr_un);

        if((socket_fd = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0) {
            perror("server: socket");
            return 1;
        }
        
        memset(&server_address, 0, sizeof(server_address));
        server_address.sun_family = AF_UNIX;
        strcpy(server_address.sun_path, "./UDSDGSRV");
        
        unlink("./UDSDGSRV");       
        if(bind(socket_fd, (const struct sockaddr *) &server_address, sizeof(server_address)) < 0) {
            close(socket_fd);
            perror("server: bind");
            return 1;
        }
    }
#endif
#endif


template<class XddpTxTypes, class XddpRxTypes>
class XDDP_pipe {

public:
    typedef XddpTxTypes    xddp_tx_t;
    typedef XddpRxTypes    xddp_rx_t;

    XDDP_pipe(int _pool_size = 8192):
        pool_size(_pool_size)
    {
        fd = 0;
    }

    void init(const std::string pipe_name) {
        
        std::string pipe = pipe_prefix + pipe_name;

#ifdef __XENO__
        fd = xddp_bind(pipe_name.c_str(), pool_size);
#else
        mkfifo(pipe.c_str(), S_IRWXU|S_IRWXG);
        fd = open(pipe.c_str(), O_RDWR | O_NONBLOCK);
#endif
        DPRINTF(" .... open %s\n", pipe.c_str());
        assert(fd > 0);
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
        //char buff[pool_size];
        
        if ( fd <= 0) { return 0; }
        //tx.sprint(buff,sizeof(buff));
        //printf("%s\n", buff);
        return ::write(fd, (void*)&tx, sizeof(tx));
    }

    int xddp_read(xddp_rx_t & rx)
    {
         if ( fd <= 0) { return 0; }
        /////////////////////////////////////////////////////////
        // NON-BLOCKING, read buff_size byte from pipe or cross domain socket
#if __XENO__
        return recvfrom(fd, (void*)&rx, sizeof(rx), MSG_DONTWAIT, NULL, 0);
#else
        // NON-BLOCKING
        return read(fd, (void*)&rx, sizeof(rx));
#endif
    }


protected:
    std::string pipe_name;

private:
    int fd;
    int pool_size;

};

#endif