#ifndef __PIPES_H__
#define __PIPES_H__

#include <sys/socket.h>
#include <sys/stat.h>
#include <string>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#ifdef __XENO__
    #include <iit/ecat/advr/rt_ipc.h>
#endif


class XDDP_pipe {

public:
    XDDP_pipe(const std::string pipe_name, int pool_size):
    pipe_name(pipe_name),
    pool_size(pool_size)
    {
#ifdef __XENO__
        fd = xddp_bind(pipe_name.c_str(), pool_size);
#else
        std::string pipe = pipe_prefix + pipe_name;
        mkfifo(pipe.c_str(), S_IRWXU|S_IRWXG);
        fd = open(pipe.c_str(), O_RDWR | O_NONBLOCK);
#endif
        assert(fd);
    }

    virtual ~XDDP_pipe()
    {
        close(fd);
#ifndef __XENO__
        std::string pipe = pipe_prefix + pipe_name;
        unlink(pipe.c_str());
#endif

    }

protected:
    int fd;
    int pool_size;
    std::string pipe_name;
};


class Write_XDDP_pipe : public XDDP_pipe {
public:
    Write_XDDP_pipe(std::string pipe_name, int pool_size):
    XDDP_pipe(pipe_name, pool_size) {}

    int write(void *buffer, int nbytes)
    {
        return ::write(fd, buffer, nbytes);
    }
};

class Read_XDDP_pipe : public XDDP_pipe {
public:
    Read_XDDP_pipe(std::string pipe_name, int pool_size):
    XDDP_pipe(pipe_name, pool_size) {}

    int read(void *buffer, ssize_t buff_size)
    {
        /////////////////////////////////////////////////////////
        // NON-BLOCKING, read buff_size byte from pipe or cross domain socket
#if __XENO__
        return recvfrom(fd, buffer, buff_size, MSG_DONTWAIT, NULL, 0);
#else
        // NON-BLOCKING
        return ::read(fd, buffer, buff_size);
#endif
    }
};

#endif

