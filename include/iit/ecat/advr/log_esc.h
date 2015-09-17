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

#include <iit/ecat/utils.h>

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
        iit::ecat::dump_buffer(log_filename, esc_log);
    }

    void start_log(bool start) {
        _start_log = start;
        _start_log_ts = iit::ecat::get_time_ns();
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





#endif
