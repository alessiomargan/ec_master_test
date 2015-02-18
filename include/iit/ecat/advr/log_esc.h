/*
 *
 *  Created on: Dec, 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_LOG_H__
#define __IIT_ECAT_ADVR_LOG_H__

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

    void push_back(const EscLogTypes& item) {
        esc_log.push_back(item);
    }

protected:

    std::string log_filename;
    boost::circular_buffer<EscLogTypes> esc_log;
};


#endif
