/*
 *
 *  Created on: Dec, 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_ESC_H__
#define __IIT_ECAT_ADVR_ESC_H__

#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/utils.h>

#include <map>
#include <fstream>

#include <boost/circular_buffer.hpp>

#define DEG2RAD(X)  ((float)X*M_PI)/180.0

#define J2M(p,s,o)  (M_PI - (s*o) + (s*p)) 
#define M2J(p,s,o)  ((p - M_PI + (s*o))/s) 

// Control commands
#define CTRL_POWER_MOD_ON		0x00A5
#define CTRL_POWER_MOD_OFF		0x005A
#define CTRL_SET_IMPED_MODE		0x00D4
#define CTRL_SET_POS_MODE		0x003B
#define CTRL_SET_DIRECT_MODE	0x004F
#define CTRL_FAN_ON				0x0026
#define CTRL_FAN_OFF			0x0062
#define CTRL_LED_ON				0x0019
#define CTRL_LED_OFF			0x0091

//#define CTRL_ALIGN_ENCODERS		0x00B2
#define CTRL_SET_ZERO_POSITION	0x00AB

// FT6
#define CTRL_REMOVE_TORQUE_OFFS	0x00CD

#define CTRL_CMD_DONE			0x7800
#define CTRL_CMD_ERROR			0xAA00


namespace iit {
namespace ecat {
namespace advr {


inline int check_cmd_ack(int16_t cmd, int16_t ack)
{
    if ( ack == ((cmd & 0x00FF) | CTRL_CMD_DONE) ) {
        DPRINTF("DONE 0x%04X\n", cmd);
        return 0;
    } else if ( ack == ((cmd & 0x00FF) | CTRL_CMD_ERROR) ) {
        DPRINTF("FAIL 0x%04X\n", cmd);
        return -1;
    } else {
        DPRINTF("PROTOCOL FAILURE cmd 0x%04X ack 0x%04X !!!\n", cmd, ack);
        return -2;
    }

}

struct FAULT_BITS {
    uint16_t  rxpdo_pos_ref:1;
    uint16_t  rxpdo_tor_offs:1;
    uint16_t  rxpdo_kp_pos:1;
    uint16_t  rxpdo_ki_pos:1;
    uint16_t  rxpdo_kd_pos:1;
    uint16_t  fault_encoder_1:1;
    uint16_t  fault_encoder_2:1;
    uint16_t  fault_hardware:1;
    uint16_t  params_out_of_range:1;
    uint16_t  Max_cur_limited_for_temp:1;
    uint16_t  flag_10:1;
    uint16_t  flag_11:1;
    uint16_t  flag_12:1;
    uint16_t  flag_13:1;
    uint16_t  flag_14:1;
    uint16_t  irq_alive:1;
    uint16_t  spare;
};

typedef union{
    uint32_t            all;
    struct FAULT_BITS   bit;
} FAULT;


struct McEscPdoTypes {
    // TX  slave_input -- master output
    typedef struct {
        float	    pos_ref;
        float		tor_offs;
        float		PosGainP;
        float		PosGainI;
        float		PosGainD;
        uint64_t	ts;

        void fprint(FILE *fp) {
            fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%lu\n", pos_ref,tor_offs,PosGainP,PosGainI,PosGainD,ts);
        }
        void sprint(char *buff, size_t size) {
            snprintf(buff, size, "%f\t%f\t%f\t%f\t%f\t%lu\n", pos_ref,tor_offs,PosGainP,PosGainI,PosGainD,ts);
        }

    }  __attribute__((__packed__)) pdo_tx;

    // RX  slave_output -- master input
    typedef struct {
        float		max_temperature; 	// C
        float	    position;   		// rad
        float		velocity;   		// rad/s
        float		torque;     		// Nm
        int32_t     fault;
        uint64_t	rtt;        		// ns

        void fprint(FILE *fp) {
            fprintf(fp, "%f\t%f\t%f\t%f\t0xX%\t%lu\n", max_temperature,position,velocity,torque,fault,rtt);
        }
        void sprint(char *buff, size_t size) {
            snprintf(buff, size, "%f\t%f\t%f\t%f\t0x%X\t%lu\n", max_temperature,position,velocity,torque,fault,rtt);
        }
    }  __attribute__((__packed__)) pdo_rx;
};




template<class EscPDOTypes>
class PDO_log
{
public:
    typedef typename EscPDOTypes::pdo_rx    pdo_rx_t;

    PDO_log(std::string filename, int capacity): log_filename(filename) {
        esc_log.set_capacity(capacity);
    }
    virtual ~PDO_log() {
        dump_buffer(log_filename, esc_log);
    }

    void push_back(const pdo_rx_t& item) {
        esc_log.push_back(item);
    }

protected:

    std::string log_filename;
    boost::circular_buffer<pdo_rx_t> esc_log;
};

} 
}
}

#endif /* IIT_ECAT_ADVR_ESC_H_ */
