/*
 * mc_hipwr_esc.h
 * 
 *  HiPower Motor Controlleer 
 *  based on TI TMS320F28335 - Delfino Microcontroller
 *  High-Performance 32-Bit CPU 150 Mhz
 *  
 *  http://www.ti.com/product/tms320f28335
 *  
 *  Created on: Dec 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_MC_HIPWR_ESC_H__
#define __IIT_ECAT_ADVR_MC_HIPWR_ESC_H__

#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/advr/esc.h>
#include <iit/ecat/utils.h>
#include <map>

#define DTYPE_UNSIGNED8 0
#define DTYPE_UNSIGNED16 1
#define DTYPE_INTEGER16 2
#define DTYPE_UNSIGNED64 3
#define DTYPE_REAL32 4
#define DTYPE_VISIBLE_STRING 5

#define ATYPE_RO 17
#define ATYPE_RW 18

namespace iit {
namespace ecat {
namespace advr {


struct McESCTypes {
    // TX  slave_input -- master output
    typedef struct {
        float	    pos_ref;
        float		tor_offs;
        float		PosGainP;
        float		PosGainI;
        float		PosGainD;
        uint64_t	ts;

    }  __attribute__((__packed__)) pdo_tx;

    // RX  slave_output -- master input
    typedef struct {
        float	    position;   		// rad
        float		velocity;   		// rad/s
        float		torque;     		// Nm
        float		max_temperature; 	// C
        uint16_t    fault;
        uint64_t	rtt;        		// ns
    }  __attribute__((__packed__)) pdo_rx;
};


typedef struct {

    unsigned long Sensor_type;      // Sensor type: NOT USED

    float TorGainP;
    float TorGainI;
    float TorGainD;
    float TorGainFF;

    float Pos_I_lim;                // Integral limit: NOT USED
    float Tor_I_lim;                // Integral limit: NOT USED

    float Min_pos;
    float Max_pos;
    float Max_vel;
    float Max_tor;
    float Max_cur;

    float Enc_offset;
    float Enc_relative_offset;
    float Phase_angle;
    float Torque_lin_coeff;

} tFlashParameters;

typedef struct 
{
    char        firmware_version[8];
    uint16_t    ack_board_fault_all;
    float       Direct_ref;
    float       V_batt_filt_100ms;
    float       Board_Temperature;
    float       T_mot1_filt_100ms;
    uint16_t    ctrl_status_cmd;
    uint16_t    ctrl_status_cmd_ack;
    uint16_t    flash_params_cmd;
    uint16_t    flash_params_cmd_ack;
    uint64_t    abs_enc_mot;
    uint64_t    abs_enc_load;     
    float       angle_enc_mot;
    float       angle_enc_load;     
} tParameters;

/**
 *  
 **/ 

class McESC : public BasicEscWrapper<McESCTypes>
{

public:
    typedef BasicEscWrapper<McESCTypes> Base;
public:
    McESC(const ec_slavet& slave_descriptor) :
           Base(slave_descriptor) {
    }

    virtual ~McESC(void) { DPRINTF("~%s %d\n", typeid(this).name(), position); }


    int start(void);
    int stop(void);

public:

    static tFlashParameters flash_param;
    static tParameters      param;
    static McESCTypes::pdo_rx sdo_rx_pdo;
    static McESCTypes::pdo_tx sdo_tx_pdo;


    static const objd_t SDOs[];
    //
    static const objd_t * SDOs6000;
    static const objd_t * SDOs7000;
    static const objd_t * SDOs8000;
    static const objd_t * SDOs8001;

};

typedef std::map<int, McESC *>  McSlavesMap;


}
}
}
#endif /* IIT_ECAT_ADVR_ESC_H_ */
