/*
 * ft6_esc.h
 * 
 *  Force Toruqe sensor
 *  based on TI tm4c123AH6PM - Tiva Microcontroller
 *  
 *  http://www.ti.com/product/tm4c123ah6pm
 *  
 *  Created on: Jan 2015
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_FT6_ESC_H__
#define __IIT_ECAT_ADVR_FT6_ESC_H__

#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/advr/esc.h>
#include <iit/ecat/utils.h>
#include <map>

namespace iit {
namespace ecat {
namespace advr {

struct FtESCTypes {
    // TX  slave_input -- master output
    typedef struct {
        uint64_t	ts;

    }  __attribute__((__packed__)) pdo_tx;

    // RX  slave_output -- master input
    typedef struct {
        float	    force_X;   		    // N
        float	    force_Y;   		    // N
        float	    force_Z;   		    // N
        float		torque_X;     		// Nm
        float		torque_Y;     		// Nm
        float		torque_Z;     		// Nm
        uint16_t    fault;
        uint64_t	rtt;        		// ns

        void sprint(char *buff, size_t size) {
            snprintf(buff, size, "%f\t%f\t%f\t%f\t%f\t%f\t%d\t%lld\n", force_X,force_Y,force_Z,torque_X,torque_Y,torque_Z,fault,rtt);
        }
        void fprint(FILE *fp) {
            fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f\t%d\t%lld\n", force_X,force_Y,force_Z,torque_X,torque_Y,torque_Z,fault,rtt);
        }

    }  __attribute__((__packed__)) pdo_rx;
};

typedef struct {
    unsigned long Block_control;
    long NumAvSamples;   

    unsigned long calibration_offset0;
    unsigned long calibration_offset1;
    unsigned long calibration_offset2;
    unsigned long calibration_offset3;
    unsigned long calibration_offset4;
    unsigned long calibration_offset5;
    
    float matrix_r1_c1;
    float matrix_r1_c2;
    float matrix_r1_c3;
    float matrix_r1_c4;
    float matrix_r1_c5;
    float matrix_r1_c6;
    
    float matrix_r2_c1;
    float matrix_r2_c2;
    float matrix_r2_c3;
    float matrix_r2_c4;
    float matrix_r2_c5;
    float matrix_r2_c6;
    
} FT6_tFlashParameters;

typedef struct 
{
    char        firmware_version[8];
    uint16_t    ack_board_fault;
    float       matrix_rn_c1;
    float       matrix_rn_c2;
    float       matrix_rn_c3;
    float       matrix_rn_c4;
    float       matrix_rn_c5;
    float       matrix_rn_c6;
    uint16_t    flash_params_cmd;
    uint16_t    flash_params_cmd_ack;     
} FT6_tParameters;

/**
 *  
 **/ 

class FtESC : public BasicEscWrapper<FtESCTypes>
{

public:
    typedef BasicEscWrapper<FtESCTypes> Base;
public:
    FtESC(const ec_slavet& slave_descriptor) :
           Base(slave_descriptor) {
    }

    virtual ~FtESC(void) { DPRINTF("~%s %d\n", typeid(this).name(), position); }

public:

    static FT6_tFlashParameters flash_param;
    static FT6_tParameters      param;

    static FtESCTypes::pdo_rx sdo_rx_pdo;
    static FtESCTypes::pdo_tx sdo_tx_pdo;

    static const objd_t SDOs[];
    //
    static const objd_t * SDOs6000;
    static const objd_t * SDOs7000;
    static const objd_t * SDOs8000;
    static const objd_t * SDOs8001;
    
    virtual const objd_t* get_SDOs() { return SDOs; };
    virtual const objd_t* get_SDOs6000() { return SDOs6000; };
    virtual const objd_t* get_SDOs7000() { return SDOs7000; };
    virtual const objd_t* get_SDOs8000() { return SDOs8000; };
    virtual const objd_t* get_SDOs8001() { return SDOs8001; };

};

typedef std::map<int, FtESC*>  FtSlavesMap;

}
}
}
#endif /* __IIT_ECAT_ADVR_FT6_ESC_H__ */
