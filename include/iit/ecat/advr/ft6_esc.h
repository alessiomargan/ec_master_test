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

struct Ft6EscPdoTypes {
    // TX  slave_input -- master output
    typedef struct {
        uint64_t    ts;
    }  __attribute__((__packed__)) pdo_tx;
    // RX  slave_output -- master input
    typedef struct {
        float       force_X;            // N
        float       force_Y;            // N
        float       force_Z;            // N
        float       torque_X;           // Nm
        float       torque_Y;           // Nm
        float       torque_Z;           // Nm
        uint16_t    fault;
        uint64_t    rtt;                // ns
        void sprint(char *buff, size_t size) {
            snprintf(buff, size, "%f\t%f\t%f\t%f\t%f\t%f\t%d\t%lu\n", force_X,force_Y,force_Z,torque_X,torque_Y,torque_Z,fault,rtt);
        }
        void fprint(FILE *fp) {
            fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f\t%d\t%lu\n", force_X,force_Y,force_Z,torque_X,torque_Y,torque_Z,fault,rtt);
        }
    }  __attribute__((__packed__)) pdo_rx;
};

struct Ft6EscSdoTypes {

    // flash

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

    // ram

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
};

/**
*  
**/ 

class Ft6ESC : public BasicEscWrapper<Ft6EscPdoTypes,Ft6EscSdoTypes> {

public:
    typedef BasicEscWrapper<Ft6EscPdoTypes,Ft6EscSdoTypes> Base;
public:
    Ft6ESC(const ec_slavet& slave_descriptor) :
        Base(slave_descriptor)
    {
        init_SDOs();
        init_sdo_lookup();
    }

    virtual ~Ft6ESC(void) {
        delete [] SDOs;
        DPRINTF("~%s %d\n", typeid(this).name(), position); }
    
    int set_cal_matrix(std::vector<std::vector<float>> &cal_matrix);

    virtual const objd_t * get_SDO_objd() { return SDOs; }
    virtual void init_SDOs(void);

private:
    objd_t * SDOs;
};


typedef std::map<int, Ft6ESC*>  FtSlavesMap;

inline int Ft6ESC::set_cal_matrix(std::vector<std::vector<float>> &cal_matrix)
{
    int     res = 0;
    int16_t ack;
    int16_t flash_row_cmd = 0x00C7;
    
    for ( int r=0; r<6; r++ ) {
    
        // set row n param value
        set_SDO_byname<float>("matrix_rn_c1",cal_matrix[r][0]);
        set_SDO_byname<float>("matrix_rn_c2",cal_matrix[r][1]);
        set_SDO_byname<float>("matrix_rn_c3",cal_matrix[r][2]);
        set_SDO_byname<float>("matrix_rn_c4",cal_matrix[r][3]);
        set_SDO_byname<float>("matrix_rn_c5",cal_matrix[r][4]);
        set_SDO_byname<float>("matrix_rn_c6",cal_matrix[r][5]);
    
        set_SDO_byname<int16_t>("flash_params_cmd", flash_row_cmd);
        get_SDO_byname<int16_t>("flash_params_cmd_ack", ack);
    
        if ( (res=check_cmd_ack(flash_row_cmd, ack)) ) {
              return res;
        }

        // next row cmd
        flash_row_cmd++;

    } // for rows

    return res;
}



}
}
}
#endif /* __IIT_ECAT_ADVR_FT6_ESC_H__ */
