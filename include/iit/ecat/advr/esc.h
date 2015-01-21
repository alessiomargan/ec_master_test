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

#define DTYPE_INTEGER16         0x0001
#define DTYPE_INTEGER32         0x0002
#define DTYPE_UNSIGNED8         0x0003
#define DTYPE_UNSIGNED16        0x0004
#define DTYPE_REAL32            0x0005
#define DTYPE_VISIBLE_STRING    0x0006
#define DTYPE_UNSIGNED64        0x0007

#define ATYPE_RO 17
#define ATYPE_RW 18

namespace iit {
namespace ecat {
namespace advr {

typedef struct
{
    int index;
    int subindex;
    int datatype;
    int bitlength;
    int access;
    const char * name;
    void * data;
} objd_t;


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
        float		max_temperature; 	// C
        float	    position;   		// rad
        float		velocity;   		// rad/s
        float		torque;     		// Nm
        uint16_t    fault;
        uint64_t	rtt;        		// ns

        void fprint(FILE *fp) {
            fprintf(fp, "%f\t%f\t%f\t%f\t%d\t%lld\n", max_temperature,position,velocity,torque,fault,rtt);
        }

    }  __attribute__((__packed__)) pdo_rx;
};


class McESC : public BasicEscWrapper<McESCTypes>
{

public:
    typedef BasicEscWrapper<McESCTypes> Base;
public:
    McESC(const ec_slavet& slave_descriptor) :
           Base(slave_descriptor) {
    }

    virtual ~McESC(void) { DPRINTF("~%s %d\n", typeid(this).name(), position); }

public:
   
    virtual const objd_t * get_SDOs() = 0;
    virtual const objd_t * get_SDOs6000() = 0;
    virtual const objd_t * get_SDOs7000() = 0;
    virtual const objd_t * get_SDOs8000() = 0;
    virtual const objd_t * get_SDOs8001() = 0;
    
    static McESCTypes::pdo_rx sdo_rx_pdo;
    static McESCTypes::pdo_tx sdo_tx_pdo;

};


// typedef std::shared_ptr<McESC>  McESCPtr;
typedef std::map<int, McESC*>  McSlavesMap;

    

} 
}
}

#endif /* IIT_ECAT_ADVR_ESC_H_ */
