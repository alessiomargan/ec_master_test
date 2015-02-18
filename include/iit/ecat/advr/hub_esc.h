/*
 * hub_esc.h
 * 
 *  EtherCAT hub
 *  
 *  Created on: Jan 2015
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_HUB_ESC_H__
#define __IIT_ECAT_ADVR_HUB_ESC_H__

#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/advr/esc.h>
#include <iit/ecat/utils.h>
#include <map>

namespace iit {
    namespace ecat {
        namespace advr {

struct HubEscPdoTypes {
    // TX  slave_input -- master output
    typedef struct {
    }  __attribute__((__packed__)) pdo_tx;
    // RX  slave_output -- master input
    typedef struct {
    }  __attribute__((__packed__)) pdo_rx;
};

struct HubEscSdoTypes {
};

/**
*  
**/ 

class HubESC : public BasicEscWrapper<HubEscPdoTypes,HubEscSdoTypes> {

public:
    typedef BasicEscWrapper<HubEscPdoTypes,HubEscSdoTypes> Base;
public:
    HubESC(const ec_slavet& slave_descriptor) :
        Base(slave_descriptor)
    {
    }

    virtual ~HubESC(void) {
        DPRINTF("~%s %d\n", typeid(this).name(), position);
    }

    virtual const objd_t * get_SDO_objd() { return 0; }
    virtual void init_SDOs(void)    { }
    virtual uint16_t get_ESC_type() { return HUB; }

private:

};

class HubIoESC : public HubESC {

public:
    HubIoESC(const ec_slavet& slave_descriptor) :
        HubESC(slave_descriptor) { }

    virtual uint16_t get_ESC_type() { return HUB_IO; }

    int read_io_reg(uint16_t &io) {

        int wc = ec_FPRD(configadr, 0x1000, sizeof(io), &io, EC_TIMEOUTRET3);
        if ( wc <= 0 ) {
            DPRINTF("ERROR FPRD(%x, 0x1000)\n", configadr);
        }
        return wc;
    }

private:

};


}
}
}
#endif /* __IIT_ECAT_ADVR_HUB_ESC_H__ */
