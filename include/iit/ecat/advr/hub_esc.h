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


/**
*  
**/ 

class HubESC : public EscWrapper {

public:
    HubESC(const ec_slavet& slave_descriptor) :
        EscWrapper(slave_descriptor)
    {
    }

    virtual ~HubESC(void) {
        DPRINTF("~%s %d\n", typeid(this).name(), position);
    }

    virtual const objd_t * get_SDOs() { return 0; }
    virtual uint16_t get_ESC_type() { return HUB; }

private:
    virtual void readPDO() {};
    virtual void writePDO() {};

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
