/*
 *
 *  Created on: Dec, 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_TEST_ESC_H__
#define __IIT_ECAT_ADVR_TEST_ESC_H__

#include <iit/ecat/advr/esc.h>

#include <map>
#include <string>


namespace iit {
namespace ecat {
namespace advr {


inline int set_SDO(int slave_pos, int index, int subindex, int size, void *data) {

    return ec_SDOwrite(slave_pos, index, subindex, false, size, data, EC_TIMEOUTRXM);
}

inline int set_SDO(int slave_pos, const objd_t *sdo) {

    char * err;
    ec_errort   ec_error;
    int final_size, wkc;

    if (!sdo) { return 0; }

    DPRINTF("set_SDO %s [%d.0x%04X.0x%02X]\n", sdo->name, slave_pos, sdo->index, sdo->subindex);
    final_size = sdo->bitlength/8;
    wkc = set_SDO(slave_pos, sdo->index, sdo->subindex, final_size, sdo->data);
    if ( wkc <= 0 || final_size!=sdo->bitlength/8 ) {
        DPRINTF("Slave %d >> ", slave_pos);
        if ( wkc <= 0 ) {
            err =  ec_elist2string();
            DPRINTF("Ec_error : %s\n", err);
        } else {
            DPRINTF("SDO write fail : %d != %d\n", final_size, sdo->bitlength/8);
        }
    } else {
        // OK ....
    }
    return wkc; 
}

inline int get_SDO(int slave_pos, int index, int subindex, int *size, void *data) {

    return ec_SDOread(slave_pos, index, subindex, FALSE, size, data, EC_TIMEOUTRXM);
}

inline int get_SDO(int slave_pos, const objd_t *sdo) {

    char * err;
    ec_errort   ec_error;
    int final_size, wkc;

    if (!sdo) { return 0; }

    DPRINTF("get_SDO %s [%d.0x%04X.0x%02X]\n", sdo->name, slave_pos, sdo->index, sdo->subindex);
    final_size = sdo->bitlength/8;
    wkc = get_SDO(slave_pos, sdo->index, sdo->subindex, &final_size, sdo->data);
    if ( wkc <= 0 || final_size!=sdo->bitlength/8 ) {
        DPRINTF("Slave %d >> ", slave_pos);
        if ( wkc <= 0 ) {
            err =  ec_elist2string();
            DPRINTF("Ec_error : %s\n", err);
        } else {
            DPRINTF("SDO read fail : %d != %d\n", final_size, sdo->bitlength/8);
        }
    } else {
        // OK ....
    }
    return wkc; 
}



struct TestEscPdoTypes {
    // TX  slave_input -- master output
    typedef struct {
        uint16_t    _type;
        int32_t     _value;
        uint64_t    _ts;
    } __attribute__((__packed__)) pdo_tx;


    // RX  slave_output -- master input
    typedef struct {
        uint8_t     _bit_0:1;
        uint8_t     _bit_1:1;
        uint8_t     _bit_2:1;
        uint8_t     _bit_3:1;
        uint8_t     _bit_4:1;
        uint8_t     _bit_5:1;
        uint8_t     _bit_6:1;
        uint8_t     _bit_7:1;
        uint8_t     _bits;
        int8_t      _sint;
        uint8_t     _usint;
        int16_t     _int;
        uint16_t    _uint;
        int32_t     _dint;
        uint32_t    _udint;
        int64_t     _lint;
        uint64_t    _ulint;
        float       _real;
    } __attribute__((__packed__)) pdo_rx;
};



template<class ESCParamTypes>
class TestESCtemp : public BasicEscWrapper<TestEscPdoTypes>
{
public:
    typedef BasicEscWrapper<TestEscPdoTypes>    Base;
    typedef typename ESCParamTypes::flashParam  flash_param_t;
    typedef typename ESCParamTypes::ramParam    ram_param_t;

public:
    TestESCtemp(const ec_slavet& slave_descriptor) :
        Base(slave_descriptor) { }

    virtual ~TestESCtemp(void) { DPRINTF("~%s %d\n", typeid(this).name(), position); }

    void init(void);

    flash_param_t& getFlashParam() const;
    ram_param_t&   getRamParam() const;

    int set_SDO_byname(const char * name);
    int get_SDO_byname(const char * name);

    virtual const objd_t * get_SDOs() = 0;

protected:
    static flash_param_t flash_param;
    static ram_param_t   ram_param;

    std::map<std::string, const objd_t*> sdo_look_up;
};

#define TEMPL template<class ESCParamTypes>
#define CLASS TestESCtemp<ESCParamTypes>
#define SIGNATURE(type) TEMPL inline type CLASS

SIGNATURE(void)::init(void) {

    const objd_t * sdo = get_SDOs();
    while ( sdo && sdo->index ) {
        sdo_look_up[sdo->name] = sdo;
        get_SDO(position, sdo);
        sdo ++;
    }
}

SIGNATURE(typename CLASS::flash_param_t&)::getFlashParam() const
{
    return flash_param;
}

SIGNATURE(typename CLASS::ram_param_t&)::getRamParam() const
{
    return ram_param;
}

SIGNATURE(int)::set_SDO_byname(const char * name) {

    // look up name in SDOs
    const objd_t * sdo = sdo_look_up[name];
    return set_SDO(position, sdo);
}

SIGNATURE(int)::get_SDO_byname(const char * name) {

    // look up name in SDOs
    const objd_t * sdo = sdo_look_up[name];
    return get_SDO(position, sdo);
}

#undef SIGNATURE
#undef CLASS
#undef TEMPL


///////////////////////////////////////////////////////////////////////////////

struct TestESCParamTypes {
    // flash param
    typedef struct {
        int	    par_1;
        int	    par_2;
    }  __attribute__((__packed__)) flashParam;

    // ram param
    typedef struct {
        //float		dummy;
    }  __attribute__((__packed__)) ramParam;
};

class TestESC : public TestESCtemp<TestESCParamTypes> 
{

private:
    static const objd_t SDOs[];

public:
    TestESC(const ec_slavet& slave_descriptor) :
        TestESCtemp<TestESCParamTypes>(slave_descriptor)
    {
        init(); 
    }

    virtual const objd_t * get_SDOs() { return SDOs; }

};



}
}
}

#endif


