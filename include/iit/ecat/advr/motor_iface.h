/*
 *
 *  Created on: Dec, 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_MOTOR_H__
#define __IIT_ECAT_ADVR_MOTOR_H__

namespace iit {
namespace ecat {
namespace advr {

#include <iit/ecat/advr/esc.h>

class HpESC;
class LpESC;

template <typename MotorPdoTypes>
class AbsMotor
{
public:
    typedef typename MotorPdoTypes::pdo_rx    motor_pdo_rx_t;
    typedef typename MotorPdoTypes::pdo_tx    motor_pdo_tx_t;

private:
    template <class C, typename T> 
    int writeSDO_impl(std::string const & name, T value ) {
        C *c = dynamic_cast<C*>(this);
        if (!c) { return EC_WRP_NOK; }
        return c->writeSDO_byname( name.c_str(), value );
    }

    template <class C, typename T>
    int readSDO_impl(std::string const & name, T & value ) {
        C *c = dynamic_cast<C*>(this);
        if (!c) { return EC_WRP_NOK; }
        return c->readSDO_byname( name.c_str(), value );
    }

public:

    template<typename T>
    int writeSDO(std::string const & name, T value ) {
        if ( am_i_HpESC() ) {
            return writeSDO_impl<HpESC>(name, value);
        } else if ( am_i_LpESC() ) {
            return writeSDO_impl<LpESC>(name, value);
        }
        return EC_WRP_NOK;
    }

    template<typename T>
    int readSDO(std::string const & name, T & value ) {
        if ( am_i_HpESC() ) {
            return readSDO_impl<HpESC>(name, value);
        } else if ( am_i_LpESC() ) {
            return readSDO_impl<LpESC>(name, value);
        }
        return EC_WRP_NOK;
    }

    virtual int init(const YAML::Node &) = 0;
    virtual int start(int controller_type, float _p, float _i, float _d) = 0;
    virtual int stop(void) = 0;

    virtual const motor_pdo_rx_t & getRxPDO() const = 0;
    virtual const motor_pdo_tx_t & getTxPDO() const = 0;
    virtual void setTxPDO(const motor_pdo_tx_t &) = 0;

    virtual int set_posRef(float joint_pos) = 0;
    virtual int set_torOffs(float tor_offs) = 0;
    virtual int set_posGainP(float p_gain)  = 0;
    virtual int set_posGainI(float i_gain)  = 0;
    virtual int set_posGainD(float d_gain)  = 0;

    //virtual int get_pos(float &joint_pos)   = 0;
    //virtual int get_posGainP(float &p_gain) = 0;
    //virtual int get_posGainI(float &i_gain) = 0;
    //virtual int get_posGainD(float &d_gain) = 0;

    virtual void handle_fault(void) = 0;

    virtual void set_off_sgn(float offset, int sgn) = 0;
    virtual void start_log(bool start) = 0;

    void set_state(ec_state state) { _actual_state = state; }

protected:
    
    virtual bool am_i_HpESC() = 0;
    virtual bool am_i_LpESC() = 0;

    ec_state     _actual_state;
};

typedef AbsMotor<McEscPdoTypes> Motor;


}
}
}

#endif