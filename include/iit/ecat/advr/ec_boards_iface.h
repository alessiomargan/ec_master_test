/*
   ec_boards_iface.h

   Copyright (C) 2014 Italian Institute of Technology

   Developer:
       Alessio Margan (2014-, alessio.margan@iit.it)

*/


#ifndef __EC_BOARDS_IFACE_H__
#define __EC_BOARDS_IFACE_H__

#include <iit/ecat/ec_master_iface.h>
#include <iit/ecat/slave_wrapper.h>

#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/mc_hipwr_esc.h>
//#include <iit/ecat/advr/mc_lopwr_esc.h>

#include <iit/ecat/advr/pipes.h>

#include <string>

namespace iit {
namespace ecat {
namespace advr {



/**
 * @class Ec_Boards_ctrl
 * @brief Boards_ctrl class
 */

class Ec_Boards_ctrl {

public:
    Ec_Boards_ctrl(const char * config);
    ~Ec_Boards_ctrl();

    int init(void);

    void configure_boards(void);

    int set_operative();

    int recv_from_slaves(void);

    int send_to_slaves(void);

    int handle_SDO(void);

#if 0
    int set_position(int *, int);
    int set_velocity(short *, int);
    int set_torque(short *, int);
    int set_position_velocity(int *, short *, int);
    int set_gravity_compensation(int *des_gc, int nbytes);
    int set_stiffness_damping(int *des_stiff, int *des_damp, int nElem);
    int set_pid_offset(short *, int);

    int set_position_group(const uint8_t *, const int *, const int nElem);
    int set_velocity_group(const uint8_t *, const short *, const int nElem);
    int set_torque_group(const uint8_t *, const short *, const int nElem);
    int set_position_velocity_group(const uint8_t *, const int *, const short *, const int nElem);
    int set_stiffness_damping_group(const uint8_t *, const int *, const int *, const int nElem);
    int set_gravity_compensation_group(const uint8_t *, const int *, const int nElem);

    int set_position_group(const group_ref_t &);
    int set_velocity_group(const group_ref_t &);
    int set_torque_group(const group_ref_t &);
    int set_position_velocity_group(const group_ref_comp_t &);
    int set_stiffness_damping_group(const group_ref_comp_t &);
    int set_gravity_compensation_group(const group_ref_t &);
#endif

protected:

    void factory_board(void);

    int set_param(int slave_pos, int index, int subindex, int size, void *data);
    int get_param(int slave_pos, int index, int subindex, int *size, void *data);

    SlavesMap slaves;

private:

    //YAML::Node doc;

    int             expected_wkc;
    ec_timing_t     timing;

    std::string     eth_if;

    uint64_t    sync_cycle_time_ns;
    uint64_t    sync_cycle_offset_ns;

    Write_XDDP_pipe * get_param_pipe;
    Read_XDDP_pipe *  set_param_pipe;

    void lookup_read(std::string token, float* data );
    
};


} 
}
}

#endif
