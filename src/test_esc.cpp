#include <iit/ecat/advr/test_esc.h>
#include <string>

using namespace iit::ecat;
using namespace iit::ecat::advr;


static const iit::ecat::objd_t source_SDOs[] = {

    { 0x6000, 1, DTYPE_REAL32,      32, ATYPE_RO, "link_pos",       0},
    { 0x6000, 2, DTYPE_REAL32,      32, ATYPE_RO, "motor_pos",      0},
    { 0x6000, 3, DTYPE_REAL32,      32, ATYPE_RO, "link_vel",       0},
    { 0x6000, 4, DTYPE_INTEGER16,       16, ATYPE_RO, "motor_vel",      0},
    { 0x6000, 5, DTYPE_INTEGER16,       16, ATYPE_RO, "torque",     0},
    { 0x6000, 6, DTYPE_UNSIGNED16,      16, ATYPE_RO, "temperature",        0},
    { 0x6000, 7, DTYPE_UNSIGNED16,      16, ATYPE_RO, "fault",      0},
    { 0x6000, 8, DTYPE_UNSIGNED16,      16, ATYPE_RO, "rtt",        0},
    { 0x6000, 9, DTYPE_UNSIGNED16,      16, ATYPE_RO, "op_idx_ack",     0},
    { 0x6000, 10, DTYPE_REAL32,     32, ATYPE_RO, "aux",        0},

    { 0x7000, 1, DTYPE_REAL32,      32, ATYPE_RW, "pos_ref",        0},
    { 0x7000, 2, DTYPE_INTEGER16,       16, ATYPE_RW, "vel_ref",        0},
    { 0x7000, 3, DTYPE_INTEGER16,       16, ATYPE_RW, "tor_ref",        0},
    { 0x7000, 4, DTYPE_UNSIGNED16,      16, ATYPE_RW, "gain_kp_m",      0},
    { 0x7000, 5, DTYPE_UNSIGNED16,      16, ATYPE_RW, "gain_kp_l",      0},
    { 0x7000, 6, DTYPE_UNSIGNED16,      16, ATYPE_RW, "gain_kd_m",      0},
    { 0x7000, 7, DTYPE_UNSIGNED16,      16, ATYPE_RW, "gain_kd_l",      0},
    { 0x7000, 8, DTYPE_UNSIGNED16,      16, ATYPE_RW, "gain_ki",        0},
    { 0x7000, 9, DTYPE_UNSIGNED16,      16, ATYPE_RW, "fault_ack",      0},
    { 0x7000, 10, DTYPE_UNSIGNED16,     16, ATYPE_RW, "ts",     0},
    { 0x7000, 11, DTYPE_UNSIGNED16,     16, ATYPE_RW, "op_idx_ack",     0},
    { 0x7000, 12, DTYPE_REAL32,     32, ATYPE_RW, "aux",        0},

    { 0x8001, 1, DTYPE_VISIBLE_STRING,      64, ATYPE_RO, "fw_ver",     0},
    { 0x8001, 2, DTYPE_UNSIGNED32,      32, ATYPE_RW, "ack_board_faults",       0},
    { 0x8001, 3, DTYPE_UNSIGNED16,      16, ATYPE_RW, "ctrl_status_cmd",        0},
    { 0x8001, 4, DTYPE_UNSIGNED16,      16, ATYPE_RO, "ctrl_status_cmd_ack",        0},
    { 0x8001, 5, DTYPE_REAL32,      32, ATYPE_RW, "direct_ref",     0},
    { 0x8001, 6, DTYPE_REAL32,      32, ATYPE_RO, "abs_pos",        0},
    { 0x8001, 7, DTYPE_REAL32,      32, ATYPE_RO, "m_current",      0},
    { 0x8001, 8, DTYPE_UNSIGNED16,      16, ATYPE_RW, "flash_params_cmd",       0},
    { 0x8001, 9, DTYPE_UNSIGNED16,      16, ATYPE_RO, "flash_params_cmd_ack",       0},

    {0, 0, 0, 0, 0, 0, 0 }
};

void TestESC::init_SDOs(void) {                                         

    int objd_num, i = 0;                                              

    objd_num = sizeof(source_SDOs)/sizeof(objd_t);                    
    SDOs = new objd_t [objd_num];                                     

    memcpy((void*)SDOs, source_SDOs, sizeof(source_SDOs));

    //0x6000
    SDOs[i++].data = (void*)&TestESC::rx_pdo.link_pos;
    SDOs[i++].data = (void*)&TestESC::rx_pdo.motor_pos;
    SDOs[i++].data = (void*)&TestESC::rx_pdo.link_vel;
    SDOs[i++].data = (void*)&TestESC::rx_pdo.motor_vel;
    SDOs[i++].data = (void*)&TestESC::rx_pdo.torque;
    SDOs[i++].data = (void*)&TestESC::rx_pdo.temperature;
    SDOs[i++].data = (void*)&TestESC::rx_pdo.fault;
    SDOs[i++].data = (void*)&TestESC::rx_pdo.rtt;
    SDOs[i++].data = (void*)&TestESC::rx_pdo.op_idx_ack;
    SDOs[i++].data = (void*)&TestESC::rx_pdo.aux;

    //0x7000
    SDOs[i++].data = (void*)&TestESC::tx_pdo.pos_ref;
    SDOs[i++].data = (void*)&TestESC::tx_pdo.vel_ref;
    SDOs[i++].data = (void*)&TestESC::tx_pdo.tor_ref;
    SDOs[i++].data = (void*)&TestESC::tx_pdo.gain_kp_m;
    SDOs[i++].data = (void*)&TestESC::tx_pdo.gain_kp_l;
    SDOs[i++].data = (void*)&TestESC::tx_pdo.gain_kd_m;
    SDOs[i++].data = (void*)&TestESC::tx_pdo.gain_kd_l;
    SDOs[i++].data = (void*)&TestESC::tx_pdo.gain_ki;
    SDOs[i++].data = (void*)&TestESC::tx_pdo.fault_ack;
    SDOs[i++].data = (void*)&TestESC::tx_pdo.ts;
    SDOs[i++].data = (void*)&TestESC::tx_pdo.op_idx_aux;
    SDOs[i++].data = (void*)&TestESC::tx_pdo.aux;

    //0x8000

    //0x8001
    SDOs[i++].data = (void*)&TestESC::sdo.fw_ver;
    SDOs[i++].data = (void*)&TestESC::sdo.ack_board_faults;
    SDOs[i++].data = (void*)&TestESC::sdo.ctrl_status_cmd;
    SDOs[i++].data = (void*)&TestESC::sdo.ctrl_status_cmd_ack;
    SDOs[i++].data = (void*)&TestESC::sdo.direct_ref;
    SDOs[i++].data = (void*)&TestESC::sdo.abs_pos;
    SDOs[i++].data = (void*)&TestESC::sdo.m_current;
    SDOs[i++].data = (void*)&TestESC::sdo.flash_params_cmd;
    SDOs[i++].data = (void*)&TestESC::sdo.flash_params_cmd_ack;


    // end marker
    SDOs[i++].data = 0;

    assert ( objd_num == i );
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
