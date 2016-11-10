#include <iit/ecat/advr/mc_centAC_esc.h>
#include <string>

using namespace iit::ecat::advr;
using namespace iit::ecat;


static const iit::ecat::objd_t source_SDOs[] = {

    // SD0 0x6000
    { 0x6000, 1, DTYPE_REAL32,          32, ATYPE_RO, "link_pos",   0},
    { 0x6000, 2, DTYPE_REAL32,          32, ATYPE_RO, "motor_pos",  0},
    { 0x6000, 3, DTYPE_REAL32,          32, ATYPE_RO, "link_vel",   0},
#ifdef FLOAT_PDO
    { 0x6000, 4, DTYPE_REAL32,       16, ATYPE_RO, "motor_vel",      0},
    { 0x6000, 5, DTYPE_REAL32,       16, ATYPE_RO, "torque",     0},
#else
    { 0x6000, 4, DTYPE_INTEGER16,       16, ATYPE_RO, "motor_vel",  0},
    { 0x6000, 5, DTYPE_INTEGER16,       16, ATYPE_RO, "torque",     0},
#endif
    { 0x6000, 6, DTYPE_UNSIGNED16,      16, ATYPE_RO, "temperature",0},
    { 0x6000, 7, DTYPE_UNSIGNED16,      16, ATYPE_RO, "fault",      0},
    { 0x6000, 8, DTYPE_UNSIGNED16,      16, ATYPE_RO, "rtt",        0},
    { 0x6000, 9, DTYPE_UNSIGNED16,      16, ATYPE_RO, "op_idx_ack", 0},
    { 0x6000, 10, DTYPE_REAL32,         32, ATYPE_RO, "aux",        0},

    // SD0 0x7000
    { 0x7000, 1, DTYPE_REAL32,          32, ATYPE_RW, "pos_ref",    0},
#if FLOAT_PDO
    { 0x7000, 2, DTYPE_REAL32,          32, ATYPE_RW, "vel_ref",    0},
    { 0x7000, 3, DTYPE_REAL32,          32, ATYPE_RW, "tor_ref",    0},
    { 0x7000, 4, DTYPE_REAL32,          32, ATYPE_RW, "gain_0",     0},
    { 0x7000, 5, DTYPE_REAL32,          32, ATYPE_RW, "gain_1",     0},
    { 0x7000, 6, DTYPE_REAL32,          32, ATYPE_RW, "gain_2",     0},
    { 0x7000, 7, DTYPE_REAL32,          32, ATYPE_RW, "gain_3",     0},
    { 0x7000, 8, DTYPE_REAL32,          32, ATYPE_RW, "gain_4",     0},
#else
    { 0x7000, 2, DTYPE_INTEGER16,       16, ATYPE_RW, "vel_ref",    0},
    { 0x7000, 3, DTYPE_INTEGER16,       16, ATYPE_RW, "tor_ref",    0},
    { 0x7000, 4, DTYPE_UNSIGNED16,      16, ATYPE_RW, "gain_0",     0},
    { 0x7000, 5, DTYPE_UNSIGNED16,      16, ATYPE_RW, "gain_1",     0},
    { 0x7000, 6, DTYPE_UNSIGNED16,      16, ATYPE_RW, "gain_2",     0},
    { 0x7000, 7, DTYPE_UNSIGNED16,      16, ATYPE_RW, "gain_3",     0},
    { 0x7000, 8, DTYPE_UNSIGNED16,      16, ATYPE_RW, "gain_4",     0},
#endif
    { 0x7000, 9, DTYPE_UNSIGNED16,      16, ATYPE_RW, "fault_ack",  0},
    { 0x7000, 10, DTYPE_UNSIGNED16,     16, ATYPE_RW, "ts",         0},
    { 0x7000, 11, DTYPE_UNSIGNED16,     16, ATYPE_RW, "op_idx_aux", 0},
    { 0x7000, 12, DTYPE_REAL32,         32, ATYPE_RW, "aux",        0},

    // SD0 0x8000
    { 0x8000, 0x1,  DTYPE_UNSIGNED16,  16, ATYPE_RW,    "Hardware_config",      0},
    { 0x8000, 0x2,  DTYPE_UNSIGNED16,  16, ATYPE_RW,    "Motor_gear_ratio",     0},
    { 0x8000, 0x3,  DTYPE_REAL32,      32, ATYPE_RW,    "Motor_el_ph_angle",    0},
    { 0x8000, 0x4,  DTYPE_REAL32,      32, ATYPE_RW,    "Torsion_bar_stiff",    0},
    { 0x8000, 0x5,  DTYPE_REAL32,      32, ATYPE_RW,    "CurrGainP",            0},
    { 0x8000, 0x6,  DTYPE_REAL32,      32, ATYPE_RW,    "CurrGainI",            0},
    { 0x8000, 0x7,  DTYPE_REAL32,      32, ATYPE_RW,    "Max_cur",              0},
    { 0x8000, 0x8,  DTYPE_REAL32,      32, ATYPE_RW,    "Max_tor",              0},
    { 0x8000, 0x9,  DTYPE_REAL32,      32, ATYPE_RW,    "Max_vel",              0},
    { 0x8000, 0xa,  DTYPE_REAL32,      32, ATYPE_RW,    "Min_pos",              0},
    { 0x8000, 0xb,  DTYPE_REAL32,      32, ATYPE_RW,    "Max_pos",              0},
    { 0x8000, 0xc,  DTYPE_REAL32,      32, ATYPE_RW,    "Calibration_angle",    0},
    { 0x8000, 0xd,  DTYPE_REAL32,      32, ATYPE_RW,    "Enc_offset",           0},
    { 0x8000, 0xe,  DTYPE_INTEGER16,   16, ATYPE_RW,    "Serial_Number_A",      0},
    { 0x8000, 0xf,  DTYPE_INTEGER16,   16, ATYPE_RW,    "Joint_robot_id",       0},
    

    // SD0 0x8001
    { 0x8001, 0x1, DTYPE_VISIBLE_STRING,   64, ATYPE_RO, "m3_fw_ver" ,          0},
    { 0x8001, 0x2, DTYPE_VISIBLE_STRING,   64, ATYPE_RO, "c28_fw_ver",          0},
    { 0x8001, 0x3, DTYPE_UNSIGNED16,       16, ATYPE_RW, "ctrl_status_cmd"  ,   0},
    { 0x8001, 0x4, DTYPE_UNSIGNED16,       16, ATYPE_RO, "ctrl_status_cmd_ack", 0},
    { 0x8001, 0x5, DTYPE_UNSIGNED16,       16, ATYPE_RW, "flash_params_cmd",    0},
    { 0x8001, 0x6, DTYPE_UNSIGNED16,       16, ATYPE_RO, "flash_params_cmd_ack",0},
    { 0x8001, 0x7, DTYPE_REAL32,           32, ATYPE_RW, "Direct_ref",          0},
    { 0x8001, 0x8, DTYPE_UNSIGNED16,       16, ATYPE_RO, "fault",               0},
    { 0x8001, 0x9, DTYPE_REAL32,           32, ATYPE_RO, "v_batt",              0},
    { 0x8001, 0xa, DTYPE_REAL32,           32, ATYPE_RO, "i_batt",              0},
    { 0x8001, 0xb, DTYPE_REAL32,           32, ATYPE_RO, "torque_read",         0},
    { 0x8001, 0xc, DTYPE_REAL32,           32, ATYPE_RO, "board_temp",          0},
    { 0x8001, 0xd, DTYPE_REAL32,           32, ATYPE_RO, "motor_temp",          0},

    // SD0 0x8002
    { 0x8002, 1, DTYPE_REAL32,      32, ATYPE_RO, "pos_ref_fb",                 0},
    { 0x8002, 2, DTYPE_REAL32,      32, ATYPE_RO, "iq_ref_fb",                  0},
    { 0x8002, 3, DTYPE_REAL32,      32, ATYPE_RO, "iq_out_fb",                  0},

    {0, 0, 0, 0, 0, 0, 0 }


};


void CentAcESC::init_SDOs ( void ) {

    int objd_num, i = 0;

    objd_num = sizeof ( source_SDOs ) /sizeof ( objd_t );
    SDOs = new objd_t [objd_num];

    memcpy ( ( void* ) SDOs, source_SDOs, sizeof ( source_SDOs ) );

    //0x6000
    SDOs[i++].data = (void*)&CentAcESC::rx_pdo.link_pos;
    SDOs[i++].data = (void*)&CentAcESC::rx_pdo.motor_pos;
    SDOs[i++].data = (void*)&CentAcESC::rx_pdo.link_vel;
    SDOs[i++].data = (void*)&CentAcESC::rx_pdo.motor_vel;
    SDOs[i++].data = (void*)&CentAcESC::rx_pdo.torque;
    SDOs[i++].data = (void*)&CentAcESC::rx_pdo.temperature;
    SDOs[i++].data = (void*)&CentAcESC::rx_pdo.fault;
    SDOs[i++].data = (void*)&CentAcESC::rx_pdo.rtt;
    SDOs[i++].data = (void*)&CentAcESC::rx_pdo.op_idx_ack;
    SDOs[i++].data = (void*)&CentAcESC::rx_pdo.aux;

    //0x7000
    SDOs[i++].data = (void*)&CentAcESC::tx_pdo.pos_ref;
    SDOs[i++].data = (void*)&CentAcESC::tx_pdo.vel_ref;
    SDOs[i++].data = (void*)&CentAcESC::tx_pdo.tor_ref;
    SDOs[i++].data = (void*)&CentAcESC::tx_pdo.gain_0;
    SDOs[i++].data = (void*)&CentAcESC::tx_pdo.gain_1;
    SDOs[i++].data = (void*)&CentAcESC::tx_pdo.gain_2;
    SDOs[i++].data = (void*)&CentAcESC::tx_pdo.gain_3;
    SDOs[i++].data = (void*)&CentAcESC::tx_pdo.gain_4;
    SDOs[i++].data = (void*)&CentAcESC::tx_pdo.fault_ack;
    SDOs[i++].data = (void*)&CentAcESC::tx_pdo.ts;
    SDOs[i++].data = (void*)&CentAcESC::tx_pdo.op_idx_aux;
    SDOs[i++].data = (void*)&CentAcESC::tx_pdo.aux;
    
    // 0x8000
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.Hardware_configuration;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.Motor_gear_ratio;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.Motor_electrical_phase_angle;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.Torsion_bar_stiffness;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.CurrGainP;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.CurrGainI;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.Max_cur;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.Max_tor;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.Max_vel;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.Joint_Min_pos;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.Joint_Max_pos;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.Calibration_angle;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.Enc_offset;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.Serial_number_A;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.Joint_robot_id;
    
    // 0x8001
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.m3_fw_ver;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.c28_fw_ver;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.ctrl_status_cmd;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.ctrl_status_cmd_ack;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.flash_params_cmd;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.flash_params_cmd_ack;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.iq_ref;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.fault;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.v_batt;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.i_batt;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.torque_read;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.board_temp;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.motor_temp;
    
    // 0x8002
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.pos_ref_fb;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.iq_ref_fb;
    SDOs[i++].data = ( void* ) &CentAcESC::sdo.iq_out_fb;
    
    // end marker
    SDOs[i++].data = 0;

    assert ( objd_num == i );
}
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
