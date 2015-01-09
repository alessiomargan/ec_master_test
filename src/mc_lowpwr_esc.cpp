#include <iit/ecat/advr/mc_lowpwr_esc.h>
#include <string>
using namespace iit::ecat::advr;

// LPtFlashParameters    LPESC::flash_param;
// LPtParameters         LPESC::param; 
// LPESCTypes::pdo_rx LPESC::sdo_rx_pdo;
// LPESCTypes::pdo_tx LPESC::sdo_tx_pdo;

char acName1000[] = "Device Type";
char acName1000_0[] = "Device Type";
char acName1008[] = "Manufacturer Device Name";
char acName1008_0[] = "Manufacturer Device Name";
char acName1009[] = "Manufacturer Hardware Version";
char acName1009_0[] = "Manufacturer Hardware Version";
char acName100A[] = "Manufacturer Software Version";
char acName100A_0[] = "Manufacturer Software Version";
char acName1018[] = "Identity Object";
char acName1018_0[] = "Number of Elements";
char acName1018_1[] = "Vendor ID";
char acName1018_2[] = "Product Code";
char acName1018_3[] = "Revision Number";
char acName1018_4[] = "Serial Number";
char acName1600[] = "Receive PDO Mapping";
char acName1600_0[] = "Number of Elements";
char acName1600_n[] = "Mapped Object";
char acName1A00[] = "Transmit PDO Mapping";
char acName1A00_0[] = "Number of Elements";
char acName1A00_n[] = "Mapped Object";
char acName1C00[] = "Sync Manager Communication Type";
char acName1C00_0[] = "Number of Elements";
char acName1C00_1[] = "Communications Type SM0";
char acName1C00_2[] = "Communications Type SM1";
char acName1C00_3[] = "Communications Type SM2";
char acName1C00_4[] = "Communications Type SM3";
char acName1C12[] = "Sync Manager 2 PDO Assignment";
char acName1C12_0[] = "Number of Elements";
char acName1C12_1[] = "PDO Mapping";
char acName1C13[] = "Sync Manager 3 PDO Assignment";
char acName1C13_0[] = "Number of Elements";
char acName1C13_1[] = "PDO Mapping";

char acName6000[] = "Inputs";
char acName6000_0[] = "Number of Elements";
char acName6000_1[] = "rel_enc";
char acName6000_2[] = "abs_enc";
char acName6000_3[] = "adc";
char acName6000_4[] = "ain";
char acName6000_5[] = "hall";
char acName6000_6[] = "pwm";
char acName6000_rtt[] = "rtt";
char acName6000_pid_out[] = "pid_out";
char acName6000_pos[] = "pos";
char acName6000_vel[] = "vel";
char acName6000_tor[] = "tor";
char acName6000_tor_D[] = "tor_D";
char acName6000_curr[] = "curr";
char acName6000_temp[] = "temperature";
char acName6000_fault[] = "fault";

char acName7000[] = "Outputs";
char acName7000_0[] = "Number of Elements";
char acName7000_1[] = "pos_ref";
char acName7000_2[] = "tor_ref";
char acName7000_3[] = "PGain";
char acName7000_4[] = "IGain";
char acName7000_5[] = "DGain";
char acName7000_6[] = "ts";


char acName8000[] = "Flash Parameter";
char acName8000_0[] = "Number of Elements";
char acName8000_1[] = "Block control";
char acName8000_2[] = "nonius offset low";
char acName8000_3[] = "pos_gain_P";
char acName8000_4[] = "pos_gain_I";
char acName8000_5[] = "pos_gain_D";
char acName8000_6[] = "tor_gain_P";
char acName8000_7[] = "tor_gain_I";
char acName8000_8[] = "tor_gain_D";
char acName8000_9[] = "Torque_Mult";
char acName8000_10[] = "pos_integral_limit";
char acName8000_11[] = "tor_integral_limit";
char acName8000_12[] = "min_pos";
char acName8000_13[] = "max_pos";
char acName8000_14[] = "nonius offset high";
char acName8000_15[] = "max_tor";
char acName8000_16[] = "max_cur";
char acName8000_17[] = "Enc_offset_1";
char acName8000_18[] = "Enc_offset_2";
char acName8000_19[] = "Torque_Offset";
char acName8000_20[] = "ConfigFlags";
char acName8000_21[] = "ConfigFlags2";
char acName8000_22[] = "NumEncoderLines";
char acName8000_23[] = "ImpedancePosGainP";
char acName8000_24[] = "nonius offset2 low";
char acName8000_25[] = "ImpedancePosGainD";
char acName8000_26[] = "Num_Abs_counts_rev";
char acName8000_27[] = "MaxPWM";
char acName8000_28[] = "Gearbox_ratio";
char acName8000_29[] = "ulCalPosition";
char acName8000_30[] = "Cal_Abs_Position";
char acName8000_31[] = "Cal_Abs2_Position";
char acName8000_32[] = "nonius offset2 high";


char acName8001[] = "Parameter";
char acName8001_1[] = "fw_ver";
char acName8001_2[] = "ack_board_faults";
char acName8001_3[] = "controller_set_status";
char acName8001_4[] = "controller_get_status";
char acName8001_5[] = "V_batt";
char acName8001_6[] = "T_inv";
char acName8001_7[] = "T_mot";
char acName8001_8[] = "flash_parameters_command";
char acName8001_9[] = "flash_parameters_command_ack";

const objd_t LPESC::SDOs[] =
{
    // SDO6000[] =
    {0x6000, 0x1, DTYPE_REAL32,      32, ATYPE_RO, acName6000_temp,    (void*)&LPESC::sdo_rx_pdo.max_temperature},
    {0x6000, 0x2, DTYPE_REAL32,      32, ATYPE_RO, acName6000_pos,     (void*)&LPESC::sdo_rx_pdo.position},
    {0x6000, 0x3, DTYPE_REAL32,      32, ATYPE_RO, acName6000_vel,     (void*)&LPESC::sdo_rx_pdo.velocity},
    {0x6000, 0x4, DTYPE_REAL32,      32, ATYPE_RO, acName6000_tor,     (void*)&LPESC::sdo_rx_pdo.torque},
    {0x6000, 0x5, DTYPE_UNSIGNED16,  16, ATYPE_RO, acName6000_fault,   (void*)&LPESC::sdo_rx_pdo.fault},
    {0x6000, 0x6, DTYPE_UNSIGNED64,  64, ATYPE_RO, acName6000_rtt,     (void*)&LPESC::sdo_rx_pdo.rtt},

    // SDO7000[] =
    {0x7000, 0x1, DTYPE_REAL32,     32, ATYPE_RW, acName7000_1, (void*)&LPESC::sdo_tx_pdo.pos_ref},
    {0x7000, 0x2, DTYPE_REAL32,     32, ATYPE_RW, acName7000_2, (void*)&LPESC::sdo_tx_pdo.tor_offs},
    {0x7000, 0x3, DTYPE_REAL32,     32, ATYPE_RW, acName7000_3, (void*)&LPESC::sdo_tx_pdo.PosGainP},
    {0x7000, 0x4, DTYPE_REAL32,     32, ATYPE_RW, acName7000_4, (void*)&LPESC::sdo_tx_pdo.PosGainI},
    {0x7000, 0x5, DTYPE_REAL32,     32, ATYPE_RW, acName7000_5, (void*)&LPESC::sdo_tx_pdo.PosGainD},
    {0x7000, 0x6, DTYPE_UNSIGNED64, 64, ATYPE_RW, acName7000_6, (void*)&LPESC::sdo_tx_pdo.ts},

    // SDO8000[] =
    {0x8000, 0x1, DTYPE_INTEGER32,     32, ATYPE_RW, acName8000_1, (void*)&LPESC::flash_param.Block_control},
    {0x8000, 0x2, DTYPE_INTEGER32,     32, ATYPE_RW, acName8000_2, (void*)&LPESC::flash_param.nonius_offset_low},
    {0x8000, 0x3, DTYPE_REAL32,        32, ATYPE_RW, acName8000_3, (void*)&LPESC::flash_param.PosGainP},
    {0x8000, 0x4, DTYPE_REAL32,        32, ATYPE_RW, acName8000_4, (void*)&LPESC::flash_param.PosGainI},
    {0x8000, 0x5, DTYPE_REAL32,        32, ATYPE_RW, acName8000_5, (void*)&LPESC::flash_param.PosGainD},
    {0x8000, 0x6, DTYPE_REAL32,        32, ATYPE_RW, acName8000_6, (void*)&LPESC::flash_param.TorGainP},
    {0x8000, 0x7, DTYPE_REAL32,        32, ATYPE_RW, acName8000_7, (void*)&LPESC::flash_param.TorGainI},
    {0x8000, 0x8, DTYPE_REAL32,        32, ATYPE_RW, acName8000_8, (void*)&LPESC::flash_param.TorGainD},
    {0x8000, 0x9, DTYPE_REAL32,        32, ATYPE_RW, acName8000_9, (void*)&LPESC::flash_param.Torque_Mult},
    {0x8000, 0xa, DTYPE_REAL32,        32, ATYPE_RW, acName8000_10, (void*)&LPESC::flash_param.Pos_I_lim},
    {0x8000, 0xb, DTYPE_REAL32,        32, ATYPE_RW, acName8000_11, (void*)&LPESC::flash_param.Tor_I_lim},
    {0x8000, 0xc, DTYPE_REAL32,        32, ATYPE_RW, acName8000_12, (void*)&LPESC::flash_param.Min_pos},
    {0x8000, 0xd, DTYPE_REAL32,        32, ATYPE_RW, acName8000_13, (void*)&LPESC::flash_param.Max_pos},
    {0x8000, 0xe, DTYPE_INTEGER32,     32, ATYPE_RW, acName8000_14, (void*)&LPESC::flash_param.nonius_offset_high},
    {0x8000, 0xf, DTYPE_REAL32,        32, ATYPE_RW, acName8000_15, (void*)&LPESC::flash_param.Max_tor},
    {0x8000, 0x10, DTYPE_REAL32,       32, ATYPE_RW, acName8000_16, (void*)&LPESC::flash_param.Max_cur},
    {0x8000, 0x11, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_17, (void*)&LPESC::flash_param.Enc_offset_1},
    {0x8000, 0x12, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_18, (void*)&LPESC::flash_param.Enc_offset_2},
    {0x8000, 0x13, DTYPE_REAL32,       32, ATYPE_RW, acName8000_19, (void*)&LPESC::flash_param.Torque_Offset},
    {0x8000, 0x14, DTYPE_INTEGER16,    16, ATYPE_RW, acName8000_20, (void*)&LPESC::flash_param.ConfigFlags},
    {0x8000, 0x15, DTYPE_INTEGER16,    16, ATYPE_RW, acName8000_21, (void*)&LPESC::flash_param.ConfigFlags2},
    {0x8000, 0x16, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_22, (void*)&LPESC::flash_param.NumEncoderLines},
    {0x8000, 0x17, DTYPE_REAL32,       32, ATYPE_RW, acName8000_23, (void*)&LPESC::flash_param.ImpedancePosGainP},
    {0x8000, 0x18, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_24, (void*)&LPESC::flash_param.nonius_offset2_low},
    {0x8000, 0x19, DTYPE_REAL32,       32, ATYPE_RW, acName8000_25, (void*)&LPESC::flash_param.ImpedancePosGainD},  
    {0x8000, 0x1a, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_26, (void*)&LPESC::flash_param.Num_Abs_counts_rev},
    {0x8000, 0x1b, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_27, (void*)&LPESC::flash_param.MaxPWM},
    {0x8000, 0x1c, DTYPE_REAL32,       32, ATYPE_RW, acName8000_28, (void*)&LPESC::flash_param.Gearbox_ratio},
    {0x8000, 0x1d, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_29, (void*)&LPESC::flash_param.ulCalPosition},
    {0x8000, 0x1e, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_30, (void*)&LPESC::flash_param.Cal_Abs_Position},
    {0x8000, 0x1f, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_31, (void*)&LPESC::flash_param.Cal_Abs2_Position},
    {0x8000, 0x20, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_32, (void*)&LPESC::flash_param.nonius_offset2_high},

    // SDO8001[] =
    {0x8001, 0x1, DTYPE_VISIBLE_STRING,   64, ATYPE_RO, acName8001_1, (void*)&LPESC::param.firmware_version},
    {0x8001, 0x2, DTYPE_INTEGER16,        16, ATYPE_RW, acName8001_2, (void*)&LPESC::param.ack_board_fault},
    {0x8001, 0x3, DTYPE_INTEGER16,        16, ATYPE_RW, acName8001_3, (void*)&LPESC::param.set_ctrl_status},
    {0x8001, 0x4, DTYPE_INTEGER16,        16, ATYPE_RO, acName8001_4, (void*)&LPESC::param.get_ctrl_status},
    {0x8001, 0x5, DTYPE_REAL32,           32, ATYPE_RO, acName8001_5, (void*)&LPESC::param.V_batt_filt_100ms},
    {0x8001, 0x6, DTYPE_REAL32,           32, ATYPE_RO, acName8001_6, (void*)&LPESC::param.T_inv_filt_100ms},
    {0x8001, 0x7, DTYPE_REAL32,           32, ATYPE_RO, acName8001_7, (void*)&LPESC::param.T_mot1_filt_100ms},
    {0x8001, 0x8, DTYPE_INTEGER16,        16, ATYPE_RW, acName8001_8, (void*)&LPESC::param.flash_params_cmd},
    {0x8001, 0x9, DTYPE_INTEGER16,        16, ATYPE_RO, acName8001_9, (void*)&LPESC::param.flash_params_cmd_ack},
    
    {0, 0, 0, 0, 0, 0}
};

const objd_t * LPESC::SDOs6000 = &LPESC::SDOs[0];   // #6
const objd_t * LPESC::SDOs7000 = &LPESC::SDOs[6];   // #6
const objd_t * LPESC::SDOs8000 = &LPESC::SDOs[12];  // #16
const objd_t * LPESC::SDOs8001 = &LPESC::SDOs[28];  // #14
