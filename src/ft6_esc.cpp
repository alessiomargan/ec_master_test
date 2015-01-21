#include <iit/ecat/advr/ft6_esc.h>
//#include <string>

using namespace iit::ecat::advr;

static const char acName6000[] = "Inputs";
static const char acName6000_0[] = "Number of Elements";
static const char acName6000_1[] = "rel_enc";
static const char acName6000_2[] = "abs_enc";
static const char acName6000_3[] = "adc";
static const char acName6000_4[] = "ain";
static const char acName6000_5[] = "hall";
static const char acName6000_6[] = "pwm";
static const char acName6000_rtt[] = "rtt";
static const char acName6000_pid_out[] = "pid_out";
static const char acName6000_pos[] = "pos";
static const char acName6000_vel[] = "vel";
static const char acName6000_tor[] = "tor";
static const char acName6000_tor_D[] = "tor_D";

static const char acName6000_force0[] = "ForceX";
static const char acName6000_force1[] = "ForceY";
static const char acName6000_force2[] = "ForceZ";
static const char acName6000_force3[] = "TorqueX";
static const char acName6000_force4[] = "TorqueY";
static const char acName6000_force5[] = "TorqueZ";

static const char acName6000_curr[] = "curr";
static const char acName6000_fault[] = "fault";

static const char acName7000[] = "Outputs";
static const char acName7000_0[] = "Number of Elements";
static const char acName7000_1[] = "pos_ref";
static const char acName7000_2[] = "tor_ref";
static const char acName7000_3[] = "direct_ref";
static const char acName7000_4[] = "ts";

static const char acName8000[] = "Flash Parameter";
static const char acName8000_0[] = "Number of Elements";
static const char acName8000_1[] = "Block control";
static const char acName8000_2[] = "Num Av Samples";
static const char acName8000_3[] = "Cal offset 0";
static const char acName8000_4[] = "Cal offset 1";
static const char acName8000_5[] = "Cal offset 2";
static const char acName8000_6[] = "Cal offset 3";
static const char acName8000_7[] = "Cal offset 4";
static const char acName8000_8[] = "Cal offset 5";
static const char acName8000_9[] = "matrix_r1_c1";
static const char acName8000_10[] = "matrix_r1_c2";
static const char acName8000_11[] = "matrix_r1_c3";
static const char acName8000_12[] = "matrix_r1_c4";
static const char acName8000_13[] = "matrix_r1_c5";
static const char acName8000_14[] = "matrix_r1_c6";
static const char acName8000_15[] = "matrix_r2_c1";
static const char acName8000_16[] = "matrix_r2_c2";
static const char acName8000_17[] = "matrix_r2_c3";
static const char acName8000_18[] = "matrix_r2_c4";
static const char acName8000_19[] = "matrix_r2_c5";
static const char acName8000_20[] = "matrix_r2_c6";

static const char acName8001[] = "Parameter";
static const char acName8001_1[] = "fw_ver";
static const char acName8001_2[] = "ack_board_faults";
static const char acName8001_3[] = "Matrix c1";
static const char acName8001_4[] = "Matrix c2";
static const char acName8001_5[] = "Matrix c3";
static const char acName8001_6[] = "Matrix c4";
static const char acName8001_7[] = "Matrix c5";
static const char acName8001_8[] = "Matrix c6";
static const char acName8001_9[] = "flash_parameters_command";
static const char acName8001_10[] = "flash_parameters_command_ack";


FT6_tFlashParameters    FtESC::flash_param;
FT6_tParameters         FtESC::param;
FtESCTypes::pdo_rx      FtESC::sdo_rx_pdo;
FtESCTypes::pdo_tx      FtESC::sdo_tx_pdo;

const objd_t FtESC::SDOs[] =
{
    // SDO6000[] =
    {0x6000,    0x1, DTYPE_REAL32,      32, ATYPE_RO, acName6000_force0,     (void*)&FtESC::sdo_rx_pdo.force_X},
    {0x6000,    0x2, DTYPE_REAL32,      32, ATYPE_RO, acName6000_force1,     (void*)&FtESC::sdo_rx_pdo.force_Y},
    {0x6000,    0x3, DTYPE_REAL32,      32, ATYPE_RO, acName6000_force2,     (void*)&FtESC::sdo_rx_pdo.force_Z},
    {0x6000,    0x4, DTYPE_REAL32,      32, ATYPE_RO, acName6000_force3,     (void*)&FtESC::sdo_rx_pdo.torque_X},
    {0x6000,    0x5, DTYPE_REAL32,      32, ATYPE_RO, acName6000_force4,     (void*)&FtESC::sdo_rx_pdo.torque_Y},
    {0x6000,    0x6, DTYPE_REAL32,      32, ATYPE_RO, acName6000_force5,     (void*)&FtESC::sdo_rx_pdo.torque_Z},
    {0x6000,    0x7, DTYPE_UNSIGNED16,  16, ATYPE_RO, acName6000_fault,      (void*)&FtESC::sdo_rx_pdo.fault},
    {0x6000,    0x8, DTYPE_UNSIGNED64,  64, ATYPE_RO, acName6000_rtt,        (void*)&FtESC::sdo_rx_pdo.rtt},
    // SDO7000[] =                                                           
    {0x7000,    0x1, DTYPE_UNSIGNED64,  64, ATYPE_RW, acName7000_4,          (void*)&FtESC::sdo_tx_pdo.ts},
    // SDO8000[] =                                                           
    {0x8000,    0x1, DTYPE_INTEGER32,   32, ATYPE_RW, acName8000_1,          (void*)&FtESC::flash_param.Block_control},
    {0x8000,    0x2, DTYPE_INTEGER32,   32, ATYPE_RW, acName8000_2,          (void*)&FtESC::flash_param.NumAvSamples},
    {0x8000,    0x3, DTYPE_INTEGER32,   32, ATYPE_RW, acName8000_3,          (void*)&FtESC::flash_param.calibration_offset0},
    {0x8000,    0x4, DTYPE_INTEGER32,   32, ATYPE_RW, acName8000_4,          (void*)&FtESC::flash_param.calibration_offset1},
    {0x8000,    0x5, DTYPE_INTEGER32,   32, ATYPE_RW, acName8000_5,          (void*)&FtESC::flash_param.calibration_offset2},
    {0x8000,    0x6, DTYPE_INTEGER32,   32, ATYPE_RW, acName8000_6,          (void*)&FtESC::flash_param.calibration_offset3},
    {0x8000,    0x7, DTYPE_INTEGER32,   32, ATYPE_RW, acName8000_7,          (void*)&FtESC::flash_param.calibration_offset4},
    {0x8000,    0x8, DTYPE_INTEGER32,   32, ATYPE_RW, acName8000_8,          (void*)&FtESC::flash_param.calibration_offset5},
    {0x8000,    0x9, DTYPE_REAL32,      32, ATYPE_RW, acName8000_9,          (void*)&FtESC::flash_param.matrix_r1_c1},
    {0x8000,    0xa, DTYPE_REAL32,      32, ATYPE_RW, acName8000_10,         (void*)&FtESC::flash_param.matrix_r1_c2},
    {0x8000,    0xb, DTYPE_REAL32,      32, ATYPE_RW, acName8000_11,         (void*)&FtESC::flash_param.matrix_r1_c3},
    {0x8000,    0xc, DTYPE_REAL32,      32, ATYPE_RW, acName8000_12,         (void*)&FtESC::flash_param.matrix_r1_c4},
    {0x8000,    0xd, DTYPE_REAL32,      32, ATYPE_RW, acName8000_13,         (void*)&FtESC::flash_param.matrix_r1_c5},
    {0x8000,    0xe, DTYPE_REAL32,      32, ATYPE_RW, acName8000_14,         (void*)&FtESC::flash_param.matrix_r1_c6},
    {0x8000,    0xf, DTYPE_REAL32,      32, ATYPE_RW, acName8000_15,         (void*)&FtESC::flash_param.matrix_r2_c1},
    {0x8000,    0x10, DTYPE_REAL32,     32, ATYPE_RW, acName8000_16,         (void*)&FtESC::flash_param.matrix_r2_c2},
    {0x8000,    0x11, DTYPE_REAL32,     32, ATYPE_RW, acName8000_17,         (void*)&FtESC::flash_param.matrix_r2_c3},
    {0x8000,    0x12, DTYPE_REAL32,     32, ATYPE_RW, acName8000_18,         (void*)&FtESC::flash_param.matrix_r2_c4},
    {0x8000,    0x13, DTYPE_REAL32,     32, ATYPE_RW, acName8000_19,         (void*)&FtESC::flash_param.matrix_r2_c5},
    {0x8000,    0x14, DTYPE_REAL32,     32, ATYPE_RW, acName8000_20,         (void*)&FtESC::flash_param.matrix_r2_c6},
    // SDO8001[] =                                                           
    {0x8001,    0x1, DTYPE_VISIBLE_STRING,   64, ATYPE_RO, acName8001_1,     (void*)&FtESC::param.firmware_version},
    {0x8001,    0x2, DTYPE_INTEGER16,        16, ATYPE_RW, acName8001_2,     (void*)&FtESC::param.ack_board_fault},
    {0x8001,    0x3, DTYPE_REAL32,           32, ATYPE_RW, acName8001_3,     (void*)&FtESC::param.matrix_rn_c1},
    {0x8001,    0x4, DTYPE_REAL32,           32, ATYPE_RW, acName8001_4,     (void*)&FtESC::param.matrix_rn_c2},
    {0x8001,    0x5, DTYPE_REAL32,           32, ATYPE_RW, acName8001_5,     (void*)&FtESC::param.matrix_rn_c3},
    {0x8001,    0x6, DTYPE_REAL32,           32, ATYPE_RW, acName8001_6,     (void*)&FtESC::param.matrix_rn_c4},
    {0x8001,    0x7, DTYPE_REAL32,           32, ATYPE_RW, acName8001_7,     (void*)&FtESC::param.matrix_rn_c5},
    {0x8001,    0x8, DTYPE_REAL32,           32, ATYPE_RW, acName8001_8,     (void*)&FtESC::param.matrix_rn_c6},
    {0x8001,    0x9, DTYPE_INTEGER16,        16, ATYPE_RW, acName8001_9,     (void*)&FtESC::param.flash_params_cmd},
    {0x8001,    0xa, DTYPE_INTEGER16,        16, ATYPE_RO, acName8001_10,    (void*)&FtESC::param.flash_params_cmd_ack},

    {0, 0, 0, 0, 0, 0}
};

const objd_t * FtESC::SDOs6000 = &FtESC::SDOs[0];   // #8
const objd_t * FtESC::SDOs7000 = &FtESC::SDOs[8];   // #1
const objd_t * FtESC::SDOs8000 = &FtESC::SDOs[9];   // #20
const objd_t * FtESC::SDOs8001 = &FtESC::SDOs[29];  // #10

