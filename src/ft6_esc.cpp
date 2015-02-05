#include <iit/ecat/advr/ft6_esc.h>
//#include <string>

using namespace iit::ecat;
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
static const char acName8001_3[] = "matrix_rn_c1";
static const char acName8001_4[] = "matrix_rn_c2";
static const char acName8001_5[] = "matrix_rn_c3";
static const char acName8001_6[] = "matrix_rn_c4";
static const char acName8001_7[] = "matrix_rn_c5";
static const char acName8001_8[] = "matrix_rn_c6";
static const char acName8001_9[] = "flash_params_cmd";
static const char acName8001_10[] = "flash_params_cmd_ack";



static const iit::ecat::objd_t source_SDOs[] =
{

    // SDO6000[] =
    {0x6000,    0x1, DTYPE_REAL32,      32, ATYPE_RO, acName6000_force0     ,0     },
    {0x6000,    0x2, DTYPE_REAL32,      32, ATYPE_RO, acName6000_force1     ,0     },
    {0x6000,    0x3, DTYPE_REAL32,      32, ATYPE_RO, acName6000_force2     ,0     },
    {0x6000,    0x4, DTYPE_REAL32,      32, ATYPE_RO, acName6000_force3     ,0     },
    {0x6000,    0x5, DTYPE_REAL32,      32, ATYPE_RO, acName6000_force4     ,0     },
    {0x6000,    0x6, DTYPE_REAL32,      32, ATYPE_RO, acName6000_force5     ,0     },
    {0x6000,    0x7, DTYPE_UNSIGNED16,  16, ATYPE_RO, acName6000_fault      ,0     },
    {0x6000,    0x8, DTYPE_UNSIGNED64,  64, ATYPE_RO, acName6000_rtt        ,0     },
    // SDO7000[] =                                                           
    {0x7000,    0x1, DTYPE_UNSIGNED64,  64, ATYPE_RW, acName7000_4          ,0     },
    // SDO8000[] =                                                           
    {0x8000,    0x1, DTYPE_INTEGER32,   32, ATYPE_RW, acName8000_1          ,0     },
    {0x8000,    0x2, DTYPE_INTEGER32,   32, ATYPE_RW, acName8000_2          ,0     },
    {0x8000,    0x3, DTYPE_INTEGER32,   32, ATYPE_RW, acName8000_3          ,0     },
    {0x8000,    0x4, DTYPE_INTEGER32,   32, ATYPE_RW, acName8000_4          ,0     },
    {0x8000,    0x5, DTYPE_INTEGER32,   32, ATYPE_RW, acName8000_5          ,0     },
    {0x8000,    0x6, DTYPE_INTEGER32,   32, ATYPE_RW, acName8000_6          ,0     },
    {0x8000,    0x7, DTYPE_INTEGER32,   32, ATYPE_RW, acName8000_7          ,0     },
    {0x8000,    0x8, DTYPE_INTEGER32,   32, ATYPE_RW, acName8000_8          ,0     },
    {0x8000,    0x9, DTYPE_REAL32,      32, ATYPE_RW, acName8000_9          ,0     },
    {0x8000,    0xa, DTYPE_REAL32,      32, ATYPE_RW, acName8000_10         ,0     },
    {0x8000,    0xb, DTYPE_REAL32,      32, ATYPE_RW, acName8000_11         ,0     },
    {0x8000,    0xc, DTYPE_REAL32,      32, ATYPE_RW, acName8000_12         ,0     },
    {0x8000,    0xd, DTYPE_REAL32,      32, ATYPE_RW, acName8000_13         ,0     },
    {0x8000,    0xe, DTYPE_REAL32,      32, ATYPE_RW, acName8000_14         ,0     },
    {0x8000,    0xf, DTYPE_REAL32,      32, ATYPE_RW, acName8000_15         ,0     },
    {0x8000,    0x10, DTYPE_REAL32,     32, ATYPE_RW, acName8000_16         ,0     },
    {0x8000,    0x11, DTYPE_REAL32,     32, ATYPE_RW, acName8000_17         ,0     },
    {0x8000,    0x12, DTYPE_REAL32,     32, ATYPE_RW, acName8000_18         ,0     },
    {0x8000,    0x13, DTYPE_REAL32,     32, ATYPE_RW, acName8000_19         ,0     },
    {0x8000,    0x14, DTYPE_REAL32,     32, ATYPE_RW, acName8000_20         ,0     },
    // SDO8001[] =                                                           
    {0x8001,    0x1, DTYPE_VISIBLE_STRING,   64, ATYPE_RO, acName8001_1     ,0     },
    {0x8001,    0x2, DTYPE_INTEGER16,        16, ATYPE_RW, acName8001_2     ,0     },
    {0x8001,    0x3, DTYPE_REAL32,           32, ATYPE_RW, acName8001_3     ,0     },
    {0x8001,    0x4, DTYPE_REAL32,           32, ATYPE_RW, acName8001_4     ,0     },
    {0x8001,    0x5, DTYPE_REAL32,           32, ATYPE_RW, acName8001_5     ,0     },
    {0x8001,    0x6, DTYPE_REAL32,           32, ATYPE_RW, acName8001_6     ,0     },
    {0x8001,    0x7, DTYPE_REAL32,           32, ATYPE_RW, acName8001_7     ,0     },
    {0x8001,    0x8, DTYPE_REAL32,           32, ATYPE_RW, acName8001_8     ,0     },
    {0x8001,    0x9, DTYPE_INTEGER16,        16, ATYPE_RW, acName8001_9     ,0     },
    {0x8001,    0xa, DTYPE_INTEGER16,        16, ATYPE_RO, acName8001_10    ,0     },
                                                                          
    {0, 0, 0, 0, 0, 0, 0 }                                                    
};                                                                        
                                                                          
                                                                          
                                                                          
void Ft6ESC::init_SDOs(void) {                                            
                                                                          
    int objd_num, i = 0;                                                  
                                                                          
    objd_num = sizeof(source_SDOs)/sizeof(objd_t);
    SDOs = new objd_t [objd_num];
                                                               
    memcpy((void*)SDOs, source_SDOs, sizeof(source_SDOs));                   

    // 0x6000
    SDOs[i++].data = (void*)&Ft6ESC::rx_pdo.force_X;    
    SDOs[i++].data = (void*)&Ft6ESC::rx_pdo.force_Y;    
    SDOs[i++].data = (void*)&Ft6ESC::rx_pdo.force_Z;    
    SDOs[i++].data = (void*)&Ft6ESC::rx_pdo.torque_X;   
    SDOs[i++].data = (void*)&Ft6ESC::rx_pdo.torque_Y;   
    SDOs[i++].data = (void*)&Ft6ESC::rx_pdo.torque_Z;   
    SDOs[i++].data = (void*)&Ft6ESC::rx_pdo.fault;      
    SDOs[i++].data = (void*)&Ft6ESC::rx_pdo.rtt;        
    // 0x7000 
    SDOs[i++].data = (void*)&Ft6ESC::tx_pdo.ts;         
    // 0x8000
    SDOs[i++].data = (void*)&Ft6ESC::sdo.Block_control;      
    SDOs[i++].data = (void*)&Ft6ESC::sdo.NumAvSamples;       
    SDOs[i++].data = (void*)&Ft6ESC::sdo.calibration_offset0;
    SDOs[i++].data = (void*)&Ft6ESC::sdo.calibration_offset1;
    SDOs[i++].data = (void*)&Ft6ESC::sdo.calibration_offset2;
    SDOs[i++].data = (void*)&Ft6ESC::sdo.calibration_offset3;
    SDOs[i++].data = (void*)&Ft6ESC::sdo.calibration_offset4;
    SDOs[i++].data = (void*)&Ft6ESC::sdo.calibration_offset5;
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_r1_c1;       
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_r1_c2;       
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_r1_c3;       
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_r1_c4;       
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_r1_c5;       
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_r1_c6;       
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_r2_c1;       
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_r2_c2;       
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_r2_c3;       
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_r2_c4;       
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_r2_c5;       
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_r2_c6;       
    // 0x8001
    SDOs[i++].data = (void*)&Ft6ESC::sdo.firmware_version;    
    SDOs[i++].data = (void*)&Ft6ESC::sdo.ack_board_fault;     
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_rn_c1;        
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_rn_c2;        
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_rn_c3;        
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_rn_c4;        
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_rn_c5;        
    SDOs[i++].data = (void*)&Ft6ESC::sdo.matrix_rn_c6;        
    SDOs[i++].data = (void*)&Ft6ESC::sdo.flash_params_cmd;    
    SDOs[i++].data = (void*)&Ft6ESC::sdo.flash_params_cmd_ack;
         
    SDOs[i++].data = 0;

    assert ( objd_num == i );
}
