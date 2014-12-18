#ifndef __objectlist__
#define __objectlist__

#define FLASHSTORE 

typedef FLASHSTORE struct PACKED
{
    uint16 subindex;
    uint16 datatype;
    uint16 bitlength;
    uint16 access;
    char FLASHSTORE *name;
    uint32 value;
    void *data;
} _objd;

typedef FLASHSTORE struct PACKED
{
    uint16 index;
    uint16 objtype;
    uint8 maxsub;
    uint8 pad1;
    char FLASHSTORE *name;
    _objd FLASHSTORE *objdesc;
} _objectlist;

#define _ac FLASHSTORE char
#define nil 0

#define OTYPE_DOMAIN            0x0002
#define OTYPE_DEFTYPE           0x0005
#define OTYPE_DEFSTRUCT         0x0006
#define OTYPE_VAR               0x0007
#define OTYPE_ARRAY             0x0008
#define OTYPE_RECORD            0x0009

#define DTYPE_BOOLEAN           0x0001
#define DTYPE_INTEGER8          0x0002
#define DTYPE_INTEGER16         0x0003
#define DTYPE_INTEGER32         0x0004
#define DTYPE_UNSIGNED8         0x0005
#define DTYPE_UNSIGNED16        0x0006
#define DTYPE_UNSIGNED32        0x0007
#define DTYPE_REAL32            0x0008
#define DTYPE_VISIBLE_STRING    0x0009
#define DTYPE_OCTET_STRING      0x000A
#define DTYPE_UNICODE_STRING    0x000B
#define DTYPE_INTEGER24         0x0010
#define DTYPE_UNSIGNED24        0x0016
#define DTYPE_INTEGER64         0x0015
#define DTYPE_UNSIGNED64        0x001B
#define DTYPE_REAL64            0x0011
#define DTYPE_PDO_MAPPING       0x0021
#define DTYPE_IDENTITY          0x0023
#define DTYPE_BIT1              0x0030
#define DTYPE_BIT2              0x0031
#define DTYPE_BIT3              0x0032
#define DTYPE_BIT4              0x0033
#define DTYPE_BIT5              0x0034
#define DTYPE_BIT6              0x0035
#define DTYPE_BIT7              0x0036
#define DTYPE_BIT8              0x0037

#define ATYPE_RO                0x07
#define ATYPE_RW                0x3F
#define ATYPE_RWpre             0x0F
#define ATYPE_RXPDO             0x40
#define ATYPE_TXPDO             0x80

_ac acName1000[] = "Device Type";
_ac acName1000_0[] = "Device Type";
_ac acName1008[] = "Manufacturer Device Name";
_ac acName1008_0[] = "Manufacturer Device Name";
_ac acName1009[] = "Manufacturer Hardware Version";
_ac acName1009_0[] = "Manufacturer Hardware Version";
_ac acName100A[] = "Manufacturer Software Version";
_ac acName100A_0[] = "Manufacturer Software Version";
_ac acName1018[] = "Identity Object";
_ac acName1018_0[] = "Number of Elements";
_ac acName1018_1[] = "Vendor ID";
_ac acName1018_2[] = "Product Code";
_ac acName1018_3[] = "Revision Number";
_ac acName1018_4[] = "Serial Number";
_ac acName1600[] = "Receive PDO Mapping";
_ac acName1600_0[] = "Number of Elements";
_ac acName1600_n[] = "Mapped Object";
_ac acName1A00[] = "Transmit PDO Mapping";
_ac acName1A00_0[] = "Number of Elements";
_ac acName1A00_n[] = "Mapped Object";
_ac acName1C00[] = "Sync Manager Communication Type";
_ac acName1C00_0[] = "Number of Elements";
_ac acName1C00_1[] = "Communications Type SM0";
_ac acName1C00_2[] = "Communications Type SM1";
_ac acName1C00_3[] = "Communications Type SM2";
_ac acName1C00_4[] = "Communications Type SM3";
_ac acName1C12[] = "Sync Manager 2 PDO Assignment";
_ac acName1C12_0[] = "Number of Elements";
_ac acName1C12_1[] = "PDO Mapping";
_ac acName1C13[] = "Sync Manager 3 PDO Assignment";
_ac acName1C13_0[] = "Number of Elements";
_ac acName1C13_1[] = "PDO Mapping";

_ac acName6000[] = "Inputs";
_ac acName6000_0[] = "Number of Elements";
_ac acName6000_1[] = "link_pos";
_ac acName6000_2[] = "link_speed";
_ac acName6000_3[] = "link_tor";
_ac acName6000_4[] = "temperature";
_ac acName6000_5[] = "fault";
_ac acName6000_6[] = "rtt";

_ac acName7000[] = "Outputs";
_ac acName7000_0[] = "Number of Elements";
_ac acName7000_1[] = "pos_ref";
_ac acName7000_2[] = "tor_offset";
_ac acName7000_3[] = "pos_gain_P";
_ac acName7000_4[] = "pos_gain_I";
_ac acName7000_5[] = "pos_gain_D";
_ac acName7000_6[] = "ts";

_ac acName8000[] = "Flash Parameter";
_ac acName8000_0[] = "Number of Elements";
_ac acName8000_1[] = "abs_enc_type";
_ac acName8000_2[] = "tor_gain_P";
_ac acName8000_3[] = "tor_gain_I";
_ac acName8000_4[] = "tor_gain_D";
_ac acName8000_5[] = "tor_gain_FF";
_ac acName8000_6[] = "pos_integral_limit";
_ac acName8000_7[] = "tor_integral_limit";
_ac acName8000_8[] = "min_pos";
_ac acName8000_9[] = "max_pos";
_ac acName8000_10[] = "max_vel";
_ac acName8000_11[] = "max_tor";
_ac acName8000_12[] = "max_cur";
_ac acName8000_13[] = "Enc_offset";
_ac acName8000_14[] = "Enc_relative_offset";
_ac acName8000_15[] = "Phase_angle";

_ac acName8001[] = "Parameter";
_ac acName8001_1[] = "fw_ver";
_ac acName8001_2[] = "ack_board_faults";
_ac acName8001_3[] = "controller_set_status";
_ac acName8001_4[] = "controller_get_status";
_ac acName8001_5[] = "V_batt";
_ac acName8001_6[] = "T_inv";
_ac acName8001_7[] = "T_mot";
_ac acName8001_8[] = "flash_parameters_command";
_ac acName8001_9[] = "flash_parameters_command_ack";

FLASHSTORE _objd SDO8000[] =
{
    {0x0, DTYPE_UNSIGNED8,     8,  ATYPE_RO,    acName8000_0,  15, nil},
    {0x1, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_1,  0, "Sensor_type"},
    {0x2, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_2,  0, "TorGainP"},
    {0x3, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_3,  0, "TorGainI"},
    {0x4, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_4,  0, "TorGainD"},
    {0x5, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_5,  0, "TorGainFF"},
    {0x6, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_6,  0, "Pos_I_lim"},
    {0x7, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_7,  0, "Tor_I_lim"},
    {0x8, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_8,  0, "Min_pos"},
    {0x9, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_9,  0, "Max_pos"},
    {0xa, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_10, 0, "Max_vel"},
    {0xb, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_11, 0, "Max_tor"},
    {0xc, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_12, 0, "Max_cur"},
    {0xd, DTYPE_REAL32,            32, ATYPE_RW,    acName8000_13, 0, "Enc_offset"},
    {0xe, DTYPE_REAL32,            32, ATYPE_RW,    acName8000_14, 0, "Enc_relative_offset"},
    {0xf, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_15, 0, "Phase_angle"},
};

FLASHSTORE _objd SDO8001[] =
{
    {0x0, DTYPE_UNSIGNED8,        8,  ATYPE_RO, acName8000_0, 9, nil},
    {0x1, DTYPE_VISIBLE_STRING,   64, ATYPE_RO, acName8001_1, 0, "firmware_version"},
    
    {0x2, DTYPE_INTEGER16,        16, ATYPE_RW, acName8001_2, 0, "ack_board_fault.all"},
    {0x3, DTYPE_INTEGER16,        16, ATYPE_RW, acName8001_3, 0, "set_ctrl_status.all"},
    
    {0x4, DTYPE_INTEGER16,        16, ATYPE_RO, acName8001_4, 0, "get_ctrl_status.all"},
    {0x5, DTYPE_REAL32,           32, ATYPE_RO, acName8001_5, 0, "filters.V_batt_filt_100ms"},
    {0x6, DTYPE_REAL32,           32, ATYPE_RO, acName8001_6, 0, "Board_Temperature"},
    {0x7, DTYPE_REAL32,           32, ATYPE_RO, acName8001_7, 0, "filters.T_mot1_filt_100ms"},
    
    {0x8, DTYPE_INTEGER16,        16, ATYPE_RW, acName8001_8, 0, "flash_params_cmd"},
    {0x9, DTYPE_INTEGER16,        16, ATYPE_RO, acName8001_9, 0, "flash_params_cmd_ack"},
};


#endif
