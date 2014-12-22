#ifndef __objectlist__
#define __objectlist__

#define DTYPE_UNSIGNED8 0
#define DTYPE_UNSIGNED16 1
#define DTYPE_INTEGER16 2
#define DTYPE_UNSIGNED64 3
#define DTYPE_REAL32 4
#define DTYPE_VISIBLE_STRING 5

#define ATYPE_RO 17
#define ATYPE_RW 18

typedef struct
{
   int subindex;
   int datatype;
   int bitlength;
   int access;
   int value;
   const char *data;
} objd;

objd SDO6000[] =
{
    {0x0, DTYPE_UNSIGNED8,   8, ATYPE_RO, 6, ""},
    {0x1, DTYPE_REAL32,      32, ATYPE_RO, 0, "position"},
    {0x2, DTYPE_REAL32,      32, ATYPE_RO, 0, "velocity"},
    {0x3, DTYPE_REAL32,      32, ATYPE_RO, 0, "torque"},
    {0x4, DTYPE_REAL32,      32, ATYPE_RO, 0, "max_temperature"},
    {0x5, DTYPE_UNSIGNED16,  16, ATYPE_RO, 0, "fault"},
    {0x6, DTYPE_UNSIGNED64,  64, ATYPE_RO, 0, "rtt"},
};

objd SDO7000[] =
{
    {0x0, DTYPE_UNSIGNED8,   8, ATYPE_RO, 6, ""},
    {0x1, DTYPE_REAL32,     32, ATYPE_RW, 0, "pos_ref"},
    {0x2, DTYPE_REAL32,     32, ATYPE_RW, 0, "tor_offs"},
    {0x3, DTYPE_REAL32,     32, ATYPE_RW, 0, "PosGainP"},
    {0x4, DTYPE_REAL32,     32, ATYPE_RW, 0, "PosGainI"},
    {0x5, DTYPE_REAL32,     32, ATYPE_RW, 0, "PosGainD"},
    {0x6, DTYPE_UNSIGNED64, 64, ATYPE_RW, 0, "ts"},
};

objd SDO8000[] =
{
    {0x0, DTYPE_UNSIGNED8, 8,  ATYPE_RO, 15, ""},
    {0x1, DTYPE_REAL32, 32, ATYPE_RW, 0, "Sensor_type"},
    {0x2, DTYPE_REAL32, 32, ATYPE_RW, 0, "TorGainP"},
    {0x3, DTYPE_REAL32, 32, ATYPE_RW, 0, "TorGainI"},
    {0x4, DTYPE_REAL32, 32, ATYPE_RW, 0, "TorGainD"},
    {0x5, DTYPE_REAL32, 32, ATYPE_RW, 0, "TorGainFF"},
    {0x6, DTYPE_REAL32, 32, ATYPE_RW, 0, "Pos_I_lim"},
    {0x7, DTYPE_REAL32, 32, ATYPE_RW, 0, "Tor_I_lim"},
    {0x8, DTYPE_REAL32, 32, ATYPE_RW, 0, "Min_pos"},
    {0x9, DTYPE_REAL32, 32, ATYPE_RW, 0, "Max_pos"},
    {0xa, DTYPE_REAL32, 32, ATYPE_RW, 0, "Max_vel"},
    {0xb, DTYPE_REAL32, 32, ATYPE_RW, 0, "Max_tor"},
    {0xc, DTYPE_REAL32, 32, ATYPE_RW, 0, "Max_cur"},
    {0xd, DTYPE_REAL32, 32, ATYPE_RO, 0, "Enc_offset"},
    {0xe, DTYPE_REAL32, 32, ATYPE_RO, 0, "Enc_relative_offset"},
    {0xf, DTYPE_REAL32, 32, ATYPE_RW, 0, "Phase_angle"},
};

objd SDO8001[] =
{
    {0x0, DTYPE_UNSIGNED8,      8,	    ATYPE_RO,	14, ""},
    {0x1, DTYPE_VISIBLE_STRING, 64,	    ATYPE_RO,	0, "firmware_version"},
    {0x2, DTYPE_INTEGER16,      16,	    ATYPE_RW,	0, "all"},
    {0x3, DTYPE_REAL32,  	    32,  	ATYPE_RW,	0, "Direct_ref"},
    {0x4, DTYPE_REAL32,  	    32,	    ATYPE_RO,  	0, "V_batt_filt_100ms"},
    {0x5, DTYPE_REAL32,  	    32,  	ATYPE_RO,	0, "Board_Temperature"},
    {0x6, DTYPE_REAL32,  	    32,  	ATYPE_RO,	0, "T_mot1_filt_100ms"},
    {0x7, DTYPE_INTEGER16,  	16,  	ATYPE_RW,	0, "ctrl_status_cmd"},
    {0x8, DTYPE_INTEGER16,  	16,  	ATYPE_RO,	0, "ctrl_status_cmd_ack"},
    {0x9, DTYPE_INTEGER16,  	16,  	ATYPE_RW,	0, "flash_params_cmd"},
    {0xa, DTYPE_INTEGER16,  	16,  	ATYPE_RO,	0, "flash_params_cmd_ack"},
    {0xb, DTYPE_REAL32,  	    32,  	ATYPE_RO,	0, "abs_enc_mot"},
    {0xc, DTYPE_REAL32,  	    32,  	ATYPE_RO,	0, "abs_enc_load"},
    {0xd, DTYPE_REAL32,         32, 	ATYPE_RO, 	0, "angle_enc_mot"},
    {0xe, DTYPE_REAL32, 	    32,	    ATYPE_RO,	0, "angle_enc_load"},
};

#endif
