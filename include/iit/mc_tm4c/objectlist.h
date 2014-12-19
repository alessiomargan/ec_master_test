#ifndef __objectlist__
#define __objectlist__


typedef struct
{
    int subindex;
    int bitlength;
    int value;
    const char *data;
} objd;

objd SDO8000[] =
{
    {0x0, 8, 15, ""},
    {0x1, 32, 0, "Sensor_type"},
    {0x2, 32, 0, "TorGainP"},
    {0x3, 32, 0, "TorGainI"},
    {0x4, 32, 0, "TorGainD"},
    {0x5, 32, 0, "TorGainFF"},
    {0x6, 32, 0, "Pos_I_lim"},
    {0x7, 32, 0, "Tor_I_lim"},
    {0x8, 32, 0, "Min_pos"},
    {0x9, 32, 0, "Max_pos"},
    {0xa, 32, 0, "Max_vel"},
    {0xb, 32, 0, "Max_tor"},
    {0xc, 32, 0, "Max_cur"},
    {0xd, 32, 0, "Enc_offset"},
    {0xe, 32, 0, "Enc_relative_offset"},
    {0xf, 32, 0, "Phase_angle"},
};

objd SDO8001[] =
{
    {0x0, 8,  10, ""},
    {0x1, 64, 0, "firmware_version"},
    {0x2, 16, 0, "all"},
    {0x3, 32, 0, "Direct_ref"},
    {0x4, 32, 0, "V_batt_filt_100ms"},
    {0x5, 32, 0, "Board_Temperature"},
    {0x6, 32, 0, "T_mot1_filt_100ms"},
    {0x7, 16, 0, "ctrl_status_cmd"},
    {0x8, 16, 0, "ctrl_status_cmd_ack"},
    {0x9, 16, 0, "flash_params_cmd"},
    {0xa, 16, 0, "flash_params_cmd_ack"},
};




#endif
