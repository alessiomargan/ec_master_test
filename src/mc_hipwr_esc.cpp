#include <iit/ecat/advr/mc_hipwr_esc.h>
#include <string>

using namespace iit::ecat::advr;
using namespace iit::ecat;


static const iit::ecat::objd_t source_SDOs[] = {
    // SD0 0x6000
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
    // SD0 0x7000
    { 0x7000, 1, DTYPE_REAL32,      32, ATYPE_RW, "pos_ref",        0},
    { 0x7000, 2, DTYPE_INTEGER16,       16, ATYPE_RW, "vel_ref",        0},
    { 0x7000, 3, DTYPE_INTEGER16,       16, ATYPE_RW, "tor_ref",        0},
    { 0x7000, 4, DTYPE_UNSIGNED16,      16, ATYPE_RW, "gain_0",      0},
    { 0x7000, 5, DTYPE_UNSIGNED16,      16, ATYPE_RW, "gain_1",      0},
    { 0x7000, 6, DTYPE_UNSIGNED16,      16, ATYPE_RW, "gain_2",      0},
    { 0x7000, 7, DTYPE_UNSIGNED16,      16, ATYPE_RW, "gain_3",      0},
    { 0x7000, 8, DTYPE_UNSIGNED16,      16, ATYPE_RW, "gain_4",        0},
    { 0x7000, 9, DTYPE_UNSIGNED16,      16, ATYPE_RW, "fault_ack",      0},
    { 0x7000, 10, DTYPE_UNSIGNED16,     16, ATYPE_RW, "ts",     0},
    { 0x7000, 11, DTYPE_UNSIGNED16,     16, ATYPE_RW, "op_idx_aux",     0},
    { 0x7000, 12, DTYPE_REAL32,     32, ATYPE_RW, "aux",        0},
    // SD0 0x8000
    { 0x8000, 0x1, DTYPE_REAL32,        32, ATYPE_RW,    "Sensor_type"              ,0     },
    { 0x8000, 0x2, DTYPE_REAL32,        32, ATYPE_RW,    "PosGainP"                 ,0     },
    { 0x8000, 0x3, DTYPE_REAL32,        32, ATYPE_RW,    "PosGainI"                 ,0     },
    { 0x8000, 0x4, DTYPE_REAL32,        32, ATYPE_RW,    "PosGainD"                 ,0     },
    { 0x8000, 0x5, DTYPE_REAL32,        32, ATYPE_RW,    "TorGainP"                 ,0     },
    { 0x8000, 0x6, DTYPE_REAL32,        32, ATYPE_RW,    "TorGainI"                 ,0     },
    { 0x8000, 0x7, DTYPE_REAL32,        32, ATYPE_RW,    "TorGainD"                 ,0     },
    { 0x8000, 0x8, DTYPE_REAL32,        32, ATYPE_RW,    "TorGainFF"                ,0     },
    { 0x8000, 0x9, DTYPE_REAL32,        32, ATYPE_RW,    "Pos_I_lim"                ,0     },
    { 0x8000, 0xa, DTYPE_REAL32,        32, ATYPE_RW,    "Tor_I_lim"                ,0     },
    { 0x8000, 0xb, DTYPE_REAL32,        32, ATYPE_RW,    "Min_pos"                  ,0     },
    { 0x8000, 0xc, DTYPE_REAL32,        32, ATYPE_RW,    "Max_pos"                  ,0     },
    { 0x8000, 0xd, DTYPE_REAL32,        32, ATYPE_RW,    "Max_vel"                  ,0     },
    { 0x8000, 0xe, DTYPE_REAL32,        32, ATYPE_RW,    "Max_tor"                  ,0     },
    { 0x8000, 0xf, DTYPE_REAL32,        32, ATYPE_RW,    "Max_cur"                  ,0     },
    { 0x8000, 0x10, DTYPE_REAL32,       32, ATYPE_RW,    "Enc_offset"               ,0     },
    { 0x8000, 0x11, DTYPE_REAL32,       32, ATYPE_RO,    "Enc_relative_offset"      ,0     },
    { 0x8000, 0x12, DTYPE_REAL32,       32, ATYPE_RW,    "Calibration_angle"        ,0     },
    { 0x8000, 0x13,DTYPE_REAL32,        32, ATYPE_RW,    "Torque_lin_coeff"         ,0     },
    { 0x8000, 0x14,DTYPE_UNSIGNED64,    64, ATYPE_RW,    "Enc_mot_nonius_calib"     ,0     },
    { 0x8000, 0x15,DTYPE_UNSIGNED64,    64, ATYPE_RW,    "Enc_load_nonius_calib"    ,0     },
    { 0x8000, 0x16,DTYPE_INTEGER16,     16, ATYPE_RW,    "Joint_number"             ,0     },
    { 0x8000, 0x17,DTYPE_INTEGER16,     16, ATYPE_RW,    "Joint_robot_id"           ,0     },

    // SD0 0x8001
    { 0x8001, 0x1, DTYPE_VISIBLE_STRING,64, ATYPE_RO,    "firmware_version"  	    ,0     },
    { 0x8001, 0x2, DTYPE_UNSIGNED32,    32, ATYPE_RW,    "board_enable_mask"        ,0     },
    { 0x8001, 0x3, DTYPE_REAL32,        32, ATYPE_RW,    "Direct_ref"  			    ,0     },
    { 0x8001, 0x4, DTYPE_REAL32,        32, ATYPE_RO,    "V_batt_filt_100ms" 	    ,0     },
    { 0x8001, 0x5, DTYPE_REAL32,        32, ATYPE_RO,    "Board_Temperature"  	    ,0     },
    { 0x8001, 0x6, DTYPE_REAL32,        32, ATYPE_RO,    "T_mot1_filt_100ms"  	    ,0     },
    { 0x8001, 0x7, DTYPE_UNSIGNED16,    16, ATYPE_RW,    "ctrl_status_cmd"   	    ,0     },
    { 0x8001, 0x8, DTYPE_UNSIGNED16,    16, ATYPE_RO,    "ctrl_status_cmd_ack" 	    ,0     },
    { 0x8001, 0x9, DTYPE_UNSIGNED16,    16, ATYPE_RW,    "flash_params_cmd" 	    ,0     },
    { 0x8001, 0xa, DTYPE_UNSIGNED16,    16, ATYPE_RO,    "flash_params_cmd_ack"     ,0     },
    { 0x8001, 0xb, DTYPE_REAL32,        32, ATYPE_RO,    "abs_enc_mot" 	  		    ,0     },
    { 0x8001, 0xc, DTYPE_REAL32,        32, ATYPE_RO,    "abs_enc_load"  		    ,0     },
    { 0x8001, 0xd, DTYPE_REAL32,        32, ATYPE_RO, 	 "angle_enc_mot" 		    ,0     },
    { 0x8001, 0xe, DTYPE_REAL32,        32, ATYPE_RO,	 "angle_enc_load" 		    ,0     },
    { 0x8001, 0xf, DTYPE_REAL32,        32, ATYPE_RO, 	 "angle_enc_diff" 		    ,0     },
    { 0x8001, 0x10,DTYPE_REAL32,        32, ATYPE_RO,	 "iq_ref" 		            ,0     },

    {0, 0, 0, 0, 0, 0, 0 }


};


void HpESC::init_SDOs ( void ) {

    int objd_num, i = 0;

    objd_num = sizeof ( source_SDOs ) /sizeof ( objd_t );
    SDOs = new objd_t [objd_num];

    memcpy((void*)SDOs, source_SDOs, sizeof(source_SDOs));

    // 0x6000 
    SDOs[i++].data = (void*)&HpESC::rx_pdo.link_pos;
    SDOs[i++].data = (void*)&HpESC::rx_pdo.motor_pos;
    SDOs[i++].data = (void*)&HpESC::rx_pdo.link_vel;
    SDOs[i++].data = (void*)&HpESC::rx_pdo.motor_vel;
    SDOs[i++].data = (void*)&HpESC::rx_pdo.torque;
    SDOs[i++].data = (void*)&HpESC::rx_pdo.temperature;
    SDOs[i++].data = (void*)&HpESC::rx_pdo.fault;
    SDOs[i++].data = (void*)&HpESC::rx_pdo.rtt;
    SDOs[i++].data = (void*)&HpESC::rx_pdo.op_idx_ack;
    SDOs[i++].data = (void*)&HpESC::rx_pdo.aux;

    //0x7000
    SDOs[i++].data = (void*)&HpESC::tx_pdo.pos_ref;
    SDOs[i++].data = (void*)&HpESC::tx_pdo.vel_ref;
    SDOs[i++].data = (void*)&HpESC::tx_pdo.tor_ref;
    SDOs[i++].data = (void*)&HpESC::tx_pdo.gain_0;
    SDOs[i++].data = (void*)&HpESC::tx_pdo.gain_1;
    SDOs[i++].data = (void*)&HpESC::tx_pdo.gain_2;
    SDOs[i++].data = (void*)&HpESC::tx_pdo.gain_3;
    SDOs[i++].data = (void*)&HpESC::tx_pdo.gain_4;
    SDOs[i++].data = (void*)&HpESC::tx_pdo.fault_ack;
    SDOs[i++].data = (void*)&HpESC::tx_pdo.ts;
    SDOs[i++].data = (void*)&HpESC::tx_pdo.op_idx_aux;
    SDOs[i++].data = (void*)&HpESC::tx_pdo.aux;
    
    // 0x8000
    SDOs[i++].data = ( void* ) &HpESC::sdo.Sensor_type;
    SDOs[i++].data = ( void* ) &HpESC::sdo.PosGainP;
    SDOs[i++].data = ( void* ) &HpESC::sdo.PosGainI;
    SDOs[i++].data = ( void* ) &HpESC::sdo.PosGainD;
    SDOs[i++].data = ( void* ) &HpESC::sdo.TorGainP;
    SDOs[i++].data = ( void* ) &HpESC::sdo.TorGainI;
    SDOs[i++].data = ( void* ) &HpESC::sdo.TorGainD;
    SDOs[i++].data = ( void* ) &HpESC::sdo.TorGainFF;
    SDOs[i++].data = ( void* ) &HpESC::sdo.Pos_I_lim;
    SDOs[i++].data = ( void* ) &HpESC::sdo.Tor_I_lim;
    SDOs[i++].data = ( void* ) &HpESC::sdo.Min_pos;
    SDOs[i++].data = ( void* ) &HpESC::sdo.Max_pos;
    SDOs[i++].data = ( void* ) &HpESC::sdo.Max_vel;
    SDOs[i++].data = ( void* ) &HpESC::sdo.Max_tor;
    SDOs[i++].data = ( void* ) &HpESC::sdo.Max_cur;
    SDOs[i++].data = ( void* ) &HpESC::sdo.Enc_offset;
    SDOs[i++].data = ( void* ) &HpESC::sdo.Enc_relative_offset;
    SDOs[i++].data = ( void* ) &HpESC::sdo.Phase_angle;
    SDOs[i++].data = ( void* ) &HpESC::sdo.Torque_lin_coeff;
    SDOs[i++].data = ( void* ) &HpESC::sdo.Enc_mot_nonius_calib;
    SDOs[i++].data = ( void* ) &HpESC::sdo.Enc_load_nonius_calib;
    SDOs[i++].data = ( void* ) &HpESC::sdo.Joint_number;
    SDOs[i++].data = ( void* ) &HpESC::sdo.Joint_robot_id;
    // 0x8001
    SDOs[i++].data = ( void* ) &HpESC::sdo.firmware_version;
    SDOs[i++].data = ( void* ) &HpESC::sdo.board_enable_mask;
    SDOs[i++].data = ( void* ) &HpESC::sdo.Direct_ref;
    SDOs[i++].data = ( void* ) &HpESC::sdo.V_batt_filt_100ms;
    SDOs[i++].data = ( void* ) &HpESC::sdo.Board_Temperature;
    SDOs[i++].data = ( void* ) &HpESC::sdo.T_mot1_filt_100ms;
    SDOs[i++].data = ( void* ) &HpESC::sdo.ctrl_status_cmd;
    SDOs[i++].data = ( void* ) &HpESC::sdo.ctrl_status_cmd_ack;
    SDOs[i++].data = ( void* ) &HpESC::sdo.flash_params_cmd;
    SDOs[i++].data = ( void* ) &HpESC::sdo.flash_params_cmd_ack;
    SDOs[i++].data = ( void* ) &HpESC::sdo.abs_enc_mot;
    SDOs[i++].data = ( void* ) &HpESC::sdo.abs_enc_load;
    SDOs[i++].data = ( void* ) &HpESC::sdo.angle_enc_mot;
    SDOs[i++].data = ( void* ) &HpESC::sdo.angle_enc_load;
    SDOs[i++].data = ( void* ) &HpESC::sdo.angle_enc_diff;
    SDOs[i++].data = ( void* ) &HpESC::sdo.iq_ref;
    // end marker
    SDOs[i++].data = 0;

    assert ( objd_num == i );
}
