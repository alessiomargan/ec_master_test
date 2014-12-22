/**
 * THIS FILE WILL BE GENERATED  
 */

#include <iit/ecat/advr/mc_hipwr_esc.h>

using namespace iit::ecat::advr;

tFlashParameters    McESC::flash_param;
tParameters         McESC::param;

McESCTypes::pdo_rx McESC::sdo_rx_pdo;
McESCTypes::pdo_tx McESC::sdo_tx_pdo;

const objd_t McESC::SDOs[] =
{
    // SD0 0x6000
    { 0X6000, 0x1, 32,  ATYPE_RO,   (void*)&McESC::sdo_rx_pdo.position          , "position"                  },
    { 0X6000, 0x2, 32,  ATYPE_RO,   (void*)&McESC::sdo_rx_pdo.velocity          , "velocity"                  },
    { 0X6000, 0x3, 32,  ATYPE_RO,   (void*)&McESC::sdo_rx_pdo.torque            , "torque"                    },
    { 0X6000, 0x4, 32,  ATYPE_RO,   (void*)&McESC::sdo_rx_pdo.max_temperature   , "max_temperature"           },
    { 0X6000, 0x5, 16,  ATYPE_RO,   (void*)&McESC::sdo_rx_pdo.fault             , "fault"                     },
    { 0X6000, 0x6, 64,  ATYPE_RO,   (void*)&McESC::sdo_rx_pdo.rtt               , "rtt"                       },

    // SD0 0x7000
    { 0X7000, 0x1, 32,  ATYPE_RW,   (void*)&McESC::sdo_tx_pdo.pos_ref           , "pos_ref"                   },
    { 0X7000, 0x2, 32,  ATYPE_RW,   (void*)&McESC::sdo_tx_pdo.tor_offs          , "tor_offs"                  },
    { 0X7000, 0x3, 32,  ATYPE_RW,   (void*)&McESC::sdo_tx_pdo.PosGainP          , "PosGainP"                  },
    { 0X7000, 0x4, 32,  ATYPE_RW,   (void*)&McESC::sdo_tx_pdo.PosGainI          , "PosGainI"                  },
    { 0X7000, 0x5, 32,  ATYPE_RW,   (void*)&McESC::sdo_tx_pdo.PosGainD          , "PosGainD"                  },
    { 0X7000, 0x6, 64,  ATYPE_RW,   (void*)&McESC::sdo_tx_pdo.ts                , "ts"                        },

    // SD0 0x8000
    { 0x8000, 0x1, 32, ATYPE_RW,    (void*)&McESC::flash_param.Sensor_type           ,  "Sensor_type"         },
    { 0x8000, 0x2, 32, ATYPE_RW,    (void*)&McESC::flash_param.TorGainP              ,  "TorGainP"            },
    { 0x8000, 0x3, 32, ATYPE_RW,    (void*)&McESC::flash_param.TorGainI              ,  "TorGainI"            },
    { 0x8000, 0x4, 32, ATYPE_RW,    (void*)&McESC::flash_param.TorGainD              ,  "TorGainD"            },
    { 0x8000, 0x5, 32, ATYPE_RW,    (void*)&McESC::flash_param.TorGainFF             ,  "TorGainFF"           },
    { 0x8000, 0x6, 32, ATYPE_RW,    (void*)&McESC::flash_param.Pos_I_lim             ,  "Pos_I_lim"           },
    { 0x8000, 0x7, 32, ATYPE_RW,    (void*)&McESC::flash_param.Tor_I_lim             ,  "Tor_I_lim"           },
    { 0x8000, 0x8, 32, ATYPE_RW,    (void*)&McESC::flash_param.Min_pos               ,  "Min_pos"             },
    { 0x8000, 0x9, 32, ATYPE_RW,    (void*)&McESC::flash_param.Max_pos               ,  "Max_pos"             },
    { 0x8000, 0xa, 32, ATYPE_RW,    (void*)&McESC::flash_param.Max_vel               ,  "Max_vel"             },
    { 0x8000, 0xb, 32, ATYPE_RW,    (void*)&McESC::flash_param.Max_tor               ,  "Max_tor"             },
    { 0x8000, 0xc, 32, ATYPE_RW,    (void*)&McESC::flash_param.Max_cur               ,  "Max_cur"             },
    { 0x8000, 0xd, 32, ATYPE_RO,    (void*)&McESC::flash_param.Enc_offset            ,  "Enc_offset"          },
    { 0x8000, 0xe, 32, ATYPE_RO,    (void*)&McESC::flash_param.Enc_relative_offset   ,  "Enc_relative_offset" },
    { 0x8000, 0xf, 32, ATYPE_RW,    (void*)&McESC::flash_param.Phase_angle           ,  "Phase_angle"         },

    // SD0 0x8001
    { 0x8001, 0x1, 64, ATYPE_RO,    (void*)&McESC::param.firmware_version            ,  "firmware_version"       },
    { 0x8001, 0x2, 16, ATYPE_RW,    (void*)&McESC::param.ack_board_fault_all         ,  "ack_board_fault_all"    },
    { 0x8001, 0x3, 32, ATYPE_RW,    (void*)&McESC::param.Direct_ref                  ,  "Direct_ref"             },
    { 0x8001, 0x4, 32, ATYPE_RO,    (void*)&McESC::param.V_batt_filt_100ms           ,  "V_batt_filt_100ms"      },
    { 0x8001, 0x5, 32, ATYPE_RO,    (void*)&McESC::param.Board_Temperature           ,  "Board_Temperature"      },
    { 0x8001, 0x6, 32, ATYPE_RO,    (void*)&McESC::param.T_mot1_filt_100ms           ,  "T_mot1_filt_100ms"      },
    { 0x8001, 0x7, 16, ATYPE_RW,    (void*)&McESC::param.ctrl_status_cmd             ,  "ctrl_status_cmd"        },
    { 0x8001, 0x8, 16, ATYPE_RO,    (void*)&McESC::param.ctrl_status_cmd_ack         ,  "ctrl_status_cmd_ack"    },
    { 0x8001, 0x9, 16, ATYPE_RW,    (void*)&McESC::param.flash_params_cmd            ,  "flash_params_cmd"       },
    { 0x8001, 0xa, 16, ATYPE_RO,    (void*)&McESC::param.flash_params_cmd_ack        ,  "flash_params_cmd_ack"   },
    { 0x8001, 0xb, 32, ATYPE_RO,    (void*)&McESC::param.abs_enc_mot                 ,  "abs_enc_mot"            },
    { 0x8001, 0xc, 32, ATYPE_RO,    (void*)&McESC::param.abs_enc_load                ,  "abs_enc_load"           },
    { 0x8001, 0xd, 32, ATYPE_RO, 	(void*)&McESC::param.abs_enc_mot                 ,  "angle_enc_mot"          },
    { 0x8001, 0xe, 32, ATYPE_RO,	(void*)&McESC::param.abs_enc_load                ,  "angle_enc_load"         },
    // 
    0,
};                                                                     


const objd_t * McESC::SDOs6000 = &McESC::SDOs[0];   // #6
const objd_t * McESC::SDOs7000 = &McESC::SDOs[6];   // #6
const objd_t * McESC::SDOs8000 = &McESC::SDOs[12];  // #15
const objd_t * McESC::SDOs8001 = &McESC::SDOs[27];  // #14

