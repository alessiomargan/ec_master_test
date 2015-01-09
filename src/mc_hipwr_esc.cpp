#include <iit/ecat/advr/mc_hipwr_esc.h>
#include <string>
using namespace iit::ecat::advr;

// HPtFlashParameters    HPESC::flash_param;
// HPtParameters         HPESC::param;
// HPESCTypes::pdo_rx HPESC::sdo_rx_pdo;
// HPESCTypes::pdo_tx HPESC::sdo_tx_pdo;

const objd_t HPESC::SDOs[] =
{
    // SD0 0x6000
    { 0X6000, 0x1, DTYPE_REAL32,        32,  ATYPE_RO,   "max_temperature"   ,(void*)&HPESC::sdo_rx_pdo.max_temperature           },
    { 0X6000, 0x2, DTYPE_REAL32,        32,  ATYPE_RO,   "position"          ,(void*)&HPESC::sdo_rx_pdo.position                  },
    { 0X6000, 0x3, DTYPE_REAL32,        32,  ATYPE_RO,   "velocity"          ,(void*)&HPESC::sdo_rx_pdo.velocity                  },
    { 0X6000, 0x4, DTYPE_REAL32,        32,  ATYPE_RO,   "torque"            ,(void*)&HPESC::sdo_rx_pdo.torque                    },
    { 0X6000, 0x5, DTYPE_UNSIGNED16,    16,  ATYPE_RO,   "fault"             ,(void*)&HPESC::sdo_rx_pdo.fault                     },
    { 0X6000, 0x6, DTYPE_UNSIGNED64,    64,  ATYPE_RO,   "rtt"               ,(void*)&HPESC::sdo_rx_pdo.rtt                       },

    // SD0 0x7000
    { 0X7000, 0x1, DTYPE_REAL32,        32,  ATYPE_RW,   "pos_ref"           ,(void*)&HPESC::sdo_tx_pdo.pos_ref                   },
    { 0X7000, 0x2, DTYPE_REAL32,        32,  ATYPE_RW,   "tor_offs"          ,(void*)&HPESC::sdo_tx_pdo.tor_offs                  },
    { 0X7000, 0x3, DTYPE_REAL32,        32,  ATYPE_RW,   "PosGainP"          ,(void*)&HPESC::sdo_tx_pdo.PosGainP                  },
    { 0X7000, 0x4, DTYPE_REAL32,        32,  ATYPE_RW,   "PosGainI"          ,(void*)&HPESC::sdo_tx_pdo.PosGainI                  },
    { 0X7000, 0x5, DTYPE_REAL32,        32,  ATYPE_RW,   "PosGainD"          ,(void*)&HPESC::sdo_tx_pdo.PosGainD                  },
    { 0X7000, 0x6, DTYPE_UNSIGNED64,    64,  ATYPE_RW,   "ts"                ,(void*)&HPESC::sdo_tx_pdo.ts                        },

    // SD0 0x8000
    { 0x8000, 0x1, DTYPE_REAL32,        32, ATYPE_RW,    "Sensor_type"           ,(void*)&HPESC::flash_param.Sensor_type         },
    { 0x8000, 0x2, DTYPE_REAL32,        32, ATYPE_RW,    "TorGainP"              ,(void*)&HPESC::flash_param.TorGainP            },
    { 0x8000, 0x3, DTYPE_REAL32,        32, ATYPE_RW,    "TorGainI"              ,(void*)&HPESC::flash_param.TorGainI            },
    { 0x8000, 0x4, DTYPE_REAL32,        32, ATYPE_RW,    "TorGainD"              ,(void*)&HPESC::flash_param.TorGainD            },
    { 0x8000, 0x5, DTYPE_REAL32,        32, ATYPE_RW,    "TorGainFF"             ,(void*)&HPESC::flash_param.TorGainFF           },
    { 0x8000, 0x6, DTYPE_REAL32,        32, ATYPE_RW,    "Pos_I_lim"             ,(void*)&HPESC::flash_param.Pos_I_lim           },
    { 0x8000, 0x7, DTYPE_REAL32,        32, ATYPE_RW,    "Tor_I_lim"             ,(void*)&HPESC::flash_param.Tor_I_lim           },
    { 0x8000, 0x8, DTYPE_REAL32,        32, ATYPE_RW,    "Min_pos"               ,(void*)&HPESC::flash_param.Min_pos             },
    { 0x8000, 0x9, DTYPE_REAL32,        32, ATYPE_RW,    "Max_pos"               ,(void*)&HPESC::flash_param.Max_pos             },
    { 0x8000, 0xa, DTYPE_REAL32,        32, ATYPE_RW,    "Max_vel"               ,(void*)&HPESC::flash_param.Max_vel             },
    { 0x8000, 0xb, DTYPE_REAL32,        32, ATYPE_RW,    "Max_tor"               ,(void*)&HPESC::flash_param.Max_tor             },
    { 0x8000, 0xc, DTYPE_REAL32,        32, ATYPE_RW,    "Max_cur"               ,(void*)&HPESC::flash_param.Max_cur             },
    { 0x8000, 0xd, DTYPE_REAL32,        32, ATYPE_RO,    "Enc_offset"            ,(void*)&HPESC::flash_param.Enc_offset          },
    { 0x8000, 0xe, DTYPE_REAL32,        32, ATYPE_RO,    "Enc_relative_offset"   ,(void*)&HPESC::flash_param.Enc_relative_offset },
    { 0x8000, 0xf, DTYPE_REAL32,        32, ATYPE_RW,    "Phase_angle"           ,(void*)&HPESC::flash_param.Phase_angle         },
    { 0x8000, 0x10,DTYPE_REAL32,        32, ATYPE_RW,    "Torque_lin_coeff"      ,(void*)&HPESC::flash_param.Torque_lin_coeff    },

    // SD0 0x8001
    { 0x8001, 0x1, DTYPE_VISIBLE_STRING,64, ATYPE_RO,    "firmware_version"  		 ,(void*)&HPESC::param.firmware_version       },
    { 0x8001, 0x2, DTYPE_INTEGER16,     16, ATYPE_RW,    "ack_board_fault_all"  	 ,(void*)&HPESC::param.ack_board_fault_all    },
    { 0x8001, 0x3, DTYPE_REAL32,        32, ATYPE_RW,    "Direct_ref"  			 ,(void*)&HPESC::param.Direct_ref             },
    { 0x8001, 0x4, DTYPE_REAL32,        32, ATYPE_RO,    "V_batt_filt_100ms" 		 ,(void*)&HPESC::param.V_batt_filt_100ms      },
    { 0x8001, 0x5, DTYPE_REAL32,        32, ATYPE_RO,    "Board_Temperature"  		 ,(void*)&HPESC::param.Board_Temperature      },
    { 0x8001, 0x6, DTYPE_REAL32,        32, ATYPE_RO,    "T_mot1_filt_100ms"  		 ,(void*)&HPESC::param.T_mot1_filt_100ms      },
    { 0x8001, 0x7, DTYPE_INTEGER16,     16, ATYPE_RW,    "ctrl_status_cmd"   		 ,(void*)&HPESC::param.ctrl_status_cmd        },
    { 0x8001, 0x8, DTYPE_INTEGER16,     16, ATYPE_RO,    "ctrl_status_cmd_ack" 		 ,(void*)&HPESC::param.ctrl_status_cmd_ack    },
    { 0x8001, 0x9, DTYPE_INTEGER16,     16, ATYPE_RW,    "flash_params_cmd" 		 ,(void*)&HPESC::param.flash_params_cmd       },
    { 0x8001, 0xa, DTYPE_INTEGER16,     16, ATYPE_RO,    "flash_params_cmd_ack" 	 ,(void*)&HPESC::param.flash_params_cmd_ack   },
    { 0x8001, 0xb, DTYPE_REAL32,        32, ATYPE_RO,    "abs_enc_mot" 	  		 ,(void*)&HPESC::param.abs_enc_mot            },
    { 0x8001, 0xc, DTYPE_REAL32,        32, ATYPE_RO,    "abs_enc_load"  		 ,(void*)&HPESC::param.abs_enc_load           },
    { 0x8001, 0xd, DTYPE_REAL32,        32, ATYPE_RO, 	 "angle_enc_mot" 		 ,(void*)&HPESC::param.angle_enc_mot          },
    { 0x8001, 0xe, DTYPE_REAL32,        32, ATYPE_RO,	 "angle_enc_load" 		 ,(void*)&HPESC::param.angle_enc_load         },
    
    {0, 0, 0, 0, 0, 0}

};

const objd_t * HPESC::SDOs6000 = &HPESC::SDOs[0];   // #6
const objd_t * HPESC::SDOs7000 = &HPESC::SDOs[6];   // #6
const objd_t * HPESC::SDOs8000 = &HPESC::SDOs[12];  // #16
const objd_t * HPESC::SDOs8001 = &HPESC::SDOs[28];  // #14



