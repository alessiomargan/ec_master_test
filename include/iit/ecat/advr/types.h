#ifndef ECAT_TYPES_H
#define ECAT_TYPES_H
/*
 FLASHSTORE _objd SDO8000[] =
{
  {0x0, DTYPE_UNSIGNED8,     8,  ATYPE_RO,    acName8000_0,  15, nil},
  {0x1, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_1,  0, &g_sParameters.Sensor_type},
  {0x2, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_2,  0, &g_sParameters.TorGainP},
  {0x3, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_3,  0, &g_sParameters.TorGainI},
  {0x4, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_4,  0, &g_sParameters.TorGainD},
  {0x5, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_5,  0, &g_sParameters.TorGainFF},
  {0x6, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_6,  0, &g_sParameters.Pos_I_lim},
  {0x7, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_7,  0, &g_sParameters.Tor_I_lim},
  {0x8, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_8,  0, &g_sParameters.Min_pos},
  {0x9, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_9,  0, &g_sParameters.Max_pos},
  {0xa, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_10, 0, &g_sParameters.Max_vel},
  {0xb, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_11, 0, &g_sParameters.Max_tor},
  {0xc, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_12, 0, &g_sParameters.Max_cur},
  {0xd, DTYPE_REAL32,            32, ATYPE_RO,    acName8000_13, 0, &g_sParameters.Enc_offset},
  {0xe, DTYPE_REAL32,            32, ATYPE_RO,    acName8000_14, 0, &g_sParameters.Enc_relative_offset},
  {0xf, DTYPE_REAL32,        32, ATYPE_RW,    acName8000_15, 0, &g_sParameters.Phase_angle},
};

FLASHSTORE _objd SDO8001[] =
{
  {0x0, DTYPE_UNSIGNED8,        8,  ATYPE_RO, acName8000_0,  10, nil},
  {0x1, DTYPE_VISIBLE_STRING,   64, ATYPE_RO, acName8001_1,  0, &firmware_version},
  {0x2, DTYPE_INTEGER16,        16, ATYPE_RW, acName8001_2,  0, &ack_board_fault.all},
  {0x3, DTYPE_REAL32,           32, ATYPE_RW, acName8001_3,  0, &Direct_ref},
  {0x4, DTYPE_REAL32,           32, ATYPE_RO, acName8001_4,  0, &filters.V_batt_filt_100ms},
  {0x5, DTYPE_REAL32,           32, ATYPE_RO, acName8001_5,  0, &Board_Temperature},
  {0x6, DTYPE_REAL32,           32, ATYPE_RO, acName8001_6,  0, &filters.T_mot1_filt_100ms},
  {0x7, DTYPE_INTEGER16,        16, ATYPE_RW, acName8001_7,  0, &ctrl_status_cmd},
  {0x8, DTYPE_INTEGER16,        16, ATYPE_RO, acName8001_8,  0, &ctrl_status_cmd_ack},
  {0x9, DTYPE_INTEGER16,        16, ATYPE_RW, acName8001_9,  0, &flash_params_cmd},
  {0xa, DTYPE_INTEGER16,        16, ATYPE_RO, acName8001_10, 0, &flash_params_cmd_ack},
};


 */
#include <map>
namespace iit {
    namespace ecat {
        namespace advr {

            /**
             *  
             **/ 
            
            struct McESCTypes {
                // TX  slave_input -- master output
                typedef struct {
                    float	    pos_ref;
                    float		tor_offs;
                    float		PosGainP;
                    float		PosGainI;
                    float		PosGainD;
                    uint64_t	ts;

                }  __attribute__((__packed__)) pdo_tx;

                // RX  slave_output -- master input
                typedef struct {
                    float	    position;   		// rad
                	float		velocity;   		// rad/s
                	float		torque;     		// Nm
                	float		max_temperature; 	// C
                	uint16_t    fault;
                	uint64_t	rtt;        		// ns
                }  __attribute__((__packed__)) pdo_rx;
            };


            typedef struct {

                unsigned long Sensor_type;      // Sensor type: NOT USED

                float TorGainP;
                float TorGainI;
                float TorGainD;

                float TorGainFF;

                float Pos_I_lim;                // Integral limit: NOT USED
                float Tor_I_lim;                // Integral limit: NOT USED

                float Min_pos;
                float Max_pos;

                float Max_vel;
                float Max_tor;
                float Max_cur;

                float Enc_offset;
                float Enc_relative_offset;

                float Phase_angle;

            }
            tDriveParameters;

        typedef struct 
        {
            std::string     firmware_version;
            uint16_t        all;
            float           Direct_ref;
            float   V_batt_filt_100ms;
            float   Board_Temperature;
            float   T_mot1_filt_100ms;
            uint16_t ctrl_status_cmd;
            uint16_t ctrl_status_cmd_ack;
            uint16_t flash_params_cmd;
            uint16_t flash_params_cmd_ack;
            uint64_t abs_enc_mot;
            uint64_t abs_enc_load;     
        } parameters8001;
        }
    }
}

#endif