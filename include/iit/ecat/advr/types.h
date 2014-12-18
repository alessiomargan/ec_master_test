#ifndef ECAT_TYPES_H
#define ECAT_TYPES_H

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
        }
    }
}

#endif 