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
                    float       pos_ref;
                    float           tor_ref;
                    float           direct_ref;
                    uint64_t        ts;
                    
                }  __attribute__((__packed__)) pdo_tx;
                
                // RX  slave_output -- master input
                typedef struct {
                    float       position;   // rad
                    float           velocity;   // rad/s
                    float           torque;     // Nm
                    float           torque_D;   // Nm/s
                    float           direct_out; // A in AC or V in DC
                    uint16_t    fault;
                    uint64_t        rtt;        // ns 
                    
                }  __attribute__((__packed__)) pdo_rx;
            };
}
}
}

#endif