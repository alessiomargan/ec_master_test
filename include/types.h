#ifndef TYPES_MOTOR_CALIBRATION_GUI
#define TYPES_MOTOR_CALIBRATION_GUI

#include <iostream>
#include <json/json.h>


// TX  slave_input -- master output
struct pdo_tx
{
    float        pos_ref;
    float        tor_ref;
    float        direct_ref;
    uint64_t    ts;

    void serializeToJson(json_object* jObj);
    void deSerializeToJson(json_object *jObj);
};
  
// RX  slave_output -- master input
struct pdo_rx
{
    float        position;   // rad
    float        velocity;   // rad/s
    float        torque;     // Nm
    float        torque_D;   // Nm/s
    float        direct_out; // A in AC or V in DC
    uint16_t    fault;
    uint64_t    rtt;        // ns 

    void serializeToJson(json_object* jObj);
    void deSerializeToJson(json_object *jObj);        
};

struct tDriveParameters
{
    unsigned long Sensor_type;

    float PosGainP;
    float PosGainI;
    float PosGainD;

    float TorGainP;
    float TorGainI;
    float TorGainD;

    float TorGainFF;

    float Pos_I_lim;
    float Tor_I_lim;

    float Min_pos;
    float Max_pos;

    float Max_tor;
    float Max_vel;
    float Max_cur;

    long Enc_offset_1;    // 32 bit
    long Enc_offset_2;    // 32 bit

    float Phase_angle;

    void serializeToJson(json_object* jObj);
    void deSerializeToJson(json_object *jObj);
};

#endif //TYPES_MOTOR_CALIBRATION_GUI
