#include "types.h"

#define params(x) #x,x

void add_object(std::string name, float value, json_object* jobj)
{
    json_object *jfloat  = json_object_new_double(value);
    json_object_object_add(jobj,name.c_str(),jfloat);
}

void add_object(std::string name, uint64_t value, json_object* jobj)
{
    json_object *jn  = json_object_new_int64(value);
    json_object_object_add(jobj,name.c_str(),jn);
}

void add_object(std::string name, uint16_t value, json_object* jobj)
{
    json_object *jn  = json_object_new_int(value);
    json_object_object_add(jobj,name.c_str(),jn);
}

void add_object(std::string name, long value, json_object* jobj)
{
    json_object *jn  = json_object_new_int(value);
    json_object_object_add(jobj,name.c_str(),jn);
}

void pdo_tx::serializeToJson(json_object* jObj)
{
    /*
    float        pos_ref;
    float        tor_ref;
    float        direct_ref;
    uint64_t    ts;
    */

    add_object(params(pos_ref),jObj);
    add_object(params(tor_ref),jObj);
    add_object(params(direct_ref),jObj);
    add_object(params(ts),jObj);
    
}

void pdo_rx::serializeToJson(json_object* jObj)
{
    /*
    float        position;   // rad
    float        velocity;   // rad/s
    float        torque;     // Nm
    float        torque_D;   // Nm/s
    float        direct_out; // A in AC or V in DC
    uint16_t    fault;
    uint64_t    rtt;        // ns 
    */

    add_object(params(position),jObj);
    add_object(params(velocity),jObj);
    add_object(params(torque),jObj);
    add_object(params(torque_D),jObj);
    add_object(params(direct_out),jObj);
    add_object(params(fault),jObj);
    add_object(params(rtt),jObj);
    
}

void tDriveParameters::serializeToJson(json_object* jObj)
{
    /*
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
    */

    add_object(params(Sensor_type),jObj);
    add_object(params(PosGainP),jObj);
    add_object(params(PosGainI),jObj);
    add_object(params(PosGainD),jObj);
    add_object(params(TorGainP),jObj);
    add_object(params(TorGainI),jObj);
    add_object(params(PosGainD),jObj);
    add_object(params(TorGainD),jObj);
    add_object(params(TorGainFF),jObj);
    add_object(params(Pos_I_lim),jObj);
    add_object(params(Tor_I_lim),jObj);
    add_object(params(Min_pos),jObj);
    add_object(params(Max_pos),jObj);
    add_object(params(Max_tor),jObj);
    add_object(params(Max_vel),jObj);
    add_object(params(Max_cur),jObj);
    add_object(params(Enc_offset_1),jObj);
    add_object(params(Enc_offset_2),jObj);
    add_object(params(Phase_angle),jObj);
    
}
  
void get_object(std::string name, float& value, json_object* jobj)
{
    json_object *jfloat  = json_object_object_get(jobj,name.c_str());
    value = json_object_get_double(jfloat);
}

void get_object(std::string name, uint64_t& value, json_object* jobj)
{
    json_object *jn  = json_object_object_get(jobj,name.c_str());
    value = json_object_get_int64(jn);
}

void get_object(std::string name, uint16_t& value, json_object* jobj)
{
    json_object *jn  = json_object_object_get(jobj,name.c_str());
    value = json_object_get_int(jn);
}

void get_object(std::string name, long& value, json_object* jobj)
{
    json_object *jn  = json_object_object_get(jobj,name.c_str());
    value = json_object_get_int(jn);
}
  
void pdo_tx::deSerializeToJson(json_object* jObj)
{
    /*
    float        pos_ref;
    float        tor_ref;
    float        direct_ref;
    uint64_t    ts;
    */

    get_object(params(pos_ref),jObj);
    get_object(params(tor_ref),jObj);
    get_object(params(direct_ref),jObj);
    get_object(params(ts),jObj);
}

void pdo_rx::deSerializeToJson(json_object* jObj)
{
    /*
    float        position;   // rad
    float        velocity;   // rad/s
    float        torque;     // Nm
    float        torque_D;   // Nm/s
    float        direct_out; // A in AC or V in DC
    uint16_t    fault;
    uint64_t    rtt;        // ns 
    */

    get_object(params(position),jObj);
    get_object(params(velocity),jObj);
    get_object(params(torque),jObj);
    get_object(params(torque_D),jObj);
    get_object(params(direct_out),jObj);
    get_object(params(fault),jObj);
    get_object(params(rtt),jObj);
}

void tDriveParameters::deSerializeToJson(json_object* jObj)
{
   /*
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
    */

    get_object(params(Sensor_type),jObj);
    get_object(params(PosGainP),jObj);
    get_object(params(PosGainI),jObj);
    get_object(params(PosGainD),jObj);
    get_object(params(TorGainP),jObj);
    get_object(params(TorGainI),jObj);
    get_object(params(PosGainD),jObj);
    get_object(params(TorGainD),jObj);
    get_object(params(TorGainFF),jObj);
    get_object(params(Pos_I_lim),jObj);
    get_object(params(Tor_I_lim),jObj);
    get_object(params(Min_pos),jObj);
    get_object(params(Max_pos),jObj);
    get_object(params(Max_tor),jObj);
    get_object(params(Max_vel),jObj);
    get_object(params(Max_cur),jObj);
    get_object(params(Enc_offset_1),jObj);
    get_object(params(Enc_offset_2),jObj);
    get_object(params(Phase_angle),jObj);
}