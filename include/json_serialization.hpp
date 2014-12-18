#ifndef JSON_SERIALIZATION_H
#define JSON_SERIALIZATION_H

#include <json/json.h>
#include "iit/ecat/advr/types.h"
#define var2string(x) #x

class json_serializer
{
private:
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

    bool get_object(std::string name, float* value, json_object* jobj)
    {
	json_object *jn;
	bool result=json_object_object_get_ex(jobj,name.c_str(),&jn);
	*value = json_object_get_double(jn);
        return result;
    }

    bool get_object(std::string name, uint64_t* value, json_object* jobj)
    {
	json_object *jn;
        bool result=json_object_object_get_ex(jobj,name.c_str(),&jn);
	*value = json_object_get_int64(jn);
        return result;
    }

    bool get_object(std::string name, uint16_t* value, json_object* jobj)
    {
	json_object *jn;
        bool result=json_object_object_get_ex(jobj,name.c_str(),&jn);
	*value = json_object_get_int(jn);
        return result;
    }

    bool get_object(std::string name, long* value, json_object* jobj)
    {
	json_object *jn;
        bool result=json_object_object_get_ex(jobj,name.c_str(),&jn);
	*value = json_object_get_int(jn);
        return result;
    }
    
public:   
    
    void serializeToJson(iit::ecat::advr::McESCTypes::pdo_tx& tx, json_object* jObj)
    {
	/*
	float        pos_ref;
	float        tor_ref;
	float        direct_ref;
	uint64_t    ts;
	*/

	add_object(var2string(pos_ref),tx.pos_ref,jObj);
	add_object(var2string(tor_ref),tx.tor_ref,jObj);
	add_object(var2string(direct_ref),tx.direct_ref,jObj);
	add_object(var2string(ts),tx.ts,jObj);
	
    }

    void serializeToJson(iit::ecat::advr::McESCTypes::pdo_rx& rx, json_object* jObj)
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

	add_object(var2string(position),rx.position,jObj);
	add_object(var2string(velocity),rx.velocity,jObj);
	add_object(var2string(torque),rx.torque,jObj);
	add_object(var2string(torque_D),rx.torque_D,jObj);
	add_object(var2string(direct_out),rx.direct_out,jObj);
	add_object(var2string(fault),rx.fault,jObj);
	add_object(var2string(rtt),rx.fault,jObj);
	
    }
/*
    void serializeToJson(tDriveParameters& tdrive, json_object* jObj)
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
/*
	add_object(var2string(Sensor_type),tdrive.Sensor_type,jObj);
	add_object(var2string(PosGainP),tdrive.PosGainP,jObj);
	add_object(var2string(PosGainI),tdrive.PosGainI,jObj);
	add_object(var2string(PosGainD),tdrive.PosGainD,jObj);
	add_object(var2string(TorGainP),tdrive.TorGainP,jObj);
	add_object(var2string(TorGainI),tdrive.TorGainI,jObj);
	add_object(var2string(TorGainD),tdrive.TorGainD,jObj);
	add_object(var2string(TorGainFF),tdrive.TorGainFF,jObj);
	add_object(var2string(Pos_I_lim),tdrive.Pos_I_lim,jObj);
	add_object(var2string(Tor_I_lim),tdrive.Tor_I_lim,jObj);
	add_object(var2string(Min_pos),tdrive.Min_pos,jObj);
	add_object(var2string(Max_pos),tdrive.Max_pos,jObj);
	add_object(var2string(Max_tor),tdrive.Max_tor,jObj);
	add_object(var2string(Max_vel),tdrive.Max_vel,jObj);
	add_object(var2string(Max_cur),tdrive.Max_cur,jObj);
	add_object(var2string(Enc_offset_1),tdrive.Enc_offset_1,jObj);
	add_object(var2string(Enc_offset_2),tdrive.Enc_offset_2,jObj);
	add_object(var2string(Phase_angle),tdrive.Phase_angle,jObj);
	
    }
*/
    void deSerializeToJson(iit::ecat::advr::McESCTypes::pdo_tx& tx, json_object* jObj)
    {
	/*
	float        pos_ref;
	float        tor_ref;
	float        direct_ref;
	uint64_t    ts;
	*/

	get_object(var2string(pos_ref),&tx.pos_ref,jObj);
	get_object(var2string(tor_ref),&tx.tor_ref,jObj);
	get_object(var2string(direct_ref),&tx.direct_ref,jObj);
	get_object(var2string(ts),&tx.ts,jObj);
    }

    void deSerializeToJson(iit::ecat::advr::McESCTypes::pdo_rx& rx, json_object* jObj)
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

	get_object(var2string(position),&(rx.position),jObj);
	get_object(var2string(velocity),&(rx.velocity),jObj);
	get_object(var2string(torque),&(rx.torque),jObj);
	get_object(var2string(torque_D),&(rx.torque_D),jObj);
	get_object(var2string(direct_out),&(rx.direct_out),jObj);
	get_object(var2string(fault),&(rx.fault),jObj);
	get_object(var2string(rtt),&(rx.rtt),jObj);
    }
/*
    void deSerializeToJson(tDriveParameters& tdrive, json_object* jObj)
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
/*
	get_object(var2string(Sensor_type),&(tdrive.Sensor_type),jObj);
	get_object(var2string(PosGainP),&(tdrive.PosGainP),jObj);
	get_object(var2string(PosGainI),&(tdrive.PosGainI),jObj);
	get_object(var2string(PosGainD),&(tdrive.PosGainD),jObj);
	get_object(var2string(TorGainP),&(tdrive.TorGainP),jObj);
	get_object(var2string(TorGainI),&(tdrive.TorGainI),jObj);
	get_object(var2string(TorGainD),&(tdrive.TorGainD),jObj);
	get_object(var2string(TorGainFF),&(tdrive.TorGainFF),jObj);
	get_object(var2string(Pos_I_lim),&(tdrive.Pos_I_lim),jObj);
	get_object(var2string(Tor_I_lim),&(tdrive.Tor_I_lim),jObj);
	get_object(var2string(Min_pos),&(tdrive.Min_pos),jObj);
	get_object(var2string(Max_pos),&(tdrive.Max_pos),jObj);
	get_object(var2string(Max_tor),&(tdrive.Max_tor),jObj);
	get_object(var2string(Max_vel),&(tdrive.Max_vel),jObj);
	get_object(var2string(Max_cur),&(tdrive.Max_cur),jObj);
	get_object(var2string(Enc_offset_1),&(tdrive.Enc_offset_1),jObj);
	get_object(var2string(Enc_offset_2),&(tdrive.Enc_offset_2),jObj);
	get_object(var2string(Phase_angle),&(tdrive.Phase_angle),jObj);
    }*/
};

#endif //JSON_SERIALIZATION_H