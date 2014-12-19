#ifndef JSON_SERIALIZATION_H
#define JSON_SERIALIZATION_H

#include <json-c/json.h>
#include "iit/ecat/advr/types.h"
#define var2string(x) #x
#include <string>
class json_serializer
{
public:
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
    
    void add_object(std::string name, std::string value, json_object* jobj)
    {
        json_object *jn  = json_object_new_string(value.c_str());
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
    
    bool get_object(std::string name, std::string* value, json_object* jobj)
    {
        json_object *jn;
        bool result=json_object_object_get_ex(jobj,name.c_str(),&jn);
        *value = json_object_get_string(jn);
        return result;
    }
    
public:   
    
    void serializeToJson(iit::ecat::advr::McESCTypes::pdo_tx& tx, json_object* jObj)
    {
    /* 
    float	    pos_ref;
    float		tor_offs;
    float		PosGainP;
    float		PosGainI;
    float		PosGainD;
    uint64_t	ts;
	*/

	add_object(var2string(pos_ref),tx.pos_ref,jObj);
	add_object(var2string(tor_offs),tx.tor_offs,jObj);
	add_object(var2string(PosGainP),tx.PosGainP,jObj);
    add_object(var2string(PosGainI),tx.PosGainI,jObj);
    add_object(var2string(PosGainD),tx.PosGainD,jObj);
    add_object(var2string(ts),tx.ts,jObj);
	
    }

    void serializeToJson(iit::ecat::advr::McESCTypes::pdo_rx& rx, json_object* jObj)
    {
    /* 
    float	    position;   		// rad
    float		velocity;   		// rad/s
    float		torque;     		// Nm
    float		max_temperature; 	// C
    uint16_t    fault;
    uint64_t	rtt;        		// ns 
     
	*/

	add_object(var2string(position),rx.position,jObj);
	add_object(var2string(velocity),rx.velocity,jObj);
	add_object(var2string(torque),rx.torque,jObj);
	add_object(var2string(max_temperature),rx.max_temperature,jObj);
	add_object(var2string(fault),rx.fault,jObj);
	add_object(var2string(rtt),rx.fault,jObj);
	
    }

    void serializeToJson(iit::ecat::advr::tDriveParameters& tdrive, json_object* jObj)
    {

        /*
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

         */
        add_object(var2string(Sensor_type),tdrive.Sensor_type,jObj);
	add_object(var2string(TorGainP),tdrive.TorGainP,jObj);
	add_object(var2string(TorGainI),tdrive.TorGainI,jObj);
	add_object(var2string(TorGainD),tdrive.TorGainD,jObj);
	add_object(var2string(TorGainFF),tdrive.TorGainFF,jObj);
	add_object(var2string(Pos_I_lim),tdrive.Pos_I_lim,jObj);
	add_object(var2string(Tor_I_lim),tdrive.Tor_I_lim,jObj);
	add_object(var2string(Min_pos),tdrive.Min_pos,jObj);
	add_object(var2string(Max_pos),tdrive.Max_pos,jObj);
	add_object(var2string(Max_vel),tdrive.Max_vel,jObj);
	add_object(var2string(Max_tor),tdrive.Max_tor,jObj);
	add_object(var2string(Max_cur),tdrive.Max_cur,jObj);
	add_object(var2string(Enc_offset),tdrive.Enc_offset,jObj);
	add_object(var2string(Enc_relative_offset),tdrive.Enc_relative_offset,jObj);
	add_object(var2string(Phase_angle),tdrive.Phase_angle,jObj);
	
    }
    void serializeToJson(iit::ecat::advr::parameters8001& p8001, json_object* jObj)
    {
        
        /*
         *               std::string firmware_version[8];
         *               uint16_t all;
         *               float Direct_ref;
         *               float V_batt_filt_100ms;
         *               float Board_Temperature;
         *               float T_mot1_filt_100ms;
         *               uint16_t ctrl_status_cmd;
         *               uint16_t ctrl_status_cmd_ack;
         *               uint16_t flash_params_cmd;
         *               uint16_t flash_params_cmd_ack;
         * 
         */
        
        add_object(var2string(firmware_version),p8001.firmware_version,jObj);
        add_object(var2string(all),p8001.all,jObj);
        add_object(var2string(Direct_ref),p8001.Direct_ref,jObj);
        add_object(var2string(V_batt_filt_100ms),p8001.V_batt_filt_100ms,jObj);
        add_object(var2string(Board_Temperature),p8001.Board_Temperature,jObj);
        add_object(var2string(T_mot1_filt_100ms),p8001.T_mot1_filt_100ms,jObj);
        add_object(var2string(ctrl_status_cmd),p8001.ctrl_status_cmd,jObj);
        add_object(var2string(ctrl_status_cmd_ack),p8001.ctrl_status_cmd_ack,jObj);
        add_object(var2string(flash_params_cmd),p8001.flash_params_cmd,jObj);
        add_object(var2string(flash_params_cmd_ack),p8001.flash_params_cmd_ack,jObj);
        
    }
    
    void deSerializeToJson(iit::ecat::advr::McESCTypes::pdo_tx& tx, json_object* jObj)
    {
	/*
	ffloat	    pos_ref;
    float		tor_offs;
    float		PosGainP;
    float		PosGainI;
    float		PosGainD;
    uint64_t	ts;
	*/

	get_object(var2string(pos_ref),&tx.pos_ref,jObj);
	get_object(var2string(tor_offs),&tx.tor_offs,jObj);
	get_object(var2string(PosGainP),&tx.PosGainP,jObj);
    get_object(var2string(PosGainI),&tx.PosGainI,jObj);
    get_object(var2string(PosGainD),&tx.PosGainD,jObj);
    get_object(var2string(ts),&tx.ts,jObj);
    }

    void deSerializeToJson(iit::ecat::advr::McESCTypes::pdo_rx& rx, json_object* jObj)
    {
    /* 
    float	    position;   		// rad
    float		velocity;   		// rad/s
    float		torque;     		// Nm
    float		max_temperature; 	// C
    uint16_t    fault;
    uint64_t	rtt;        		// ns 
	*/

	get_object(var2string(position),&(rx.position),jObj);
	get_object(var2string(velocity),&(rx.velocity),jObj);
	get_object(var2string(torque),&(rx.torque),jObj);
	get_object(var2string(max_temperature),&(rx.max_temperature),jObj);
	get_object(var2string(fault),&(rx.fault),jObj);
	get_object(var2string(rtt),&(rx.rtt),jObj);
    }

    void deSerializeToJson(iit::ecat::advr::tDriveParameters& tdrive, json_object* jObj)
    {
      /*
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
	*/

	get_object(var2string(Sensor_type),&(tdrive.Sensor_type),jObj);
	get_object(var2string(TorGainP),&(tdrive.TorGainP),jObj);
	get_object(var2string(TorGainI),&(tdrive.TorGainI),jObj);
	get_object(var2string(TorGainD),&(tdrive.TorGainD),jObj);
	get_object(var2string(TorGainFF),&(tdrive.TorGainFF),jObj);
	get_object(var2string(Pos_I_lim),&(tdrive.Pos_I_lim),jObj);
	get_object(var2string(Tor_I_lim),&(tdrive.Tor_I_lim),jObj);
	get_object(var2string(Min_pos),&(tdrive.Min_pos),jObj);
	get_object(var2string(Max_pos),&(tdrive.Max_pos),jObj);
	get_object(var2string(Max_tor),&(tdrive.Max_vel),jObj);
	get_object(var2string(Max_vel),&(tdrive.Max_tor),jObj);
	get_object(var2string(Max_cur),&(tdrive.Max_cur),jObj);
	get_object(var2string(Enc_offset),&(tdrive.Enc_offset),jObj);
	get_object(var2string(Enc_relative_offset),&(tdrive.Enc_relative_offset),jObj);
	get_object(var2string(Phase_angle),&(tdrive.Phase_angle),jObj);
    }
    void deSerializeToJson(iit::ecat::advr::parameters8001& p8001, json_object* jObj)
    {
        /*
         *              char firmware_version[8];
         *              uint16_t all;
         *              float Direct_ref;
         *              float V_batt_filt_100ms;
         *              float Board_Temperature;
         *              float T_mot1_filt_100ms;
         *              uint16_t ctrl_status_cmd;
         *              uint16_t ctrl_status_cmd_ack;
         *              uint16_t flash_params_cmd;
         *              uint16_t flash_params_cmd_ack;
         */
        
        get_object(var2string(firmware_version),&(p8001.firmware_version),jObj);
        get_object(var2string(all),&(p8001.all),jObj);
        get_object(var2string(Direct_ref),&(p8001.Direct_ref),jObj);
        get_object(var2string(V_batt_filt_100ms),&(p8001.V_batt_filt_100ms),jObj);
        get_object(var2string(Board_Temperature),&(p8001.Board_Temperature),jObj);
        get_object(var2string(T_mot1_filt_100ms),&(p8001.T_mot1_filt_100ms),jObj);
        get_object(var2string(ctrl_status_cmd),&(p8001.ctrl_status_cmd),jObj);
        get_object(var2string(ctrl_status_cmd_ack),&(p8001.ctrl_status_cmd_ack),jObj);
        get_object(var2string(flash_params_cmd),&(p8001.flash_params_cmd),jObj);
        get_object(var2string(flash_params_cmd_ack),&(p8001.flash_params_cmd_ack),jObj);
    }
    
};

#endif //JSON_SERIALIZATION_H