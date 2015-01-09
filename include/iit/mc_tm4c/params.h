#ifndef __U_PARAMS_H__
#define __U_PARAMS_H__

#define DEFAULT_KP_TORQUE_CONTROL		(float32)(0.2)
#define DEFAULT_KI_TORQUE_CONTROL		(float32)(0.0002)
#define DEFAULT_KD_TORQUE_CONTROL		(float32)(0.0)

#define DEFAULT_TORQUE_MAX				(float32)(200.0)		// 200 Nm max torque in impedance control mode
#define DEFAULT_IQ_MAX					(float32)(80.0)			// 80 A max current in position and impedance control mode

#define DEFAULT_MIN_POSITION			(float32)(0.25)			// Half radiant between 0 to 2pi transition is forbidden
#define DEFAULT_MAX_POSITION			(float32)(6.0331853)

#define DEFAULT_GAIN_TORQUE_FF			(float32)(0.5)
#define	DEFAULT_SPEED_MAX				(float32)(10.0)			// 10 rad/sec max

#define PARAMS_TOR_KP_MIN				(float32)(0.0)
#define PARAMS_TOR_KP_MAX				(float32)(100.0)
#define PARAMS_TOR_KI_MIN				(float32)(0.0)
#define PARAMS_TOR_KI_MAX				(float32)(100.0)
#define PARAMS_TOR_KD_MIN				(float32)(0.0)
#define PARAMS_TOR_KD_MAX				(float32)(100.0)
#define PARAMS_TOR_FF_MIN				(float32)(0.0)
#define PARAMS_TOR_FF_MAX				(float32)(100.0)
#define PARAMS_MIN_POS_MIN				(float32)(0.28)
#define PARAMS_MIN_POS_MAX				(float32)(6.00)
#define PARAMS_MAX_POS_MIN				(float32)(0.28)
#define PARAMS_MAX_POS_MAX				(float32)(6.00)
#define PARAMS_MAX_SPEED_MIN			(float32)(0.0)
#define PARAMS_MAX_SPEED_MAX			(float32)(20.00)	// rad/sec
#define PARAMS_MAX_TOR_MIN				(float32)(0.0)
#define PARAMS_MAX_TOR_MAX				(float32)(20.00)	// Nm
#define PARAMS_MAX_CUR_MIN				(float32)(0.0)
#define PARAMS_MAX_CUR_MAX				(float32)(90.00)	// A
#define PARAMS_ENC_OFFS_MIN				(float32)(0.0)
#define PARAMS_ENC_OFFS_MAX				(float32)(6.2831853071796)	// TWO PI
#define PARAMS_REL_ENC_OFFS_MIN			(float32)(0.0)
#define PARAMS_REL_ENC_OFFS_MAX			(float32)(6.2831853071796)	// TWO PI
#define PARAMS_PHASE_ANGLE_MIN			(float32)(0.0)
#define PARAMS_PHASE_ANGLE_MAX			(float32)(6.2831853071796)	// TWO PI

#define FLASH_PARAMS_VALID		0xCAFE

#define CONTROL_ACTIVE			1
#define CONTROL_NOT_ACTIVE		0

#define JOINT_IMPEDANCE_MODE	0xDEAD
#define JOINT_POSITION_MODE		0xBEEF
#define JOINT_DIRECT_MODE		0xCAFE

inline std::map<std::string,int> init_ctrl_params()
{
std::map<std::string,int> ctrl_params;
#define define(x,y) ctrl_params[#x]=y;

// Control commands
define ( CTRL_POWER_MOD_ON		,0x00A5)
define ( CTRL_POWER_MOD_OFF		,0x005A)
define ( CTRL_SET_IMPED_MODE		,0x00D4)
define ( CTRL_SET_POS_MODE		,0x003B)
define ( CTRL_SET_DIRECT_MODE	,0x004F)
define ( CTRL_FAN_ON				,0x0026)
define ( CTRL_FAN_OFF			,0x0062)
define ( CTRL_LED_ON				,0x0019)
define ( CTRL_LED_OFF			,0x0091)
define ( CTRL_ALIGN_ENCODERS		,0x00B2)
define ( CTRL_SET_ZERO_POSITION	,0x00AB)
define ( CTRL_REMOVE_TORQUE_OFFS	,0x00CD)

define ( CTRL_CMD_DONE			,0x7800)
define ( CTRL_CMD_ERROR			,0xAA00)
return ctrl_params;
#undef define
}
// Flash params commands
#define FLASH_PARAMS_LOADED		0xDEAD
#define DEFAULT_PARAMS_LOADED	0xBEEF
#define SAVE_PARAMS_TO_FLASH	0x0012
#define LOAD_PARAMS_FROM_FLASH	0x0034
#define LOAD_DEFAULT_PARAMS		0x0056
#define ERASE_FLASH_PARAMS		0x00F1
#define ERASE_HALL_CALIBRATION  0x00F2
#define ERASE_ENC_CALIBRATION  	0x00F3

#define PARAMS_CMD_DONE			0x7800
#define PARAMS_CMD_ERROR		0xAA00
#define PARAMS_CMD_FLASH_BLANK	0x8500

#endif
