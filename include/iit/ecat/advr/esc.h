/*
 *
 *  Created on: Dec, 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_ESC_H__
#define __IIT_ECAT_ADVR_ESC_H__

#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/utils.h>

#include <map>
#include <fstream>
#include <exception>

#include <yaml-cpp/yaml.h>

#define DEG2RAD(X)  ((float)X*M_PI)/180.0


// Control commands
#define CTRL_POWER_MOD_ON		0x00A5
#define CTRL_POWER_MOD_OFF		0x005A
#define CTRL_SET_IMPED_MODE		0x00D4
#define CTRL_SET_POS_MODE		0x003B
#define CTRL_SET_MIX_POS_MODE	0x003C
#define CTRL_SET_MIX_POS_MODE_2	0x003D
#define CTRL_SET_POS_MOT_MODE	0x005C
#define CTRL_SET_POS_LNK_MODE	0x005D
#define CTRL_SET_POS_LNK_ERR	0x005E

#define CTRL_SET_DIRECT_MODE	0x004F
#define CTRL_FAN_ON				0x0026
#define CTRL_FAN_OFF			0x0062
#define CTRL_LED_ON				0x0019
#define CTRL_LED_OFF			0x0091

//#define CTRL_ALIGN_ENCODERS		0x00B2
#define CTRL_SET_ZERO_POSITION	0x00AB

// FT6
#define CTRL_REMOVE_TORQUE_OFFS	0x00CD

#define FLASH_SAVE              0x0012

#define CTRL_CMD_DONE			0x7800
#define CTRL_CMD_ERROR			0xAA00

namespace iit {
namespace ecat {
namespace advr {


#if 0
typedef struct {
    float        pos_ref;  //link
    int16_t        vel_ref;  //link
    int16_t        tor_ref;  //link
    uint16_t    gains[5];
    uint16_t    fault_ack;
    uint16_t    ts;
    uint16_t    op_idx_aux;    // op [get/set] , idx
    float        aux;        // set value
} PACKED rx_pdo_t; // 28 bytes

typedef struct {
    float        link_pos;           // rad
    float        motor_pos;           // rad
    float        pos_ref_fb;            // rad
    int16_t        motor_vel;             // rad/s
    int16_t        torque;             // Nm
    uint16_t    max_temperature;     // C
    uint16_t    fault;
    uint16_t    rtt;                // us
    uint16_t    op_idx_ack;         // op [ack/nack] , idx
    float        aux;                // get value or nack erro code
} PACKED tx_pdo_t; // 28 bytes
#endif






// ecat slave product code see xml conf
enum Board_type : uint16_t {
    NO_TYPE         = 0,
    HI_PWR_AC_MC    = 0x10,
    HI_PWR_DC_MC    = 0x11,
    LO_PWR_DC_MC    = 0x12,
    LO_PWR_SPH_MC   = 0x13,
    LO_PWR_SPH_MCBRK= 0x14,
    FT6             = 0x20,
    POW_BOARD       = 0x30,
    POW_CMN_BOARD   = 0x31,
    HUB             = 0x100,
    HUB_IO          = 0x101,
    EC_TEST         = 1234,
};


/* Possible error codes returned */
enum ec_board_ctrl_err: int {
    /* No error */
    EC_BOARD_OK         = 0,
    /* erros */
    EC_BOARD_NOK,
    EC_BOARD_CMD_ACK,
    EC_BOARD_INIT_SDO_FAIL,
    EC_BOARD_KEY_NOT_FOUND,
    EC_BOARD_FT6_CALIB_FAIL,
    EC_BOARD_INVALID_ROBOT_ID,
    EC_BOARD_RECV_FAIL,
    EC_BOARD_SEND_FAIL,
    EC_BOARD_ZERO_POS_FAIL,
};

struct fault_bits {
    uint16_t  rxpdo_pos_ref:1;
    uint16_t  rxpdo_tor_offs:1;
    uint16_t  rxpdo_kp_pos:1;
    uint16_t  rxpdo_ki_pos:1;
    uint16_t  rxpdo_kd_pos:1;
    uint16_t  fault_encoder_1:1;
    uint16_t  fault_encoder_2:1;
    uint16_t  fault_hardware:1;
    uint16_t  params_out_of_range:1;
    uint16_t  Max_cur_limited_for_temp:1;
    uint16_t  flag_10:1;
    uint16_t  flag_11:1;
    uint16_t  flag_12:1;
    uint16_t  flag_13:1;
    uint16_t  flag_14:1;
    uint16_t  irq_alive:1;
};

typedef union {
    uint16_t            all;
    struct fault_bits   bit;
} fault_t;

typedef std::map<std::string, std::string> jmap_t;

#define JPDO(x) jpdo[#x] = std::to_string(x);


struct McEscPdoTypes {
    // TX  slave_input -- master output
    struct pdo_tx {
        float       pos_ref;
        uint16_t    fault_ack;
        uint16_t    gainP;
        uint16_t    gainD;
        uint16_t    ts;

        void fprint ( FILE *fp ) {
            fprintf ( fp, "%f\t0x%X\t%d\t%d\t%d\n", pos_ref,fault_ack,gainP,gainD,ts );
        }
        int sprint ( char *buff, size_t size ) {
            return snprintf ( buff, size, "%f\t0x%X\t%d\t%d\t%d", pos_ref,fault_ack,gainP,gainD,ts );
        }

    }  __attribute__ ( ( __packed__ ) ); // 12 bytes

    // RX  slave_output -- master input
    struct pdo_rx {
        float       link_pos;     // rad
        float       motor_pos;     // rad
        float       pos_ref_fb;   // rad
        uint16_t    temperature;  // C * 10
        int16_t     torque;       // Nm * 100
        uint16_t    fault;
        uint16_t    rtt;          //

        void fprint ( FILE *fp ) {
            fprintf ( fp, "%f\t%f\t%f\t%d\t%d\t0x%X\t%d\n", link_pos,motor_pos,pos_ref_fb,temperature,torque,fault,rtt );
        }
        int sprint ( char *buff, size_t size ) {
            return snprintf ( buff, size, "%f\t%f\t%f\t%d\t%d\t0x%X\t%d", link_pos,motor_pos,pos_ref_fb,temperature,torque,fault,rtt );
        }
        void to_map ( jmap_t & jpdo ) {
            JPDO ( link_pos );
            JPDO ( motor_pos );
            JPDO ( pos_ref_fb );
            JPDO ( temperature );
            JPDO ( torque );
            JPDO ( fault );
            JPDO ( rtt );
        }

    }  __attribute__ ( ( __packed__ ) ); // 20 bytes
};


inline int check_cmd_ack ( int16_t cmd, int16_t ack ) {
    if ( ack == ( ( cmd & 0x00FF ) | CTRL_CMD_DONE ) ) {
        DPRINTF ( "DONE 0x%04X\n", cmd );
        return EC_BOARD_OK;
    } else if ( ack == ( ( cmd & 0x00FF ) | CTRL_CMD_ERROR ) ) {
        DPRINTF ( "FAIL 0x%04X\n", cmd );
        return EC_BOARD_CMD_ACK;
    } else {
        DPRINTF ( "PROTOCOL FAILURE cmd 0x%04X ack 0x%04X !!!\n", cmd, ack );
        return EC_BOARD_NOK;
    }

}

template <class C>
inline int ack_faults_X ( C *c, int32_t faults ) {
    int32_t clear_faults = faults & 0x7FFF;
    //xor_faults ^= xor_faults;
    return c->template writeSDO_byname ( "ack_board_fault_all", clear_faults );

}


template <class C>
inline int set_ctrl_status_X ( C *c, uint16_t cmd ) {
    uint16_t ack;

    cmd = cmd & 0x00FF;
    c->template writeSDO_byname ( "ctrl_status_cmd", cmd );
    c->template readSDO_byname ( "ctrl_status_cmd_ack", ack );

    // check
    DPRINTF ( "set_ctrl_status " );
    return check_cmd_ack ( cmd, ack );
}

template <class C>
inline int set_flash_cmd_X ( C *c, uint16_t cmd ) {
    uint16_t ack;

    cmd = cmd & 0x00FF;
    c->template writeSDO_byname ( "flash_params_cmd", cmd );
    c->template readSDO_byname ( "flash_params_cmd_ack", ack );

    // check
    DPRINTF ( "flash_params_cmd " );
    return check_cmd_ack ( cmd, ack );

}


}
}
}

#endif /* IIT_ECAT_ADVR_ESC_H_ */
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
