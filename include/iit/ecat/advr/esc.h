/*
 *
 *  Created on: Dec, 2014
 *      Author: alessio margan
 */

#ifndef __IIT_ECAT_ADVR_ESC_H__
#define __IIT_ECAT_ADVR_ESC_H__

#include <iit/ecat/slave_wrapper.h>
#include <iit/ecat/utils.h>
#include <protobuf/ecat_pdo.pb.h>

#include <map>
#include <fstream>
#include <exception>

#include <yaml-cpp/yaml.h>

#define DEG2RAD(X)  ((float)X*M_PI)/180.0


// Control commands
#define CTRL_POWER_MOD_ON		0x00A5
#define CTRL_POWER_MOD_OFF		0x005A

#define CTRL_SET_DIRECT_MODE    0x004F

#define CTRL_SET_IMPED_MODE		0x00D4
#define CTRL_SET_POS_MODE		0x003B
#define CTRL_SET_POS_LINK_MODE  0x003C
#define CTRL_SET_VEL_MODE       0x0037
#define CTRL_SET_VOLT_MODE      0x0039

#define CTRL_SET_MIX_POS_MODE	0x003C
#define CTRL_SET_MIX_POS_MODE_2	0x003D
#define CTRL_SET_POS_MOT_MODE	0x005C
#define CTRL_SET_POS_LNK_MODE	0x005D
#define CTRL_SET_POS_LNK_ERR	0x005E

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

// ecat slave product code see xml conf
enum Board_type : uint16_t
{ 
    NO_TYPE         = 0,
    HI_PWR_AC_MC    = 0x10,
    HI_PWR_DC_MC    = 0x11,
    LO_PWR_DC_MC    = 0x12,
    LO_PWR_SPH_MC   = 0x13,
    LO_PWR_SPH_MCBRK= 0x14,
    CENT_AC         = 0x15,
    FT6             = 0x20,
    FOOT_SENSOR     = 0x21,
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
    //
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

/////////////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////////////
#if FLOAT_PDO

struct McEscPdoTypes {
    
    // TX  slave_input -- master output
    struct pdo_tx {
        float       pos_ref;    //link
        float       vel_ref;    //link
        float       tor_ref;    //link
        float       gain_0;     //kp_m  ImpPosP 
        float       gain_1;     //kp_l  ImpTorP
        float       gain_2;     //kd_m  ImpPosD
        float       gain_3;     //kd_l  ImpTorD
        float       gain_4;     //ki    ImpTorI
        uint16_t    fault_ack;
        uint16_t    ts;
        uint16_t    op_idx_aux;  // op [get/set] , idx
        float       aux;         // set value

        std::ostream& dump ( std::ostream& os, const std::string delim ) const {
            os << pos_ref << delim;
            os << vel_ref << delim;
            os << tor_ref << delim;
            os << gain_0 << delim;
            os << gain_1 << delim;
            os << gain_2 << delim;
            os << gain_3 << delim;
            os << gain_4 << delim;
            os << fault_ack << delim;
            os << ts << delim;
            //os << std::endl;
            return os;
        }
        void fprint ( FILE *fp ) {
            std::ostringstream oss;
            dump(oss,"\t");
            fprintf ( fp, "%s", oss.str().c_str() );
        }
        int sprint ( char *buff, size_t size ) {
            std::ostringstream oss;
            dump(oss,"\t");
            return snprintf ( buff, size, "%s", oss.str().c_str() );
        }

    }  __attribute__ ( ( __packed__ ) ); // 42 bytes

    // RX  slave_output -- master input
    struct pdo_rx {
        float       link_pos;           // rad
        float       motor_pos;          // rad
        float       link_vel;           // rad TBD on the firmware 
        float       motor_vel;          // rad/s
        float       torque;             // Nm
        uint16_t    temperature;        // C
        uint16_t    fault;
        uint16_t    rtt;                // us
        uint16_t    op_idx_ack;         // op [ack/nack] , idx
        float       aux;                // get value or nack erro code

        std::ostream& dump ( std::ostream& os, const std::string delim ) const {
            os << link_pos << delim;
            os << motor_pos << delim;
            os << link_vel << delim;
            os << motor_vel << delim;
            os << torque << delim;
            os << temperature << delim;
            os << fault << delim;
            os << rtt << delim;
            os << op_idx_ack << delim;
            os << aux << delim;
            //os << std::endl;
            return os;
        }
        void fprint ( FILE *fp ) {
            std::ostringstream oss;
            dump(oss,"\t");
            fprintf ( fp, "%s", oss.str().c_str() );
        }
        int sprint ( char *buff, size_t size ) {
            std::ostringstream oss;
            dump(oss,"\t");
            return snprintf ( buff, size, "%s", oss.str().c_str() );
        }
        void to_map ( jmap_t & jpdo ) {
            JPDO ( link_pos );
            JPDO ( motor_pos );
            JPDO ( link_vel );
            JPDO ( motor_vel );
            JPDO ( torque );
            JPDO ( temperature );
            JPDO ( fault );
            JPDO ( rtt );
            JPDO ( op_idx_ack );
            JPDO ( aux );
        }
        void pb_toString( std::string * pb_str ) {
            static iit::advr::Ec_slave_pdo pb_rx_pdo;
            static struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            // Header
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_sec(ts.tv_sec);
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_nsec(ts.tv_nsec);
            // Type
            pb_rx_pdo.set_type(iit::advr::Ec_slave_pdo::RX_XT_MOTOR);
            // Motor_xt_tx_pdo
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_link_pos(link_pos);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_motor_pos(motor_pos);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_link_vel(link_vel);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_motor_vel(motor_vel);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_torque(torque);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_temperature(temperature);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_fault(fault);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_rtt(rtt);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_op_idx_ack(op_idx_ack);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_aux(aux);
            pb_rx_pdo.SerializeToString(pb_str);
        }
    }  __attribute__ ( ( __packed__ ) ); // 32 bytes

}; // 74 bytes

#else

struct McEscPdoTypes {
                    
    // TX  slave_input -- master output
    struct pdo_tx {
        float       pos_ref;    // rad   
        int16_t     vel_ref;    // mrad/s 
        int16_t     tor_ref;    // mNm
        uint16_t    gain_0;      
        uint16_t    gain_1;     
        uint16_t    gain_2;     
        uint16_t    gain_3;     
        uint16_t    gain_4;     
        uint16_t    fault_ack;
        uint16_t    ts;
        uint16_t    op_idx_aux;  // op [get/set] , idx
        float       aux;         // set value

        std::ostream& dump ( std::ostream& os, const std::string delim ) const {
            os << pos_ref << delim;
            os << vel_ref << delim;
            os << tor_ref << delim;
            os << gain_0 << delim;
            os << gain_1 << delim;
            os << gain_2 << delim;
            os << gain_3 << delim;
            os << gain_4 << delim;
            os << std::hex << fault_ack << std::dec << delim;
            os << ts << delim;
            //os << std::endl;
            return os;
        }
        void fprint ( FILE *fp ) {
            std::ostringstream oss;
            dump(oss,"\t");
            fprintf ( fp, "%s", oss.str().c_str() );
        }
        int sprint ( char *buff, size_t size ) {
            std::ostringstream oss;
            dump(oss,"\t");
            return snprintf ( buff, size, "%s", oss.str().c_str() );
        }

    }  __attribute__ ( ( __packed__ ) ); // 28 bytes

    // RX  slave_output -- master input
    struct pdo_rx {
        float        link_pos;          // rad
        float        motor_pos;         // rad
        int16_t      link_vel;          // mrad/s 
        int16_t      motor_vel;         // mrad/s
        float        torque;            // Nm
        uint16_t     temperature;       // C
        uint16_t     fault;
        uint16_t     rtt;               // us
        uint16_t     op_idx_ack;        // op [ack/nack] , idx
        float        aux;               // get value or nack erro code

        std::ostream& dump ( std::ostream& os, const std::string delim ) const {
            os << link_pos << delim;
            os << motor_pos << delim;
            os << (float)link_vel/1000 << delim;
            os << (float)motor_vel/1000 << delim;
            os << torque << delim;
            os << temperature << delim;
            os << fault << delim;
            os << rtt << delim;
            os << op_idx_ack << delim;
            os << aux << delim;
            //os << std::endl;
            return os;
        }
        void fprint ( FILE *fp ) {
            std::ostringstream oss;
            dump(oss,"\t");
            fprintf ( fp, "%s", oss.str().c_str() );
        }
        int sprint ( char *buff, size_t size ) {
            std::ostringstream oss;
            dump(oss,"\t");
            return snprintf ( buff, size, "%s", oss.str().c_str() );
        }
        void to_map ( jmap_t & jpdo ) {
            JPDO ( link_pos );
            JPDO ( motor_pos );
            JPDO ( (float)link_vel/1000 );
            JPDO ( (float)motor_vel/1000 );
            JPDO ( torque );
            JPDO ( temperature );
            JPDO ( fault );
            JPDO ( rtt );
            JPDO ( op_idx_ack );
            JPDO ( aux );
        }
        void pb_toString( std::string * pb_str ) {
            static iit::advr::Ec_slave_pdo pb_rx_pdo;
            static struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            // Header
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_sec(ts.tv_sec);
            pb_rx_pdo.mutable_header()->mutable_stamp()->set_nsec(ts.tv_nsec);
            // Type
            pb_rx_pdo.set_type(iit::advr::Ec_slave_pdo::RX_XT_MOTOR);
            // Motor_xt_tx_pdo
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_link_pos(link_pos);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_motor_pos(motor_pos);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_link_vel((float)link_vel/1000);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_motor_vel((float)motor_vel/1000);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_torque(torque);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_temperature(temperature);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_fault(fault);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_rtt(rtt);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_op_idx_ack(op_idx_ack);
            pb_rx_pdo.mutable_motor_xt_rx_pdo()->set_aux(aux);
            pb_rx_pdo.SerializeToString(pb_str);
        }
    }  __attribute__ ( ( __packed__ ) ); // 28 bytes

}; // 56 bytes


#endif

/////////////////////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////////////////////


/*
template <typename T>
inline std::ostream& operator<< (std::ostream& os, const T& obj ) {
    return obj.dump(os,"\t");
}
*/

inline std::ostream& operator<< (std::ostream& os, const McEscPdoTypes::pdo_tx& tx_pdo ) {
    return tx_pdo.dump(os,"\t");
}

inline std::ostream& operator<< (std::ostream& os, const McEscPdoTypes::pdo_rx& rx_pdo ) {
    return rx_pdo.dump(os,"\t");
}


class PDO_aux {
public:
    PDO_aux(): sdo_objd(NULL) {}
    PDO_aux( const objd_t * sdo_obj_data ): sdo_objd( sdo_obj_data ) {}
    PDO_aux( const PDO_aux& rhs ): sdo_objd( rhs.sdo_objd ) {}
    //
    // these template methods expect [rx/tx]_pdo struct with op_idx_aux/op_idx_ack and aux fields
    // 
    template<class T>
    int on_tx( T& tx_pdo ) {
        if ( sdo_objd == 0 ) return -1;
        if ( sdo_objd->access == ATYPE_RW ) { 
            // set op
            tx_pdo.op_idx_aux = 0xFB << 8 | sdo_objd->subindex & 0xFF;
            tx_pdo.aux = *(float*)sdo_objd->data;
        } else {
            // get op
            tx_pdo.op_idx_aux = 0xBF << 8 | sdo_objd->subindex & 0xFF;
        }
        //DPRINTF("PDO_aux 0x%04X\n", tx_pdo.op_idx_aux);
        return 0;
    };
    
    template<class T>
    int on_rx( T& rx_pdo) {
        if ( sdo_objd == 0 ) return -1;
        // check nack
        if ( (rx_pdo.op_idx_ack >> 8) == 0xEE ) {
            DPRINTF("Fail PDO_aux reason 0x%02X\n", (uint32_t)rx_pdo.aux);
            return -1;
        }
        // check idx
        if ( (rx_pdo.op_idx_ack & 0xFF) != sdo_objd->subindex ) {
            DPRINTF("Fail PDO_aux idx %d != %d\n", sdo_objd->subindex, rx_pdo.op_idx_ack & 0xFF);
            return -1;
        }
        
        *(float*)sdo_objd->data = rx_pdo.aux;
        return 0;
    }
    
    int get_idx(void) { return (sdo_objd == 0) ? 0 : sdo_objd->subindex; }
    
private:
    const objd_t *  sdo_objd; 
};



inline int check_cmd_ack ( int16_t cmd, int16_t ack ) {
    if ( ack == ( ( cmd & 0x00FF ) | CTRL_CMD_DONE ) ) {
        //DPRINTF ( "DONE 0x%04X\n", cmd );
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
    c->template writeSDO_byname("ctrl_status_cmd", cmd);
    c->template readSDO_byname("ctrl_status_cmd_ack", ack);
    
    // check 
    DPRINTF("set_ctrl_status ");
    return check_cmd_ack(cmd, ack);
}

template <class C>
inline int set_flash_cmd_X(C *c, uint16_t cmd)
{
    uint16_t ack;

    cmd = cmd & 0x00FF;
    c->template writeSDO_byname ( "flash_params_cmd", cmd );
    c->template readSDO_byname ( "flash_params_cmd_ack", ack );

    // check
    DPRINTF ( "flash_params_cmd 0x%04X\n", cmd );
    return check_cmd_ack ( cmd, ack );

}


}
}
}

#endif /* IIT_ECAT_ADVR_ESC_H_ */
