#include <iit/ecat/advr/power_board.h>
#include <string>

using namespace iit::ecat::advr;
using namespace iit::ecat;


static const iit::ecat::objd_t source_SDOs[] =
{
    // SD0 0x6000
    { 0X6000, 0x1, DTYPE_UNSIGNED16,    16,  ATYPE_RO,   "status"                   ,0     },
    { 0X6000, 0x2, DTYPE_UNSIGNED16,    16,  ATYPE_RO,   "board_temp"               ,0     },
    { 0X6000, 0x3, DTYPE_UNSIGNED16,    16,  ATYPE_RO,   "battery_temp"             ,0     },
    { 0X6000, 0x4, DTYPE_UNSIGNED16,    16,  ATYPE_RO,   "battery_volt"             ,0     },
    { 0X6000, 0x5, DTYPE_INTEGER16,     16,  ATYPE_RO,   "battery_curr"             ,0     },
    { 0X6000, 0x6, DTYPE_INTEGER16,     16,  ATYPE_RO,   "load_curr"                ,0     },
    { 0X6000, 0x7, DTYPE_UNSIGNED16,    16,  ATYPE_RO,   "fault"                    ,0     },
    { 0X6000, 0x8, DTYPE_UNSIGNED16,    16,  ATYPE_RO,   "rtt"                      ,0     },
    // SD0 0x7000                                                                         
    { 0X7000, 0x1, DTYPE_UNSIGNED16,    16,  ATYPE_RW,   "master_cmd"               ,0     },  
    { 0X7000, 0x2, DTYPE_UNSIGNED16,    16,  ATYPE_RW,   "fault_ack"                ,0     },  
    { 0X7000, 0x3, DTYPE_UNSIGNED16,    16,  ATYPE_RW,   "ts"                       ,0     },  
    // SD0 0x8000                                                                         
    // SD0 0x8001                                                                         
    { 0x8001, 0x1, DTYPE_VISIBLE_STRING,64, ATYPE_RO,    "firmware_version"         ,0     },
    { 0x8001, 0x2, DTYPE_UNSIGNED16,     16, ATYPE_RW,    "ctrl_status_cmd"          ,0     },
    { 0x8001, 0x3, DTYPE_UNSIGNED16,     16, ATYPE_RO,    "ctrl_status_cmd_ack"      ,0     },
    { 0x8001, 0x4, DTYPE_UNSIGNED16,     16, ATYPE_RW,    "flash_params_cmd"         ,0     },
    { 0x8001, 0x5, DTYPE_UNSIGNED16,     16, ATYPE_RO,    "flash_params_cmd_ack"     ,0     },
    { 0x8001, 0x6, DTYPE_UNSIGNED16,     16, ATYPE_RO,    "v_pack_adc"               ,0     },
    { 0x8001, 0x7, DTYPE_UNSIGNED16,     16, ATYPE_RO,    "v_batt_adc"               ,0     },
    { 0x8001, 0x8, DTYPE_UNSIGNED16,     16, ATYPE_RO,    "i_batt_adc"               ,0     },
    { 0x8001, 0x9, DTYPE_UNSIGNED16,     16, ATYPE_RO,    "i_load_adc"               ,0     },
    { 0x8001, 0xa, DTYPE_UNSIGNED16,     16, ATYPE_RO,    "t_batt_adc"               ,0     },
    { 0x8001, 0xb, DTYPE_UNSIGNED16,     16, ATYPE_RO,    "t_board_adc"              ,0     },
    { 0x8001, 0xc, DTYPE_UNSIGNED16,     16, ATYPE_RO,    "FSM"        ,0     },
    { 0x8001, 0xd, DTYPE_REAL32,         32, ATYPE_RO,    "v_batt_filt"        ,0     },
    { 0x8001, 0xe, DTYPE_REAL32,         32, ATYPE_RO,    "v_pack_filt"        ,0     },

    {0, 0, 0, 0, 0, 0, 0 }


};


void PowESC::init_SDOs(void) {

    int objd_num, i = 0;

    objd_num = sizeof(source_SDOs)/sizeof(objd_t);
    SDOs = new objd_t [objd_num];

    memcpy((void*)SDOs, source_SDOs, sizeof(source_SDOs));

    // 0x6000 
    SDOs[i++].data = (void*)&PowESC::rx_pdo.status;
    SDOs[i++].data = (void*)&PowESC::rx_pdo.board_temp;       
    SDOs[i++].data = (void*)&PowESC::rx_pdo.battery_temp;       
    SDOs[i++].data = (void*)&PowESC::rx_pdo.battery_volt;
    SDOs[i++].data = (void*)&PowESC::rx_pdo.battery_curr;
    SDOs[i++].data = (void*)&PowESC::rx_pdo.load_curr;
    SDOs[i++].data = (void*)&PowESC::rx_pdo.fault;          
    SDOs[i++].data = (void*)&PowESC::rx_pdo.rtt;            
    // 0x7000                                
    SDOs[i++].data = (void*)&PowESC::tx_pdo.master_command;        
    SDOs[i++].data = (void*)&PowESC::tx_pdo.fault_ack;       
    SDOs[i++].data = (void*)&PowESC::tx_pdo.ts;             
    // 0x8000
    // 0x8001
    SDOs[i++].data = (void*)&PowESC::sdo.firmware_version;    
    SDOs[i++].data = (void*)&PowESC::sdo.ctrl_status_cmd;     
    SDOs[i++].data = (void*)&PowESC::sdo.ctrl_status_cmd_ack; 
    SDOs[i++].data = (void*)&PowESC::sdo.flash_params_cmd;    
    SDOs[i++].data = (void*)&PowESC::sdo.flash_params_cmd_ack;
    SDOs[i++].data = (void*)&PowESC::sdo.v_pack_adc;         
    SDOs[i++].data = (void*)&PowESC::sdo.v_batt_adc;        
    SDOs[i++].data = (void*)&PowESC::sdo.i_batt_adc;       
    SDOs[i++].data = (void*)&PowESC::sdo.i_load_adc;      
    SDOs[i++].data = (void*)&PowESC::sdo.t_batt_adc;      
    SDOs[i++].data = (void*)&PowESC::sdo.t_board_adc;              
    SDOs[i++].data = (void*)&PowESC::sdo.FSM;              
    SDOs[i++].data = (void*)&PowESC::sdo.v_batt_filt;              
    SDOs[i++].data = (void*)&PowESC::sdo.v_pack_filt;              
    // end marker
    SDOs[i++].data = 0;

    assert ( objd_num == i );
}
