#include <iit/ecat/advr/power_coman_board.h>
#include <string>

using namespace iit::ecat::advr;
using namespace iit::ecat;


static const iit::ecat::objd_t source_SDOs[] =
{
    // SD0 0x6000
    { 0x6000, 0x1, DTYPE_REAL32,      	32,  ATYPE_RO, 	 "temperature"              ,0     },
    { 0X6000, 0x2, DTYPE_REAL32,    	32,  ATYPE_RO,   "v_batt"                   ,0     },
    { 0X6000, 0x3, DTYPE_UNSIGNED16,    16,  ATYPE_RO,   "status"                   ,0     },
    { 0X6000, 0x4, DTYPE_UNSIGNED16,    16,  ATYPE_RO,   "rtt"                      ,0     },
    // SD0 0x7000                                                                         
    { 0X7000, 0x1, DTYPE_UNSIGNED16,    16,  ATYPE_RW,   "master_command"           ,0     },  
    { 0X7000, 0x2, DTYPE_UNSIGNED16,    16,  ATYPE_RW,   "ts"                       ,0     },  
    // SD0 0x8000                                                                         
    // SD0 0x8001                                                                         
    { 0x8001, 0x1, DTYPE_VISIBLE_STRING, 64, ATYPE_RO,   "firmware_version"         ,0     },
    { 0X8001, 0x2, DTYPE_REAL32,         32, ATYPE_RO,   "temperature_filtered"     ,0     },
    { 0X8001, 0x3, DTYPE_REAL32,         32, ATYPE_RO,   "v_batt_filtered"          ,0     },
    { 0X8001, 0x4, DTYPE_REAL32,         32, ATYPE_RO,   "v_pack_filtered"          ,0     },
    { 0x8001, 0x5, DTYPE_UNSIGNED16,     16, ATYPE_RW,   "ctrl_status_cmd"          ,0     },
    { 0x8001, 0x6, DTYPE_UNSIGNED16,     16, ATYPE_RO,   "ctrl_status_cmd_ack"      ,0     },
    
    {0, 0, 0, 0, 0, 0, 0 }

};


void PowCmnESC::init_SDOs(void) {

    int objd_num, i = 0;

    objd_num = sizeof(source_SDOs)/sizeof(objd_t);
    SDOs = new objd_t [objd_num];

    memcpy((void*)SDOs, source_SDOs, sizeof(source_SDOs));

    // 0x6000 
    SDOs[i++].data = (void*)&PowCmnESC::rx_pdo.temperature;
    SDOs[i++].data = (void*)&PowCmnESC::rx_pdo.v_batt;       
    SDOs[i++].data = (void*)&PowCmnESC::rx_pdo.status;       
    SDOs[i++].data = (void*)&PowCmnESC::rx_pdo.rtt;            
    // 0x7000                                
    SDOs[i++].data = (void*)&PowCmnESC::tx_pdo.master_command;        
    SDOs[i++].data = (void*)&PowCmnESC::tx_pdo.ts;             
    // 0x8000
    // 0x8001
    SDOs[i++].data = (void*)&PowCmnESC::sdo.firmware_version;    
    SDOs[i++].data = (void*)&PowCmnESC::sdo.temperature_filtered;     
    SDOs[i++].data = (void*)&PowCmnESC::sdo.v_batt_filtered; 
    SDOs[i++].data = (void*)&PowCmnESC::sdo.v_pack_filtered; 
    SDOs[i++].data = (void*)&PowCmnESC::sdo.ctrl_status_cmd;    
    SDOs[i++].data = (void*)&PowCmnESC::sdo.ctrl_status_cmd_ack;
    // end marker
    SDOs[i++].data = 0;

    assert ( objd_num == i );
}
