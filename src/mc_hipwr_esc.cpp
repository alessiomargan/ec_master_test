#include <iit/ecat/advr/mc_hipwr_esc.h>
#include <string>

using namespace iit::ecat::advr;
using namespace iit::ecat;


static const iit::ecat::objd_t source_SDOs[] =
{
    // SD0 0x6000
    { 0X6000, 0x1, DTYPE_REAL32,        32,  ATYPE_RO,   "position"                 ,0     },
    { 0X6000, 0x2, DTYPE_REAL32,        32,  ATYPE_RO,   "pos_ref_fb"               ,0     },
    { 0X6000, 0x3, DTYPE_UNSIGNED16,    16,  ATYPE_RO,   "temperature"              ,0     },
    { 0X6000, 0x4, DTYPE_INTEGER16,     16,  ATYPE_RO,   "torque"                   ,0     },
    { 0X6000, 0x5, DTYPE_UNSIGNED16,    16,  ATYPE_RO,   "fault"                    ,0     },
    { 0X6000, 0x6, DTYPE_UNSIGNED16,    16,  ATYPE_RO,   "rtt"                      ,0     },
    // SD0 0x7000                                                                         
    { 0X7000, 0x1, DTYPE_REAL32,        32,  ATYPE_RW,   "pos_ref"                  ,0     },  
    { 0X7000, 0x2, DTYPE_UNSIGNED16,    16,  ATYPE_RW,   "fault_ack"                ,0     },  
    { 0X7000, 0x3, DTYPE_UNSIGNED16,    16,  ATYPE_RW,   "gainP"                    ,0     },  
    { 0X7000, 0x4, DTYPE_UNSIGNED16,    16,  ATYPE_RW,   "gainD"                    ,0     },  
    { 0X7000, 0x5, DTYPE_UNSIGNED16,    16,  ATYPE_RW,   "ts"                       ,0     },  
    // SD0 0x8000                                                                         
    { 0x8000, 0x1, DTYPE_REAL32,        32, ATYPE_RW,    "Sensor_type"              ,0     },
    { 0x8000, 0x2, DTYPE_REAL32,        32, ATYPE_RW,    "PosGainP"                 ,0     },
    { 0x8000, 0x3, DTYPE_REAL32,        32, ATYPE_RW,    "PosGainI"                 ,0     },
    { 0x8000, 0x4, DTYPE_REAL32,        32, ATYPE_RW,    "PosGainD"                 ,0     },
    { 0x8000, 0x5, DTYPE_REAL32,        32, ATYPE_RW,    "TorGainP"                 ,0     },
    { 0x8000, 0x6, DTYPE_REAL32,        32, ATYPE_RW,    "TorGainI"                 ,0     },
    { 0x8000, 0x7, DTYPE_REAL32,        32, ATYPE_RW,    "TorGainD"                 ,0     },
    { 0x8000, 0x8, DTYPE_REAL32,        32, ATYPE_RW,    "TorGainFF"                ,0     },
    { 0x8000, 0x9, DTYPE_REAL32,        32, ATYPE_RW,    "Pos_I_lim"                ,0     },
    { 0x8000, 0xa, DTYPE_REAL32,       32, ATYPE_RW,    "Tor_I_lim"                ,0     },
    { 0x8000, 0xb, DTYPE_REAL32,        32, ATYPE_RW,    "Min_pos"                  ,0     },
    { 0x8000, 0xc, DTYPE_REAL32,        32, ATYPE_RW,    "Max_pos"                  ,0     },
    { 0x8000, 0xd, DTYPE_REAL32,        32, ATYPE_RW,    "Max_vel"                  ,0     },
    { 0x8000, 0xe, DTYPE_REAL32,        32, ATYPE_RW,    "Max_tor"                  ,0     },
    { 0x8000, 0xf, DTYPE_REAL32,        32, ATYPE_RW,    "Max_cur"                  ,0     },
    { 0x8000, 0x10, DTYPE_REAL32,        32, ATYPE_RO,    "Enc_offset"               ,0     },
    { 0x8000, 0x11, DTYPE_REAL32,        32, ATYPE_RO,    "Enc_relative_offset"      ,0     },
    { 0x8000, 0x12, DTYPE_REAL32,        32, ATYPE_RW,    "Calibration_angle"        ,0     },
    { 0x8000, 0x13,DTYPE_REAL32,        32, ATYPE_RW,    "Torque_lin_coeff"         ,0     },
    { 0x8000, 0x14,DTYPE_UNSIGNED64,    64, ATYPE_RW,    "Enc_mot_nonius_calib"     ,0     },
    { 0x8000, 0x15,DTYPE_UNSIGNED64,    64, ATYPE_RW,    "Enc_load_nonius_calib"    ,0     },
    { 0x8000, 0x16,DTYPE_INTEGER16,     16, ATYPE_RW,    "Joint_number"             ,0     },
    { 0x8000, 0x17,DTYPE_INTEGER16,     16, ATYPE_RW,    "Joint_robot_id"           ,0     },
                                                                                          
    // SD0 0x8001                                                                         
    { 0x8001, 0x1, DTYPE_VISIBLE_STRING,64, ATYPE_RO,    "firmware_version"  	    ,0     },
    { 0x8001, 0x2, DTYPE_INTEGER32,     32, ATYPE_RW,    "enable_pdo_gains"         ,0     },
    { 0x8001, 0x3, DTYPE_REAL32,        32, ATYPE_RW,    "Direct_ref"  			    ,0     },
    { 0x8001, 0x4, DTYPE_REAL32,        32, ATYPE_RO,    "V_batt_filt_100ms" 	    ,0     },
    { 0x8001, 0x5, DTYPE_REAL32,        32, ATYPE_RO,    "Board_Temperature"  	    ,0     },
    { 0x8001, 0x6, DTYPE_REAL32,        32, ATYPE_RO,    "T_mot1_filt_100ms"  	    ,0     },
    { 0x8001, 0x7, DTYPE_INTEGER16,     16, ATYPE_RW,    "ctrl_status_cmd"   	    ,0     },
    { 0x8001, 0x8, DTYPE_INTEGER16,     16, ATYPE_RO,    "ctrl_status_cmd_ack" 	    ,0     },
    { 0x8001, 0x9, DTYPE_INTEGER16,     16, ATYPE_RW,    "flash_params_cmd" 	    ,0     },
    { 0x8001, 0xa, DTYPE_INTEGER16,     16, ATYPE_RO,    "flash_params_cmd_ack"     ,0     },
    { 0x8001, 0xb, DTYPE_REAL32,        32, ATYPE_RO,    "abs_enc_mot" 	  		    ,0     },
    { 0x8001, 0xc, DTYPE_REAL32,        32, ATYPE_RO,    "abs_enc_load"  		    ,0     },
    { 0x8001, 0xd, DTYPE_REAL32,        32, ATYPE_RO, 	 "angle_enc_mot" 		    ,0     },
    { 0x8001, 0xe, DTYPE_REAL32,        32, ATYPE_RO,	 "angle_enc_load" 		    ,0     },
    { 0x8001, 0xf, DTYPE_REAL32,        32, ATYPE_RO, 	 "angle_enc_diff" 		    ,0     },
    { 0x8001, 0x10,DTYPE_REAL32,        32, ATYPE_RO,	 "iq_ref" 		            ,0     },
    
    {0, 0, 0, 0, 0, 0, 0 }


};


void HpESC::init_SDOs(void) {

    int objd_num, i = 0;

    objd_num = sizeof(source_SDOs)/sizeof(objd_t);
    SDOs = new objd_t [objd_num];

    memcpy((void*)SDOs, source_SDOs, sizeof(source_SDOs));

    // 0x6000 
    SDOs[i++].data = (void*)&HpESC::rx_pdo.position;
    SDOs[i++].data = (void*)&HpESC::rx_pdo.pos_ref_fb;       
    SDOs[i++].data = (void*)&HpESC::rx_pdo.temperature;       
    SDOs[i++].data = (void*)&HpESC::rx_pdo.torque;         
    SDOs[i++].data = (void*)&HpESC::rx_pdo.fault;          
    SDOs[i++].data = (void*)&HpESC::rx_pdo.rtt;            
    // 0x7000                                
    SDOs[i++].data = (void*)&HpESC::tx_pdo.pos_ref;        
    SDOs[i++].data = (void*)&HpESC::tx_pdo.fault_ack;       
    SDOs[i++].data = (void*)&HpESC::tx_pdo.gainP;       
    SDOs[i++].data = (void*)&HpESC::tx_pdo.gainD;       
    SDOs[i++].data = (void*)&HpESC::tx_pdo.ts;             
    // 0x8000
    SDOs[i++].data = (void*)&HpESC::sdo.Sensor_type;
    SDOs[i++].data = (void*)&HpESC::sdo.PosGainP;            
    SDOs[i++].data = (void*)&HpESC::sdo.PosGainI;            
    SDOs[i++].data = (void*)&HpESC::sdo.PosGainD;            
    SDOs[i++].data = (void*)&HpESC::sdo.TorGainP;            
    SDOs[i++].data = (void*)&HpESC::sdo.TorGainI;            
    SDOs[i++].data = (void*)&HpESC::sdo.TorGainD;            
    SDOs[i++].data = (void*)&HpESC::sdo.TorGainFF;           
    SDOs[i++].data = (void*)&HpESC::sdo.Pos_I_lim;           
    SDOs[i++].data = (void*)&HpESC::sdo.Tor_I_lim;           
    SDOs[i++].data = (void*)&HpESC::sdo.Min_pos;             
    SDOs[i++].data = (void*)&HpESC::sdo.Max_pos;             
    SDOs[i++].data = (void*)&HpESC::sdo.Max_vel;             
    SDOs[i++].data = (void*)&HpESC::sdo.Max_tor;             
    SDOs[i++].data = (void*)&HpESC::sdo.Max_cur;             
    SDOs[i++].data = (void*)&HpESC::sdo.Enc_offset;          
    SDOs[i++].data = (void*)&HpESC::sdo.Enc_relative_offset; 
    SDOs[i++].data = (void*)&HpESC::sdo.Phase_angle;         
    SDOs[i++].data = (void*)&HpESC::sdo.Torque_lin_coeff;   
    SDOs[i++].data = (void*)&HpESC::sdo.Enc_mot_nonius_calib;
    SDOs[i++].data = (void*)&HpESC::sdo.Enc_load_nonius_calib;
    SDOs[i++].data = (void*)&HpESC::sdo.Joint_number;
    SDOs[i++].data = (void*)&HpESC::sdo.Joint_robot_id; 
    // 0x8001
    SDOs[i++].data = (void*)&HpESC::sdo.firmware_version;    
    SDOs[i++].data = (void*)&HpESC::sdo.enable_pdo_gains; 
    SDOs[i++].data = (void*)&HpESC::sdo.Direct_ref;          
    SDOs[i++].data = (void*)&HpESC::sdo.V_batt_filt_100ms;   
    SDOs[i++].data = (void*)&HpESC::sdo.Board_Temperature;   
    SDOs[i++].data = (void*)&HpESC::sdo.T_mot1_filt_100ms;   
    SDOs[i++].data = (void*)&HpESC::sdo.ctrl_status_cmd;     
    SDOs[i++].data = (void*)&HpESC::sdo.ctrl_status_cmd_ack; 
    SDOs[i++].data = (void*)&HpESC::sdo.flash_params_cmd;    
    SDOs[i++].data = (void*)&HpESC::sdo.flash_params_cmd_ack;
    SDOs[i++].data = (void*)&HpESC::sdo.abs_enc_mot;         
    SDOs[i++].data = (void*)&HpESC::sdo.abs_enc_load;        
    SDOs[i++].data = (void*)&HpESC::sdo.angle_enc_mot;       
    SDOs[i++].data = (void*)&HpESC::sdo.angle_enc_load;      
    SDOs[i++].data = (void*)&HpESC::sdo.angle_enc_diff;      
    SDOs[i++].data = (void*)&HpESC::sdo.iq_ref;              
    // end marker
    SDOs[i++].data = 0;

    assert ( objd_num == i );
}
