#include <iit/ecat/advr/mc_lowpwr_esc.h>
#include <string>

using namespace iit::ecat;
using namespace iit::ecat::advr;

char acName1000[] = "Device Type";
char acName1000_0[] = "Device Type";
char acName1008[] = "Manufacturer Device Name";
char acName1008_0[] = "Manufacturer Device Name";
char acName1009[] = "Manufacturer Hardware Version";
char acName1009_0[] = "Manufacturer Hardware Version";
char acName100A[] = "Manufacturer Software Version";
char acName100A_0[] = "Manufacturer Software Version";
char acName1018[] = "Identity Object";
char acName1018_0[] = "Number of Elements";
char acName1018_1[] = "Vendor ID";
char acName1018_2[] = "Product Code";
char acName1018_3[] = "Revision Number";
char acName1018_4[] = "Serial Number";
char acName1600[] = "Receive PDO Mapping";
char acName1600_0[] = "Number of Elements";
char acName1600_n[] = "Mapped Object";
char acName1A00[] = "Transmit PDO Mapping";
char acName1A00_0[] = "Number of Elements";
char acName1A00_n[] = "Mapped Object";
char acName1C00[] = "Sync Manager Communication Type";
char acName1C00_0[] = "Number of Elements";
char acName1C00_1[] = "Communications Type SM0";
char acName1C00_2[] = "Communications Type SM1";
char acName1C00_3[] = "Communications Type SM2";
char acName1C00_4[] = "Communications Type SM3";
char acName1C12[] = "Sync Manager 2 PDO Assignment";
char acName1C12_0[] = "Number of Elements";
char acName1C12_1[] = "PDO Mapping";
char acName1C13[] = "Sync Manager 3 PDO Assignment";
char acName1C13_0[] = "Number of Elements";
char acName1C13_1[] = "PDO Mapping";

char acName6000[] = "Inputs";
char acName6000_0[] = "Number of Elements";
char acName6000_1[] = "rel_enc";
char acName6000_2[] = "abs_enc";
char acName6000_3[] = "adc";
char acName6000_4[] = "ain";
char acName6000_5[] = "hall";
char acName6000_6[] = "pwm";
char acName6000_rtt[] = "rtt";
char acName6000_pid_out[] = "pid_out";
char acName6000_pos[] = "position";
char acName6000_vel[] = "velocity";
char acName6000_tor[] = "torque";
char acName6000_tor_D[] = "tor_D";
char acName6000_curr[] = "curr";
char acName6000_temp[] = "temperature";
char acName6000_fault[] = "fault";

char acName7000[] = "Outputs";
char acName7000_0[] = "Number of Elements";
char acName7000_1[] = "pos_ref";
char acName7000_2[] = "tor_offs";
char acName7000_3[] = "PGain";
char acName7000_4[] = "IGain";
char acName7000_5[] = "DGain";
char acName7000_6[] = "ts";

char acName8000[] = "Flash Parameter";
char acName8000_0[] = "Number of Elements";
char acName8000_1[] = "Block control";
char acName8000_2[] = "nonius offset low";
char acName8000_3[] = "pos_gain_P";
char acName8000_4[] = "pos_gain_I";
char acName8000_5[] = "pos_gain_D";
char acName8000_6[] = "tor_gain_P";
char acName8000_7[] = "tor_gain_I";
char acName8000_8[] = "tor_gain_D";
char acName8000_9[] = "Torque_Mult";
char acName8000_10[] = "pos_integral_limit";
char acName8000_11[] = "tor_integral_limit";
char acName8000_12[] = "Min_pos";
char acName8000_13[] = "Max_pos";
char acName8000_14[] = "nonius offset high";
char acName8000_15[] = "max_tor";
char acName8000_16[] = "max_cur";
char acName8000_17[] = "Enc_offset_1";
char acName8000_18[] = "Enc_offset_2";
char acName8000_19[] = "Torque_Offset";
char acName8000_20[] = "ConfigFlags";
char acName8000_21[] = "ConfigFlags2";
char acName8000_22[] = "NumEncoderLines";
char acName8000_23[] = "ImpedancePosGainP";
char acName8000_24[] = "nonius offset2 low";
char acName8000_25[] = "ImpedancePosGainD";
char acName8000_26[] = "Num_Abs_counts_rev";
char acName8000_27[] = "MaxPWM";
char acName8000_28[] = "Gearbox_ratio";
char acName8000_29[] = "ulCalPosition";
char acName8000_30[] = "Cal_Abs_Position";
char acName8000_31[] = "Cal_Abs2_Position";
char acName8000_32[] = "nonius offset2 high";
char acName8000_33[] = "Joint_number";
char acName8000_34[] = "Joint_robot_id";
char acName8000_35[] = "Target velocity";

char acName8001[] = "Parameter";
char acName8001_1[] = "fw_ver";
char acName8001_2[] = "ack_board_faults";
char acName8001_3[] = "ctrl_status_cmd";
char acName8001_4[] = "ctrl_status_cmd_ack";
char acName8001_5[] = "V_batt";
char acName8001_6[] = "T_inv";
char acName8001_7[] = "T_mot";
char acName8001_8[] = "flash_params_cmd";
char acName8001_9[] = "flash_params_cmd_ack";


//template<class EscPDOTypes, class EscSDOTypes>
//typename BasicEscWrapper<EscPDOTypes, EscSDOTypes>::sdo_t    BasicEscWrapper<EscPDOTypes, EscSDOTypes>::sdo;

static const iit::ecat::objd_t source_SDOs[] =
{

    // SDO6000[] =
    {0x6000, 0x1, DTYPE_REAL32,      32, ATYPE_RO, acName6000_temp        ,0   }, 
    {0x6000, 0x2, DTYPE_REAL32,      32, ATYPE_RO, acName6000_pos         ,0   }, 
    {0x6000, 0x3, DTYPE_REAL32,      32, ATYPE_RO, acName6000_vel         ,0   }, 
    {0x6000, 0x4, DTYPE_REAL32,      32, ATYPE_RO, acName6000_tor         ,0   }, 
    {0x6000, 0x5, DTYPE_INTEGER32,   32, ATYPE_RO, acName6000_fault       ,0   }, 
    {0x6000, 0x6, DTYPE_UNSIGNED64,  64, ATYPE_RO, acName6000_rtt         ,0   }, 
    // SDO7000[] =                                                        
    {0x7000, 0x1, DTYPE_REAL32,     32, ATYPE_RW, acName7000_1            ,0   },
//     {0x7000, 0x2, DTYPE_REAL32,     32, ATYPE_RW, acName7000_2            ,0   },
//     {0x7000, 0x3, DTYPE_REAL32,     32, ATYPE_RW, acName7000_3            ,0   },
//     {0x7000, 0x4, DTYPE_REAL32,     32, ATYPE_RW, acName7000_4            ,0   },
//     {0x7000, 0x5, DTYPE_REAL32,     32, ATYPE_RW, acName7000_5            ,0   },
    {0x7000, 0x2, DTYPE_UNSIGNED64, 64, ATYPE_RW, acName7000_6            ,0   },
    // SDO8000[] =                                                          
    {0x8000, 0x1, DTYPE_INTEGER32,     32, ATYPE_RW, acName8000_1         ,0   }, 
    {0x8000, 0x2, DTYPE_INTEGER32,     32, ATYPE_RW, acName8000_2         ,0   }, 
    {0x8000, 0x3, DTYPE_REAL32,        32, ATYPE_RW, acName8000_3         ,0   }, 
    {0x8000, 0x4, DTYPE_REAL32,        32, ATYPE_RW, acName8000_4         ,0   }, 
    {0x8000, 0x5, DTYPE_REAL32,        32, ATYPE_RW, acName8000_5         ,0   }, 
    {0x8000, 0x6, DTYPE_REAL32,        32, ATYPE_RW, acName8000_6         ,0   }, 
    {0x8000, 0x7, DTYPE_REAL32,        32, ATYPE_RW, acName8000_7         ,0   }, 
    {0x8000, 0x8, DTYPE_REAL32,        32, ATYPE_RW, acName8000_8         ,0   }, 
    {0x8000, 0x9, DTYPE_REAL32,        32, ATYPE_RW, acName8000_9         ,0   }, 
    {0x8000, 0xa, DTYPE_REAL32,        32, ATYPE_RW, acName8000_10        ,0   }, 
    {0x8000, 0xb, DTYPE_REAL32,        32, ATYPE_RW, acName8000_11        ,0   }, 
    {0x8000, 0xc, DTYPE_REAL32,        32, ATYPE_RW, acName8000_12        ,0   }, 
    {0x8000, 0xd, DTYPE_REAL32,        32, ATYPE_RW, acName8000_13        ,0   }, 
    {0x8000, 0xe, DTYPE_INTEGER32,     32, ATYPE_RW, acName8000_14        ,0   }, 
    {0x8000, 0xf, DTYPE_REAL32,        32, ATYPE_RW, acName8000_15        ,0   }, 
    {0x8000, 0x10, DTYPE_REAL32,       32, ATYPE_RW, acName8000_16        ,0   }, 
    {0x8000, 0x11, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_17        ,0   }, 
    {0x8000, 0x12, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_18        ,0   }, 
    {0x8000, 0x13, DTYPE_REAL32,       32, ATYPE_RW, acName8000_19        ,0   }, 
    {0x8000, 0x14, DTYPE_INTEGER16,    16, ATYPE_RW, acName8000_20        ,0   }, 
    {0x8000, 0x15, DTYPE_INTEGER16,    16, ATYPE_RW, acName8000_21        ,0   }, 
    {0x8000, 0x16, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_22        ,0   }, 
    {0x8000, 0x17, DTYPE_REAL32,       32, ATYPE_RW, acName8000_23        ,0   }, 
    {0x8000, 0x18, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_24        ,0   }, 
    {0x8000, 0x19, DTYPE_REAL32,       32, ATYPE_RW, acName8000_25        ,0   }, 
    {0x8000, 0x1a, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_26        ,0   }, 
    {0x8000, 0x1b, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_27        ,0   }, 
    {0x8000, 0x1c, DTYPE_REAL32,       32, ATYPE_RW, acName8000_28        ,0   }, 
    {0x8000, 0x1d, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_29        ,0   }, 
    {0x8000, 0x1e, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_30        ,0   }, 
    {0x8000, 0x1f, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_31        ,0   }, 
    {0x8000, 0x20, DTYPE_INTEGER32,    32, ATYPE_RW, acName8000_32        ,0   }, 
    {0x8000, 0x21,DTYPE_INTEGER16,     16, ATYPE_RW, acName8000_33        ,0   }, 
    {0x8000, 0x22,DTYPE_INTEGER16,     16, ATYPE_RW, acName8000_34        ,0   }, 
                                                                        
    // SDO8001[] =                                                      
    {0x8001, 0x1, DTYPE_VISIBLE_STRING,   64, ATYPE_RO, acName8001_1      ,0   }, 
    {0x8001, 0x2, DTYPE_INTEGER32,        32, ATYPE_RW, acName8001_2      ,0   }, 
    {0x8001, 0x3, DTYPE_INTEGER16,        16, ATYPE_RW, acName8001_3      ,0   }, 
    {0x8001, 0x4, DTYPE_INTEGER16,        16, ATYPE_RO, acName8001_4      ,0   }, 
    {0x8001, 0x5, DTYPE_REAL32,           32, ATYPE_RO, acName8001_5      ,0   }, 
    {0x8001, 0x6, DTYPE_REAL32,           32, ATYPE_RO, acName8001_6      ,0   }, 
    {0x8001, 0x7, DTYPE_REAL32,           32, ATYPE_RO, acName8001_7      ,0   }, 
    {0x8001, 0x8, DTYPE_INTEGER16,        16, ATYPE_RW, acName8001_8      ,0   }, 
    {0x8001, 0x9, DTYPE_INTEGER16,        16, ATYPE_RO, acName8001_9      ,0   }, 
                                                                      
    {0, 0, 0, 0, 0, 0, 0 }                                                
};                                                                    
                                                                      
                                                                      
                                                                      
                                                                      
                                                                      
void LpESC::init_SDOs(void) {                                         
                                                                      
    int objd_num, i = 0;                                              
                                                                      
    objd_num = sizeof(source_SDOs)/sizeof(objd_t);                    
    SDOs = new objd_t [objd_num];                                     
                                                                      
    memcpy((void*)SDOs, source_SDOs, sizeof(source_SDOs));            

    // 0x6000        
    SDOs[i++].data = (void*)&LpESC::rx_pdo.temperature; 
    SDOs[i++].data = (void*)&LpESC::rx_pdo.position;       
    SDOs[i++].data = (void*)&LpESC::rx_pdo.velocity;       
    SDOs[i++].data = (void*)&LpESC::rx_pdo.torque;         
    SDOs[i++].data = (void*)&LpESC::rx_pdo.fault;            
    SDOs[i++].data = (void*)&LpESC::rx_pdo.rtt;            
    // 0x7000        
    SDOs[i++].data = (void*)&LpESC::tx_pdo.pos_ref; 
//     SDOs[i++].data = (void*)&LpESC::tx_pdo.tor_offs;
//     SDOs[i++].data = (void*)&LpESC::tx_pdo.PosGainP;
//     SDOs[i++].data = (void*)&LpESC::tx_pdo.PosGainI;
//     SDOs[i++].data = (void*)&LpESC::tx_pdo.PosGainD;
    SDOs[i++].data = (void*)&LpESC::tx_pdo.ts;      
    // 0x8000
    SDOs[i++].data = (void*)&LpESC::sdo.Block_control;      
    SDOs[i++].data = (void*)&LpESC::sdo.nonius_offset_low;  
    SDOs[i++].data = (void*)&LpESC::sdo.PosGainP;           
    SDOs[i++].data = (void*)&LpESC::sdo.PosGainI;           
    SDOs[i++].data = (void*)&LpESC::sdo.PosGainD;           
    SDOs[i++].data = (void*)&LpESC::sdo.TorGainP;           
    SDOs[i++].data = (void*)&LpESC::sdo.TorGainI;           
    SDOs[i++].data = (void*)&LpESC::sdo.TorGainD;           
    SDOs[i++].data = (void*)&LpESC::sdo.Torque_Mult;        
    SDOs[i++].data = (void*)&LpESC::sdo.Pos_I_lim;          
    SDOs[i++].data = (void*)&LpESC::sdo.Tor_I_lim;          
    SDOs[i++].data = (void*)&LpESC::sdo.Min_pos;            
    SDOs[i++].data = (void*)&LpESC::sdo.Max_pos;            
    SDOs[i++].data = (void*)&LpESC::sdo.nonius_offset_high; 
    SDOs[i++].data = (void*)&LpESC::sdo.Max_tor;            
    SDOs[i++].data = (void*)&LpESC::sdo.Max_cur;            
    SDOs[i++].data = (void*)&LpESC::sdo.Enc_offset_1;       
    SDOs[i++].data = (void*)&LpESC::sdo.Enc_offset_2;       
    SDOs[i++].data = (void*)&LpESC::sdo.Torque_Offset;      
    SDOs[i++].data = (void*)&LpESC::sdo.ConfigFlags;        
    SDOs[i++].data = (void*)&LpESC::sdo.ConfigFlags2;       
    SDOs[i++].data = (void*)&LpESC::sdo.NumEncoderLines;    
    SDOs[i++].data = (void*)&LpESC::sdo.ImpedancePosGainP;  
    SDOs[i++].data = (void*)&LpESC::sdo.nonius_offset2_low; 
    SDOs[i++].data = (void*)&LpESC::sdo.ImpedancePosGainD;  
    SDOs[i++].data = (void*)&LpESC::sdo.Num_Abs_counts_rev; 
    SDOs[i++].data = (void*)&LpESC::sdo.MaxPWM;             
    SDOs[i++].data = (void*)&LpESC::sdo.Gearbox_ratio;      
    SDOs[i++].data = (void*)&LpESC::sdo.ulCalPosition;      
    SDOs[i++].data = (void*)&LpESC::sdo.Cal_Abs_Position;   
    SDOs[i++].data = (void*)&LpESC::sdo.Cal_Abs2_Position;  
    SDOs[i++].data = (void*)&LpESC::sdo.nonius_offset2_high;
    SDOs[i++].data = (void*)&LpESC::sdo.Joint_number;
    SDOs[i++].data = (void*)&LpESC::sdo.Joint_robot_id;
    // 0x8001
    SDOs[i++].data = (void*)&LpESC::sdo.firmware_version;     
    SDOs[i++].data = (void*)&LpESC::sdo.ack_board_fault;      
    SDOs[i++].data = (void*)&LpESC::sdo.set_ctrl_status;      
    SDOs[i++].data = (void*)&LpESC::sdo.get_ctrl_status;      
    SDOs[i++].data = (void*)&LpESC::sdo.V_batt_filt_100ms;    
    SDOs[i++].data = (void*)&LpESC::sdo.T_inv_filt_100ms;     
    SDOs[i++].data = (void*)&LpESC::sdo.T_mot1_filt_100ms;    
    SDOs[i++].data = (void*)&LpESC::sdo.flash_params_cmd;     
    SDOs[i++].data = (void*)&LpESC::sdo.flash_params_cmd_ack; 
    // end marker
    SDOs[i++].data = 0;

    assert ( objd_num == i );
}

