#include <signal.h>
#include <sys/mman.h>
#include <execinfo.h>
#include <stdio.h>
#include <stdint.h>
#include <yaml-cpp/yaml.h>


#include <iostream>
#include <memory>
#include <numeric>

#include <iit/ecat/advr/ec_boards_iface.h>


using namespace iit::ecat;
using namespace iit::ecat::advr;


///////////////////////////////////////////////////////////////////////////////


Ec_Boards_ctrl * ec_boards_ctrl;

extern void main_common ( int *argcp, char *const **argvp, __sighandler_t sig_handler );

static int main_loop = 1;

void shutdown ( int sig __attribute__ ( ( unused ) ) ) {
    main_loop = 0;
    printf ( "got signal .... Shutdown\n" );
}

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main ( int argc, char * const argv[] ) try {

    if ( argc != 2 ) {
        printf ( "Usage: %s config.yaml\n", argv[0] );
        return 0;
    }

    main_common ( &argc, &argv, shutdown );
    
    ///////////////////////////////////////////////////////////////////////////

    Ec_Boards_ctrl * ec_boards_ctrl = new Ec_Boards_ctrl ( argv[1] );

    if ( ec_boards_ctrl->init() != EC_BOARD_OK ) {
        std::cout << "Error in boards init()... cannot proceed!" << std::endl;
        delete ec_boards_ctrl;
        return 0;
    }

    Rid2PosMap  rid2pos = ec_boards_ctrl->get_Rid2PosMap();

    const YAML::Node doc = ec_boards_ctrl->get_config_YAML_Node();
    const YAML::Node flash_files = doc["flash_files"];

    std::vector<int> slave_list;
    bool use_rId = true;

    if ( flash_files["slave_rId_list"] ) {
        slave_list = flash_files["slave_rId_list"].as<std::vector<int>>();
    } else if ( flash_files["slave_pos_list"] ) {
        slave_list = flash_files["slave_pos_list"].as<std::vector<int>>();
        use_rId = false;
    } else {
        std::cout << "NO slave list found !!" << std::endl;
        delete ec_boards_ctrl;
        return 1;
    }

    if ( slave_list.size() == 1 ) {
        if ( slave_list[0] == -4 ) {
            // centAC motor
            use_rId = false;
            slave_list.clear();
            std::map<int, CentAcESC*> centAC_boards;
            ec_boards_ctrl->get_esc_map_byclass ( centAC_boards );
            for ( auto const& item : centAC_boards ) {
                slave_list.push_back ( item.first );
            }
            ec_boards_ctrl->get_zombie_map_bytype ( CENT_AC, centAC_boards );
            for ( auto const& item : centAC_boards ) {
                slave_list.push_back ( item.first );
            }
        }
    }

    std::string bin_path;
    std::string bin_file;
    std::string save_as;
    uint32_t passwd;
    uint32_t size_byte;
    EscWrapper * esc;
    uint32_t   bType;
    int sPos, rId = -1;

    bin_path = flash_files["bin_path"].as<std::string>();

    for ( auto it = slave_list.begin(); it != slave_list.end(); it++ ) {
        if ( use_rId ) {
            // use rId
            try {
                sPos = rid2pos.at ( *it );
            } catch ( const std::out_of_range& oor ) {
                DPRINTF ( "%s\n", oor.what() );
                //continue;
            }
        } else {
            // use slave pos
            sPos = *it;
        }
        esc = ec_boards_ctrl->slave_as_EscWrapper ( sPos );
        if ( !esc ) {
            esc = ec_boards_ctrl->slave_as_Zombie ( sPos );
            DPRINTF ( "..... try with Z0mb13 %d\n", sPos );
        } else {
            rId = esc->get_robot_id();
        }
        if ( esc ) {
            YAML::Node esc_type;
            bType = esc->get_ESC_type();
            switch ( bType ) {
/*
            case HI_PWR_AC_MC :
                esc_type = firmware_update["big_motor"];
                break;
            case HI_PWR_DC_MC :
                esc_type = firmware_update["medium_motor"];
                break;
            case LO_PWR_DC_MC :
                esc_type = firmware_update["small_motor"];
                break;
            case FT6 :
                esc_type = firmware_update["force_torque_6"];
                break;
            case POW_BOARD  :
                esc_type = firmware_update["power_hub"];
                break;
            case POW_F28M36_BOARD  :
                esc_type = firmware_update["power_f28m36"];
                break;
*/
            case CENT_AC :
                esc_type = flash_files["cent_AC"];
                break;
            case F28M36_TEST :
                esc_type = flash_files["test_f28m36"];
                break;
            default :
                break;
            }

            if ( ! esc_type.IsNull() ) {
                
                if ( esc_F28M36_uc_set.find(esc->get_ESC_type()) != esc_F28M36_uc_set.end() ) {
                    // special case F28M3x MCUs have 2 cores
                    // M3
                    for ( auto const& flash_sector : {"params","torque_cal"} ) {
                    
                        if ( esc_type["m3"][flash_sector] ) {
                            auto m3_flash_sec = esc_type["m3"][flash_sector]; 
                            bin_file    = m3_flash_sec["bin_file"].as<std::string>();
                            passwd      = m3_flash_sec["passwd"].as<int>();
                            size_byte   = m3_flash_sec["size_byte"].as<int>();
                            save_as     = bin_path+bin_file+"_pos_"+std::to_string(sPos)+"_id_"+std::to_string(rId)+".bin";
                            DPRINTF ( "%d %s 0x%04X %d\n", *it, ( bin_file+".bin" ).c_str(), passwd, size_byte );
                            if ( ! ec_boards_ctrl->upload_flash ( sPos, bin_file+".bin", passwd, size_byte, save_as) ) {
                                DPRINTF ( "FAIL read %s from M3 MCU pos %d\n" ,( bin_file+".bin" ).c_str(), sPos );
                            }
                        }
                    }
                    // C28
                    if ( esc_type["c28"] ) {
                        DPRINTF ( "No flash on C28 MCU %d\n" ,sPos );
                    }
                } else {
                    // 
                    DPRINTF ( "No flash on MCU %d\n" ,sPos );
                }
            } else {
                DPRINTF ( "Unknown esc type %d\n" ,bType );
            }

        }
    }

    delete ec_boards_ctrl;

    ///////////////////////////////////////////////////////////////////////////
    sleep(1);
    ///////////////////////////////////////////////////////////////////////////

    ec_boards_ctrl = new Ec_Boards_ctrl ( argv[1] );

    if ( ec_boards_ctrl->init() != EC_BOARD_OK ) {
        std::cout << "Error in boards init()... cannot proceed!" << std::endl;
        delete ec_boards_ctrl;
        return 0;
    }

    delete ec_boards_ctrl;

    return 1;


} catch ( std::exception& e ) {

    std::cout << "Main catch .... " <<  e.what() << std::endl;

}
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
