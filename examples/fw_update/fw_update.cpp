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

extern void main_common ( __sighandler_t sig_handler );

static int main_loop = 1;

void shutdown ( int sig __attribute__ ( ( unused ) ) ) {
    main_loop = 0;
    printf ( "got signal .... Shutdown\n" );
}

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main ( int argc, char *argv[] ) try {

    if ( argc != 2 ) {
        printf ( "Usage: %s config.yaml\n", argv[0] );
        return 0;
    }

    main_common ( shutdown );

    ///////////////////////////////////////////////////////////////////////////

    Ec_Boards_ctrl * ec_boards_ctrl = new Ec_Boards_ctrl ( argv[1] );

    if ( ec_boards_ctrl->init() != EC_BOARD_OK ) {
        std::cout << "Error in boards init()... cannot proceed!" << std::endl;
        delete ec_boards_ctrl;
        return 0;
    }

    Rid2PosMap  rid2pos = ec_boards_ctrl->get_Rid2PosMap();

    const YAML::Node doc = ec_boards_ctrl->get_config_YAML_Node();
    const YAML::Node firmware_update = doc["firmware_update"];

    std::vector<int> slave_list;
    bool use_rId = true;

    if ( firmware_update["slave_rId_list"] ) {
        slave_list = firmware_update["slave_rId_list"].as<std::vector<int>>();
    } else if ( firmware_update["slave_pos_list"] ) {
        slave_list = firmware_update["slave_pos_list"].as<std::vector<int>>();
        use_rId = false;
    } else {
        std::cout << "NO slave list found !!" << std::endl;
        delete ec_boards_ctrl;
        return 1;
    }

    if ( slave_list.size() == 1 ) {
        if ( slave_list[0] == 0 ) {
            // all esc
            use_rId = false;
            slave_list.resize ( ec_slavecount );
            std::cout << "slave list size " << slave_list.size() << std::endl;
            // exclude power board
            // slave_list : [2 .. ec_slavecount)
            std::iota ( slave_list.begin(), slave_list.end(), 2 );

        } else if ( slave_list[0] == -1 ) {
            // big motor
            use_rId = false;
            slave_list.clear();
            std::map<int, HpESC*> hp_boards;
            ec_boards_ctrl->get_esc_map_bytype ( HI_PWR_AC_MC, hp_boards );
            for ( auto it = hp_boards.begin(); it != hp_boards.end(); it++ ) {
                slave_list.push_back ( it->first );
            }
            ec_boards_ctrl->get_zombie_map_bytype ( HI_PWR_AC_MC, hp_boards );
            for ( auto it = hp_boards.begin(); it != hp_boards.end(); it++ ) {
                slave_list.push_back ( it->first );
            }
        } else if ( slave_list[0] == -2 ) {
            // med motor
            use_rId = false;
            slave_list.clear();
            std::map<int, HpESC*> hp_boards;
            ec_boards_ctrl->get_esc_map_bytype ( HI_PWR_DC_MC, hp_boards );
            for ( auto it = hp_boards.begin(); it != hp_boards.end(); it++ ) {
                slave_list.push_back ( it->first );
            }
            ec_boards_ctrl->get_zombie_map_bytype ( HI_PWR_DC_MC, hp_boards );
            for ( auto it = hp_boards.begin(); it != hp_boards.end(); it++ ) {
                slave_list.push_back ( it->first );
            }
        } else if ( slave_list[0] == -3 ) {
            // lopwr motor
            use_rId = false;
            slave_list.clear();
            std::map<int, LpESC*> lp_boards;
            ec_boards_ctrl->get_esc_map_bytype ( LO_PWR_DC_MC, lp_boards );
            for ( auto it = lp_boards.begin(); it != lp_boards.end(); it++ ) {
                slave_list.push_back ( it->first );
            }
            ec_boards_ctrl->get_zombie_map_bytype ( LO_PWR_DC_MC, lp_boards );
            for ( auto it = lp_boards.begin(); it != lp_boards.end(); it++ ) {
                slave_list.push_back ( it->first );
            }
        } else if ( slave_list[0] == -4 ) {
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

    std::string fw_path;
    std::string bin_file;
    int passwd;
    EscWrapper * esc;
    uint16_t   bType;
    int sPos;

    fw_path = firmware_update["fw_path"].as<std::string>();

    for ( auto it = slave_list.begin(); it != slave_list.end(); it++ ) {
        if ( use_rId ) {
            // use rId
            try {
                sPos = rid2pos.at ( *it );
            } catch ( const std::out_of_range& oor ) {
                DPRINTF ( "%s\n", oor.what() );
                continue;
            }
        } else {
            // use slave pos
            sPos = *it;
        }
        esc = ec_boards_ctrl->slave_as_EscWrapper ( sPos );
        if ( !esc ) {
            esc = ec_boards_ctrl->slave_as_Zombie ( sPos );
            DPRINTF ( "..... try with Z0mb13 %d\n", sPos );
        }
        if ( esc ) {
            YAML::Node motor_type;
            bType = esc->get_ESC_type();
            switch ( bType ) {
            case HI_PWR_AC_MC :
                motor_type = firmware_update["big_motor"];
                break;
            case HI_PWR_DC_MC :
                motor_type = firmware_update["medium_motor"];
                break;
            case LO_PWR_DC_MC :
                motor_type = firmware_update["small_motor"];
                break;
            case FT6 :
                motor_type = firmware_update["force_torque_6"];
                break;
            case POW_BOARD  :
                motor_type = firmware_update["power_hub"];
                break;
            case CENT_AC :
                motor_type = firmware_update["cent_AC"];
                break;
            default :
                break;
            }

            if ( ! motor_type.IsNull() ) {
                
                // special case F28M3x MCUs have 2 cores
                if ( esc->get_ESC_type() == CENT_AC ) {
                    // M3
                    if ( motor_type["m3"] ) {
                        bin_file    = motor_type["m3"]["bin_file"].as<std::string>();
                        passwd      = motor_type["m3"]["passwd"].as<int>();
                        DPRINTF ( "%d %s 0x%04X \n", *it, ( fw_path+bin_file ).c_str(), passwd );
                        if ( ! ec_boards_ctrl->update_board_firmware ( sPos, fw_path+bin_file, passwd, "m3") ) {
                            DPRINTF ( "FAIL update slave pos M3 MCU %d\n" ,sPos );
                        }
                    }
                    // C28
                    if ( motor_type["c28"] ) {
                        bin_file    = motor_type["c28"]["bin_file"].as<std::string>();
                        passwd      = motor_type["c28"]["passwd"].as<int>();
                        if ( ! ec_boards_ctrl->update_board_firmware ( sPos, fw_path+bin_file, passwd, "c28" ) ) {
                            DPRINTF ( "FAIL update slave pos C28 MCU %d\n" ,sPos );
                        }
                    }
                } else {
                    bin_file    = motor_type["bin_file"].as<std::string>();
                    passwd      = motor_type["passwd"].as<int>();
                    DPRINTF ( "%d %s 0x%04X \n", *it, ( fw_path+bin_file ).c_str(), passwd );
                    if ( ! ec_boards_ctrl->update_board_firmware ( sPos, fw_path+bin_file, passwd, "none" ) ) {
                        DPRINTF ( "FAIL update slave pos %d\n" ,sPos );
                    }
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
