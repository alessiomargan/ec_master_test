#include <iit/advr/ec_boards_base.h>

#define MAX_PB_BUFF_SIZE    1024

using namespace iit::ecat::advr;

int Ec_Thread_Boards_base::repl_loop ( void ) {
    
    int                 bytes = 0;
    uint32_t            msg_size;
    uint8_t             pb_buf[MAX_PB_BUFF_SIZE];    
    iit::advr::Repl_cmd pb_msg;
    
    CentAcESC *     moto = 0;
    Ft6Msp432ESC *  ft = 0;
    
    if ( ( bytes = replInXddp.xddp_read(msg_size) ) <= 0 ) {
        return bytes;
    }
    
    DPRINTF("[REPL] cmd size >>> %d\n", msg_size);
    if ( msg_size > MAX_PB_BUFF_SIZE ) {
        DPRINTF("[REPL] cmd size too big >>> %d\n", msg_size);
        return bytes;
    }
    
    bytes += replInXddp.xddp_read(pb_buf, msg_size);
    pb_msg.ParseFromArray(pb_buf, msg_size);
    DPRINTF("[REPL] pb_msg\n%s\n", pb_msg.DebugString().c_str());
    
    uint32_t cmd_type = 0;
    int32_t  board_id = 0;
    uint32_t slave_pos = 0;
    int ret_val = EC_BOARD_NOK;
    std::string error_msg("Error !!!");
    
    // get msg content to retrive cmd_type and board_id 
    switch ( pb_msg.type() ) {
        /**********************************************************************
         *          Flash commands
         *********************************************************************/
        case iit::advr::Repl_cmd::SET_FLASH_CMD :
            board_id = pb_msg.mutable_flash_cmd()->board_id();
            slave_pos = rid2Pos(board_id);
            switch ( pb_msg.mutable_flash_cmd()->type() ) {
                case iit::advr::Flash_cmd::SAVE_PARAMS_TO_FLASH :
                    cmd_type = SAVE_PARAMS_TO_FLASH;
                    break;
                case iit::advr::Flash_cmd::LOAD_PARAMS_FROM_FLASH :
                    cmd_type = LOAD_PARAMS_FROM_FLASH;
                    break;
                case iit::advr::Flash_cmd::LOAD_DEFAULT_PARAMS :
                    cmd_type = LOAD_DEFAULT_PARAMS;
                    break;
                default :
                    break;
            }
            if ( cmd_type && board_id ) {
                // ok if at last one of set_flash_cmd_X return EC_BOARD_OK --> 0
                ret_val  = set_flash_cmd_X ( slave_as<Ft6Msp432ESC>(slave_pos), cmd_type );
                ret_val &= set_flash_cmd_X ( slave_as<CentAcESC>(slave_pos), cmd_type );
                ret_val &= set_flash_cmd_X ( slave_as<TestESC>(slave_pos), cmd_type );
            } else {
                DPRINTF("[REPL] error, invalid slave_pos %d or board_id %d\n",slave_pos, board_id);
            }
            break;
        /**********************************************************************
         * Control commands
         *********************************************************************/
        case iit::advr::Repl_cmd::SET_CTRL_CMD :
            board_id = pb_msg.mutable_ctrl_cmd()->board_id();
            slave_pos = rid2Pos(board_id);
            moto = slave_as<CentAcESC>(slave_pos);
            ft = slave_as<Ft6Msp432ESC>(slave_pos);
            
            if ( ! (moto || ft) ) {
                // set error string once for all cases
                error_msg = std::string( "Invalid slave_pos " + std::to_string(slave_pos));
            }
            
            switch ( pb_msg.mutable_ctrl_cmd()->type() ) {
                
                case iit::advr::Ctrl_cmd::CTRL_SET_ZERO_POSITION :
                    if ( moto ) {
                        if ( pb_msg.mutable_ctrl_cmd()->has_value() ) {
                            ret_val = moto->set_zero_position(pb_msg.mutable_ctrl_cmd()->value());
                        }
                    }
                    break;
                
                case iit::advr::Ctrl_cmd::CTRL_DAC_TUNE :
                    if ( ft ) { ret_val = ft->run_dac_tune(); }
                    break;
                
                case iit::advr::Ctrl_cmd::CTRL_TEST_DONE :
                    if ( ft ) { ret_val = set_ctrl_status_X ( ft, CTRL_TEST_DONE ); }
                    break;
 
                case iit::advr::Ctrl_cmd::CTRL_TEST_ERROR :
                    if ( ft ) { ret_val = set_ctrl_status_X ( ft, CTRL_TEST_ERROR ); }
                    break;
                
                default:
                    break;
 
            }
            break;
    }

    /**************************************************************************
     *
     *          Reply
     * 
     *************************************************************************/
    iit::advr::Cmd_reply    pb_reply;
    std::string             reply_str;
    uint32_t                reply_size;
    
    if ( ret_val != 0 ) {
        //fail
        pb_reply.set_type(iit::advr::Cmd_reply::NACK);
        pb_reply.set_msg(error_msg);
    } else {
        pb_reply.set_type(iit::advr::Cmd_reply::ACK);
        pb_reply.set_msg("Ok google");
    }
    DPRINTF("[REPL] pb_reply\n%s\n", pb_reply.DebugString().c_str());
    pb_reply.SerializeToString(&reply_str);
    reply_size = reply_str.length();
    bytes  = replOutXddp.xddp_write ( ( void* )&reply_size, sizeof( reply_size ) );
    bytes += replOutXddp.xddp_write ( ( void* )reply_str.c_str(), reply_str.length() );    
    
    return bytes;
}
