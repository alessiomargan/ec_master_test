#include <iit/advr/ec_boards_base.h>
#include <protobuf/repl_cmd.pb.h>

#define MAX_PB_BUFF_SIZE    1024

using namespace iit::ecat::advr;

int do_ctrl_cmd(iit::advr::Repl_cmd& repl_cmd, iit::ecat::EscWrapper * esc,
                   Ctrl_cmd_type cmd_on, Ctrl_cmd_type cmd_off,
                   std::string error_msg)
{
    int ret_val = EC_BOARD_NOK;
    
    if ( ! esc ) { return ret_val;}
    
    if ( repl_cmd.mutable_ctrl_cmd()->has_value() ) {
        if ( repl_cmd.mutable_ctrl_cmd()->value() ) {
            ret_val = esc->set_ctrl_status(cmd_on);
        } else {
            ret_val = esc->set_ctrl_status(cmd_off);
        }
    } else {
        error_msg = std::string("CTRL_FAN should have a value");
    }
    return ret_val;
}

int Ec_Thread_Boards_base::repl_loop ( void ) {
    
    int                 bytes = 0;
    uint32_t            msg_size;
        
    if ( ( bytes = replInXddp.xddp_read(msg_size) ) <= 0 ) {
        return bytes;
    }
    
    DPRINTF("[REPL] cmd size >>> %d\n", msg_size);
    if ( msg_size > MAX_PB_BUFF_SIZE ) {
        DPRINTF("[REPL] cmd size too big >>> %d\n", msg_size);
        return bytes;
    }
    
    uint8_t                 pb_buf[MAX_PB_BUFF_SIZE];    
    iit::advr::Repl_cmd     pb_msg;
    
    bytes += replInXddp.xddp_read(pb_buf, msg_size);
    pb_msg.ParseFromArray(pb_buf, msg_size);
    DPRINTF("[REPL] pb_msg\n%s\n", pb_msg.DebugString().c_str());
    
    iit::ecat::EscWrapper * esc = nullptr;
    Motor *                 moto = nullptr;
    Ft6Msp432ESC *          ft = nullptr;
    uint32_t        cmd_type = 0;
    int32_t         board_id = 0;
    uint32_t        slave_pos = 0;
    int32_t         ret_val = EC_BOARD_NOK;
    std::string     error_msg("Error !!!");
    
    // get msg content to retrive cmd_type and board_id 
    switch ( pb_msg.type() ) {
        /**********************************************************************
         *          Flash commands
         *********************************************************************/
        case iit::advr::CmdType::FLASH_CMD :
            board_id = pb_msg.mutable_flash_cmd()->board_id();
            slave_pos = rid2Pos(board_id);
            esc = slave_as_EscWrapper(slave_pos);
            
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
            if ( cmd_type && board_id && slave_pos) {
                ret_val = esc->set_flash_cmd ( cmd_type );
            } else {
                DPRINTF("[REPL] error, invalid slave_pos %d or board_id %d\n",slave_pos, board_id);
            }
            break; // case iit::advr::CmdType::FLASH_CMD
        /**********************************************************************
         * Control commands
         *********************************************************************/
        case iit::advr::CmdType::CTRL_CMD :
            board_id = pb_msg.mutable_ctrl_cmd()->board_id();
            slave_pos = rid2Pos(board_id);
            //
            esc = slave_as_EscWrapper(slave_pos);
            moto = slave_as<Motor>(slave_pos);
            ft = slave_as<Ft6Msp432ESC>(slave_pos);
            
            if ( ! esc ) {
                // set error string once for all cases
                error_msg = std::string( "Slave Id "+std::to_string(board_id)+" NOT FOUND ==> pos " + std::to_string(slave_pos));
                break;
            }
            
            switch ( pb_msg.mutable_ctrl_cmd()->type() ) {
                // 
                case iit::advr::Ctrl_cmd::CTRL_FAN :
                    ret_val = do_ctrl_cmd(pb_msg, esc, CTRL_FAN_ON, CTRL_FAN_OFF, error_msg);
                    break;
                case iit::advr::Ctrl_cmd::CTRL_LED :
                    ret_val = do_ctrl_cmd(pb_msg, esc, CTRL_LED_ON, CTRL_LED_OFF, error_msg);
                    break;
                case iit::advr::Ctrl_cmd::CTRL_POWER_MOD :
                    ret_val = do_ctrl_cmd(pb_msg, esc, CTRL_POWER_MOD_ON, CTRL_POWER_MOD_OFF, error_msg);
                    break;
                case iit::advr::Ctrl_cmd::CTRL_SANDBOX :
                    ret_val = do_ctrl_cmd(pb_msg, esc, CTRL_SAND_BOX_ON, CTRL_SAND_BOX_OFF, error_msg);
                    break;
                case iit::advr::Ctrl_cmd::CTRL_REF_FILTER :
                    ret_val = do_ctrl_cmd(pb_msg, esc, CTRL_POS_REF_FILTER_ON, CTRL_POS_REF_FILTER_OFF, error_msg);
                    break;
                    
                case iit::advr::Ctrl_cmd::CTRL_SET_ZERO_POSITION :
                    if ( moto ) {
                        if ( pb_msg.mutable_ctrl_cmd()->has_value() ) {
                            ret_val = moto->set_zero_position(pb_msg.mutable_ctrl_cmd()->value());
                        } else {
                            error_msg = std::string("CTRL_SET_ZERO_POSITION should have a value");
                        }
                    }
                    break;
                case iit::advr::Ctrl_cmd::CTRL_CMD_START :
                    if ( moto ) {
                        if ( pb_msg.mutable_ctrl_cmd()->has_value() ) {
                            // start motor with controller's gains set in yaml file
                            ret_val = moto->start(pb_msg.mutable_ctrl_cmd()->value());
                        } else {
                            // start motor with controller type and gains set in yaml file
                            ret_val = moto->start();
                        }
                    }
                    break;
                case iit::advr::Ctrl_cmd::CTRL_CMD_STOP :
                    if ( moto ) { ret_val = moto->stop(); }
                    // empty trj queue
                    trj_queue.clear();
                    break;
                
                case iit::advr::Ctrl_cmd::CTRL_DAC_TUNE :
                    if ( ft ) { ret_val = ft->run_dac_tune(); }
                    break;
                
                case iit::advr::Ctrl_cmd::CTRL_TEST_DONE :
                    ret_val = esc->set_ctrl_status( CTRL_TEST_DONE );
                    break;
 
                case iit::advr::Ctrl_cmd::CTRL_TEST_ERROR :
                    ret_val = esc->set_ctrl_status( CTRL_TEST_ERROR );
                    break;
                
                default:
                    break;
 
            }
            break; // case iit::advr::CmdType::CTRL_CMD
        
        /**********************************************************************
         * Trajectory commands
         *********************************************************************/
        case iit::advr::CmdType::SEND_TRAJECTORY :
            auto trj_cmd = pb_msg.mutable_trajectory_cmd();
            board_id = pb_msg.mutable_trajectory_cmd()->board_id();
            slave_pos = rid2Pos(board_id);
            Motor * moto = slave_as<Motor>(slave_pos);
            std::string trj_name = trj_cmd->name();
                    
            if ( ! moto ) {
                // set error string once for all cases
                error_msg = std::string( "Slave Id "+std::to_string(board_id)+" NOT FOUND ==> pos " + std::to_string(slave_pos));
            }
            
            switch ( trj_cmd->type() ) {
                
                case iit::advr::Trajectory_cmd::HOME :
                    float motor_pos;
                    if ( moto && (iit::ecat::EC_WRP_OK == moto->readSDO ( "motor_pos", motor_pos ))) {
                        start_pos[slave_pos] = motor_pos;
                        std::vector<double> Xs(trj_cmd->x().begin(), trj_cmd->x().end());
                        std::vector<double> Ys(trj_cmd->y().begin(), trj_cmd->y().end());
                        Ys[0] = start_pos[slave_pos];
                        trj_names[trj_name][slave_pos] = std::make_shared<advr::Smoother_trajectory>( Xs, Ys );
                        trj_queue.clear();
                        trj_queue.push_back(trj_names[trj_name]);
                        advr::reset_trj ( trj_queue.at(0) );
                        ret_val = EC_BOARD_OK;
                    }
                    //
                    break;
                
                case iit::advr::Trajectory_cmd::SINE :
                    std::vector<double> Xs(trj_cmd->x().begin(), trj_cmd->x().end());
                    std::vector<double> Ys(trj_cmd->y().begin(), trj_cmd->y().end());
                    //
                    trj_names[trj_name][slave_pos] = std::make_shared<advr::Smoother_trajectory>( Xs, Ys );
                    trj_queue.clear();
                    trj_queue.push_back(trj_names[name]); 
                    break;
                
                //default:
                //    break;
            }
            break; // iit::advr::CmdType::TRJ_CMD
        /**********************************************************************
         * Trajectory queue commands
         *********************************************************************/
        case iit::advr::CmdType::TRJ_QUEUE_CMD :
            trj_queue_cmd = pb_repl_msg.mutable_trj_queue_cmd();
            for ( const auto &nm : trj_queue_cmd->trj_names() ) {
                trj_queue_names.push_back(nm);
            }
            
            switch ( trj_queue_cmd->type() ) {
                //
                // TODO check initial and final trj ...
                //
                case iit::advr::Trj_queue_cmd::PUSH_QUE :
                    //trj_queue.clear();
                    for ( auto const& name : trj_queue_names ) {
                        try {
                            trj_queue.push_back(trj_names.at(name));
                        } catch (std::out_of_range ) {
                            error_msg = std::string("Error trajectory ")+name+std::string(" is not defined !");
                            // exit switch with error_msg
                            break;
                        }
                    }
                    if ( ! trj_queue.empty() ) {
                        std::map<int, double> motors_pos, next_start_points;
                        // get all actual motors position 
                        for ( auto const &[slave_pos, moto] : motors ) {
                            auto motor_pdo_rx = moto->getRxPDO();
                            motors_pos[slave_pos] = motor_pdo_rx.motor_pos;
                        }
                        // check discontinuity in trjs
                        advr::reset_trj ( trj_queue.front() );
                        start_points_trj(trj_queue.front(), next_start_points);
                        if ( motors_pos != next_start_points ) {
                            // correct discontinuity, set start_point with actual motor_pos
                            for (auto const &[slave_pos,start_pos] : next_start_points) {
                                if ( motors_pos.at(slave_pos) != start_pos ) {
                                    trj_queue.front()[slave_pos]->set_start_point(motors_pos[slave_pos]);
                                }
                            }
                        }
                    } 
                    ret_val = EC_BOARD_OK;
                    break;
                case iit::advr::Trj_queue_cmd::EMPTY_QUE :
                    trj_queue.clear();
                    ret_val = EC_BOARD_OK;
                    break;
            }
            break; // iit::advr::CmdType::TRJ_QUEUE_CMD
        /**********************************************************************
         * 
         *********************************************************************/
        case iit::advr::CmdType::SLAVE_SDO_CMD :
            board_id = pb_repl_msg.mutable_slave_sdo_cmd()->board_id();
            slave_pos = rid2Pos(board_id);
            //
            esc = slave_as_EscWrapper(slave_pos);
            varEscPtr_t varEscPtr;
            if ( slave_as<CentAcESC>(slave_pos) )       { varEscPtr = slave_as<CentAcESC>(slave_pos); }
            if ( slave_as<Ft6Msp432ESC>(slave_pos) )    { varEscPtr = slave_as<Ft6Msp432ESC>(slave_pos); }
            if ( slave_as<TestESC>(slave_pos) )         { varEscPtr = slave_as<TestESC>(slave_pos); }

            if ( (slave_pos <= 0) && (!esc) ) {
                // set error string once for all cases
                error_msg = std::string( "Slave Id "+std::to_string(board_id)+" NOT FOUND ==> pos " + std::to_string(slave_pos));
                // break case iit::advr::CmdType::SLAVE_SDO_CMD
                break;
            }

            {
                // read SDO from esc
                float tmpf;
                auto it = pb_repl_msg.mutable_slave_sdo_cmd()->rd_sdo().begin();
                while ( it != pb_repl_msg.mutable_slave_sdo_cmd()->rd_sdo().end() ) {
                    DPRINTF(">> rd_sdo >> %s\n", it->c_str());
                    try {
                        std::visit(overloaded{
                                [&](CentAcESC* s)    { s->readSDO_byname(it->c_str()); },
                                [&](Ft6Msp432ESC* s) { s->readSDO_byname(it->c_str()); },
                                [&](TestESC* s)      { s->readSDO_byname(it->c_str()); },
                            }, varEscPtr );    
                    }
                    catch (const std::exception& e) { DPRINTF(" && %s\n", e.what()); }
                    
                    it++;
                }
                ret_val = EC_BOARD_OK;
            }
            {
                // write SDO to esc
                auto it = pb_repl_msg.mutable_slave_sdo_cmd()->wr_sdo().begin();
                while ( it != pb_repl_msg.mutable_slave_sdo_cmd()->wr_sdo().end() ) {
                    DPRINTF(">> wr_sdo >> %s %f\n", it->name().c_str(), it->value());
                    try {
                        try { std::get<CentAcESC*>(varEscPtr)->writeSDO_byname(it->name().c_str()); }
                        catch (const std::bad_variant_access& e) { DPRINTF(" ** %s\n", e.what()); }
                        
                        try { std::get<Ft6Msp432ESC*>(varEscPtr)->writeSDO_byname(it->name().c_str()); }
                        catch (const std::bad_variant_access& e) { DPRINTF(" ** %s\n", e.what()); }
                    }
                    catch (const std::exception& e) { DPRINTF(" && %s\n", e.what());  }
                    
                    it++;
                }
                ret_val = EC_BOARD_OK;
            }

            break; // iit::advr::CmdType::SLAVE_SDO_CMD
        
    }

    /**************************************************************************
     *
     *          Reply
     * 
     *************************************************************************/
    std::string             reply_str;
    uint32_t                reply_size;
    iit::advr::Cmd_reply    pb_reply;

    pb_reply.set_cmd_type(pb_msg.type());
                
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

