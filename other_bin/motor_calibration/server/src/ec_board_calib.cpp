#include <ec_board_calib.h>
#include <thread>
#include <chrono>
#include <cmath>

#include "giel.h"
#include "simplemsg.h"
#include "calibrationcontroller.h"
#include <iit/advr/coman_robot_id.h>
#include <iit/advr/walkman_robot_id.h>
#include <iit/advr/centauro_robot_id.h>

#define MID_POS(m,M)    (m+(M-m)/2)

using namespace iit::ecat::advr;
using namespace SimpleMsg;

std::string generateReadableTimestamp()
{
    // Build a name with the timestampo in readable format
    time_t rawtime;
    struct tm  timeinfo;
    char buffer [80];

    time (&rawtime);
    localtime_r (&rawtime, &timeinfo);
    strftime (buffer, 80, "%Y%m%d-%H%M%S", &timeinfo);
    return std::string(buffer);
}

/**
 * @brief Ec_Board_Calib::Ec_Board_Calib
 * @param config_yaml
 */
Ec_Board_Calib::Ec_Board_Calib ( const char* config_yaml ) :
    Ec_Thread_Boards_base ( config_yaml ),

    mbf {1024*1024*32, "calib_"+generateReadableTimestamp()+".log"}
{
    std::string atiIp ;
    try
    {
        const YAML::Node& board_ctrl = root_cfg["ati_config"];
        atiIp = board_ctrl["ati_ip_addr"].as<std::string>();
    } catch(...)
    {
        atiIp = ATI_IFACE_IP;
    }

    ati.config(false, 1000000., atiIp);


    name = "EC_boards_calib";
    // non periodic
    period.period = {0,1};

#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_RR;
#endif
    priority = sched_get_priority_max ( schedpolicy ) - 10 ;
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    // open pipe ... xeno xddp or fifo
    inXddp.init ( "EC_board_input" );
    pipeControlOut.init ("controlOut");



}

/**
 * @brief Ec_Board_Calib::~Ec_Board_Calib
 */
Ec_Board_Calib::~Ec_Board_Calib() {
    std::cout << "~" << typeid ( this ).name() << std::endl;
    mbf.commit();
}

/**
 * @brief Ec_Board_Calib::init_preOP
 */
void Ec_Board_Calib::sendBrdId()
{
    Msg buf;
    buf.bid.id = ID::ID_BRDID;
    buf.bid.boardId = boardId;
    pipeControlOut.xddp_write( (uint8_t*) &buf, sizeof(buf));
}

/**
 * @brief Ec_Board_Calib::updateParams
 * @param mc
 * @param mt
 * @param countsPerUnit_
 */
void Ec_Board_Calib::updateParams(float mc,
                                  float mt,
                                  float countsPerUnit_)
{
    if  ( ! ( ( mc < 0.0 ) || ( mc > 4.01) ) )
    {
        this->maxCurr = mc;
    }
    if  ( ! ( ( mt < 0.0 ) || ( mt > 80.01) ) )
    {
        this->maxTorque = mt;
    }
    this->countsPerUnit = countsPerUnit_;
    this->ati.setCountsPerUnit(countsPerUnit);

    DPRINTF( " ## %f %f %f\n", this->maxCurr, this->maxTorque, this->countsPerUnit);
}

/**
 * @brief Ec_Board_Calib::init_preOP
 */
void Ec_Board_Calib::init_preOP ( void )
{
    DPRINTF(">> start ATI THREAD\n");
    ati.start_thread();
    int slave_pos;
    for (auto const &item : motors)
    {
        //
        slave_pos = item.first;
        actualMotor = item.second;
    }

    assert (motors.size()==1);

    // READ INTERESTING SDOS
    uint16_t mgr {};
    actualMotor->readSDO("Motor_gear_ratio", mgr);
    float maxCurrent {};
    actualMotor->readSDO("Max_cur", maxCurrent);
    float motorTorqueConstant {};
    actualMotor->readSDO("motorTorqueConstant", motorTorqueConstant);
    float Torsion_bar_stiff {};
    actualMotor->readSDO("Torsion_bar_stiff", Torsion_bar_stiff);
    //gearedMotorInertia
    float gearedMotorInertia {};
    actualMotor->readSDO("gearedMotorInertia", gearedMotorInertia);
    float torqueFixedOffset {};
    actualMotor->readSDO("torqueFixedOffset", torqueFixedOffset);
    // For Current
    float min_pos, max_pos, link_pos, motor_pos;
    actualMotor->readSDO ( "Min_pos", min_pos );
    actualMotor->readSDO ( "Max_pos", max_pos );
    actualMotor->readSDO ( "motor_pos", motor_pos );
    actualMotor->readSDO ( "link_pos", link_pos );
    start_pos[slave_pos] = motor_pos;


    DPRINTF(" Reduction: %d \n Maximum Current %f\n Torque Constant %f \n"
            "Torsion_bar_stiff: %f\n gearedMotorInertia: %f\n"
            "torqueFixedOffset: %f\n \n",
            mgr, maxCurrent, motorTorqueConstant,
            Torsion_bar_stiff, gearedMotorInertia, torqueFixedOffset);

    {
        char logBuf[128];
        snprintf(logBuf, sizeof(logBuf), "# Params\n# Joint_id %d\n# motor %f link %f\n# start %f home %f", pos2Rid ( slave_pos ), motor_pos, link_pos, start_pos[slave_pos], home[slave_pos]);
        mbf.addRow(logBuf);
        snprintf(logBuf, sizeof(logBuf), "# Reduction: %d \n# Maximum Current %f\n# Torque Constant %f ",mgr, maxCurrent, motorTorqueConstant);
        mbf.addRow(logBuf);
        snprintf(logBuf, sizeof(logBuf), "# Torsion_bar_stiff: %f\n# gearedMotorInertia: %f\n# torqueFixedOffset: %f",Torsion_bar_stiff, gearedMotorInertia, torqueFixedOffset);
        mbf.addRow(logBuf);
    }

    DPRINTF ( ">> Joint_id %d motor %f link %f start %f home %f\n", pos2Rid ( slave_pos ), motor_pos, link_pos, start_pos[slave_pos], home[slave_pos]);

    this->boardId = pos2Rid ( slave_pos );

    sendBrdId();
    CalResult res;
    res.M = Torsion_bar_stiff;
    res.B = 0.0f;
    res.R = 0.0f;
    sendResult(res);

    // set home to mid pos
    home[slave_pos] = MID_POS ( min_pos,max_pos );

    char logBuf[80];
    snprintf(logBuf, 80, "# timestamp | ft[5] | torque | linkPos | motorpos | currentsample");
    mbf.addRow(logBuf);
}

/**
 * @brief Ec_Board_Calib::init_OP
 */
void Ec_Board_Calib::init_OP ( void )
{

}

/**
 *
 */
int Ec_Board_Calib::user_input (uint8_t &user_cmd )
{

    Msg buffControl;
    auto bytes = inXddp.xddp_read((uint8_t *)&buffControl, sizeof(buffControl));
    if ( bytes > 0)
    {
        switch (buffControl.id)
        {
        case ID::ID_CMD:
        {
            user_cmd = buffControl.cmd.command;
            DPRINTF(" [user_input] %d %d \n", bytes, user_cmd);
        }
            break;
        case ID::ID_CFG:
        {
            updateParams( buffControl.cfg.maxCurr,
                          buffControl.cfg.maxTorque,
                          buffControl.cfg.countsPerUnit);
            Msg buf;
            buf.cfg.id = ID::ID_CFG;
            buf.cfg.maxCurr = this->maxCurr;
            buf.cfg.maxTorque = this->maxTorque;
            buf.cfg.countsPerUnit = this->countsPerUnit;
            pipeControlOut.xddp_write( (uint8_t*) &buf, sizeof(buf));
            user_cmd = Commands::CMD_NOCMD;
        }
            break;
        default:
        {
            user_cmd = Commands::CMD_NOCMD;
        }
            break;
        }
    }

    return bytes;
}

/**/
static auto start =  std::chrono::high_resolution_clock::now();
/**/

/**
 * @brief Ec_Board_Calib::user_loop
 * @return
 */
void Ec_Board_Calib::sendAck(uint8_t command)
{
    Msg buf;
    buf.ack.id = ID::ID_ACK;
    buf.ack.command = command;
    pipeControlOut.xddp_write( (uint8_t*) &buf, sizeof(buf));
}

/**
 * @brief Ec_Board_Calib::handleCommand
 * @param command
 */
void Ec_Board_Calib::handleCommand(uint8_t command)
{
    switch ( command )
    {
    case Commands::CMD_GETID :
    {
        sendBrdId();
    }
        break;

    case Commands::CMD_ZEROATI :
    {
        DPRINTF(" ZeroOffset \n");
        ati.zeroOffset();
    }
        break;

    case Commands::CMD_STARTCALIB :
    {
        DPRINTF(" startCalibration \n");
        calcc.startCalibration(this->maxCurr);

        char logBuf[80];
        snprintf(logBuf, 80, "# start Calibration");
        mbf.addRow(logBuf);

        sendAck(command);
    }
        break;

    case Commands::CMD_SAVERESULT :
    {
        DPRINTF(" saveResult \n");
        uint16_t fcmd=0x12;
        actualMotor->writeSDO("flash_params_cmd",fcmd);
        MILLISLEEP(10);
        actualMotor->readSDO("flash_params_cmd_ack",fcmd);
        DPRINTF(" flash_params_cmd_ack: 0x%04x ", fcmd);
        sendAck(command);
    }
        break;

    case Commands::CMD_RESETSTIFFNESS:
    {
        actualMotor->writeSDO("Torsion_bar_stiff", 1.0f);

        CalResult res;
        res.M = 1.0f;
        res.B = 0.0f;
        res.R = 0.0f;
        sendResult(res);
    }
        break;

    case Commands::CMD_STARTMOTORS:
    {
        auto res = actualMotor->start ( CTRL_SET_CURR_MODE );
        assert ( res == EC_BOARD_OK );

        sendAck(command);
    }
        break;

    case Commands::CMD_STOPMOTORS:
    {
        stop_motors();
        sendAck(command);
    }
        break;

    case Commands::CMD_GETRESULT:
    {
        float Torsion_bar_stiff {};
        actualMotor->readSDO("Torsion_bar_stiff", Torsion_bar_stiff);

        CalResult res;
        res.M = Torsion_bar_stiff;
        res.B = 0.0f;
        res.R = 0.0f;
        sendResult(res);
    }
        break;

    }
}

void Ec_Board_Calib::sendResult(const CalResult& result)
{
    Msg buf;
    buf.id = ID_REGRESULT;
    buf.res.M= result.M;
    buf.res.B= result.B;
    buf.res.R= result.R;
    pipeControlOut.xddp_write( (uint8_t*) &buf, sizeof(buf));
}

int Ec_Board_Calib::user_loop ( void )
{

    uint8_t command {ID::ID_NONE};
    user_input ( command );
    handleCommand(command);

    auto end =  std::chrono::high_resolution_clock::now();
    auto ticks = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();

    // Collect samples
    ati_log_t atiSample {};
    ati.get_last_sample(atiSample);
    auto pdorx = actualMotor->getRxPDO();

    auto sample = calcc.sample(ticks/1000000.0, atiSample.ft[5], pdorx.torque);

    //
    // Check Maximum Torque on ATI
    //
    if ( fabs(atiSample.ft[5]) > this->maxTorque )
    {
        actualMotor->set_torRef(0.0);
        stop_motors();
        DPRINTF("\n***TORQUE OVERLOAD %f %f \n", fabs(atiSample.ft[5]), this->maxTorque );

        char logBuf[80];
        snprintf(logBuf, 80, "# TORQUE OVERLOAD");
        mbf.addRow(logBuf);


        return -1;
    }

    actualMotor->set_torRef(sample);

    if (calcc.calibrationOnGoing())
    {
        if (calcc.calibrationEnded())
        {
            auto result = calcc.getResult();
            DPRINTF("\n** M: %f B: %f R: %f\n", result.M, result.B, result.R );
            calcc.reset();

            char logBuf[80];
            snprintf(logBuf, 80, "# M: %f B: %f R: %f", result.M, result.B, result.R );
            mbf.addRow(logBuf);

            sendResult(result);

            actualMotor->writeSDO("Torsion_bar_stiff", result.M);
        }
    }


    char logBuf[80];
    snprintf(logBuf, 80, "%li %f %f %f %f %f",ticks, atiSample.ft[5], pdorx.torque, pdorx.link_pos, pdorx.motor_pos, sample);
    mbf.addRow(logBuf);

    Msg buf;
    buf.id = ID::ID_SAMPLE;
    buf.curr.sample = sample;
    pipeControlOut.xddp_write( (uint8_t*) &buf, sizeof(buf));

    return 0;

}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on
