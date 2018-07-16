/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_BOARDS_BASIC_H__
#define __EC_BOARDS_BASIC_H__

#include <iit/advr/ec_boards_base.h>
#include "ati_iface.h"
#include "giel.h"
#include "calibrationcontroller.h"
/**
 */

class Ec_Board_Calib : public Ec_Thread_Boards_base {
public:

    Ec_Board_Calib ( const char * config_yaml );
    virtual ~Ec_Board_Calib();

    int user_input ( uint8_t &user_cmd );
    virtual int user_loop ( void );

    void sendBrdId();

private :

    Ati_Sens ati;
    iit::ecat::advr::Motor* actualMotor;
    int boardId{-1};
    float maxCurr{2.0};
    float maxTorque{20.0};
    float countsPerUnit{ 1000000.0f};
    CalibrationController calcc{3.00};

    void updateParams(float, float, float);

    virtual void init_preOP ( void );
    virtual void init_OP ( void );

    XDDP_pipe inXddp;
    XDDP_pipe pipeControlOut;
    int  inputBytesCount;
    void handleCommand(uint8_t command);
    void sendAck(uint8_t command);

    MemBufferLogger mbf;
    void sendResult(const CalResult& result);
};




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
