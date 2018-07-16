#include "calibgui.h"

#include <QtCharts/QXYSeries>
#include <QDebug>
#include <arpa/inet.h>
#include <errno.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>

#include <algorithm>

#include "simplemsg.h"


QT_CHARTS_USE_NAMESPACE

Q_DECLARE_METATYPE(QAbstractSeries *)
Q_DECLARE_METATYPE(QAbstractAxis *)

using namespace SimpleMsg;

void dumpBuf(uint8_t* buf, size_t size)
{
  for (uint32_t idx = 0; idx < size; idx++)
  {
    if (idx % 8 == 0)
      printf(" [ %03d - %03d ] ", idx, idx+7);
    printf(" 0x%02x ", buf[idx] );
    if (idx % 8 == 7)
      printf("\n");
  }
}
/**
 * @brief CalibGui::gatherDataThread
 * @return
 */
int CalibGui::requestInitialData()
{
    int bid = -1;

    do {
        Msg buffControl;
        fd_set readset;

        // Initialize the set
        FD_ZERO(&readset);
        FD_SET(pipeControl.get_fd(), &readset);
        struct timeval tv;
        // Initialize time out struct
        tv.tv_sec = 0;
        tv.tv_usec = 1000;

        auto result = select(pipeControl.get_fd() + 1, &readset, NULL, NULL, &tv);

        auto bytes = pipeControl.xddp_read((uint8_t *)&buffControl, sizeof(buffControl));
        if ( bytes > 0 )
        {
            if ( buffControl.id == ID::ID_BRDID )
            {
                bid = buffControl.bid.boardId;
            }
            else
            {
                static int count = 0;
                if ( (count++ % 10000) == 0)
                {
                    Msg buf2;
                    buf2.cmd.id = ID::ID_CMD;
                    buf2.cmd.command = Commands::CMD_GETID;
                    pipeInput.xddp_write( (uint8_t*) &buf2, sizeof(buf2));
                }
            }

        }
        std::this_thread::yield();
    }
    while (bid == -1);

    this->setBid(bid);

    requestStiff();

    return bid;
}

int CalibGui::gatherDataThread()
{

    pipeControl.init("controlOut");

    requestInitialData();

    pipeMotor.init("Motor_id_"+std::to_string(bid));
    pipeAti.init("atiSens");

    emit enableGui();


    auto starttime = iit::ecat::get_time_ns();
    auto originalTime = starttime;
    while(running) {
        fd_set readset;

        // Initialize the set
        FD_ZERO(&readset);
        FD_SET(pipeMotor.get_fd(), &readset);
        FD_SET(pipeAti.get_fd(), &readset);
        FD_SET(pipeControl.get_fd(), &readset);

        struct timeval tv;
        // Initialize time out struct
        tv.tv_sec = 0;
        tv.tv_usec = 1000;

        int maxFd = std::max({pipeMotor.get_fd(),pipeAti.get_fd(), pipeControl.get_fd() });
        auto result = select(maxFd + 1, &readset, NULL, NULL, &tv);

        iit::ecat::advr::McEscPdoTypes::pdo_rx pdrx;
        auto bytes = pipeMotor.xddp_read((uint8_t *)&pdrx, sizeof(pdrx));
        auto now = iit::ecat::get_time_ns();
        auto absSec = ( now - originalTime) / ( (1.0)  * NSEC_PER_SEC);
        auto sec = ( now - starttime) / ( (1.0)  * NSEC_PER_SEC);

        if ( bytes > 0)
        {
            std::lock_guard< std::mutex > lock(data_mutex);
            vec2.append(QPointF(absSec, pdrx.torque));
        }

        float ft[6] = {};
        bytes = pipeAti.xddp_read((uint8_t *)&ft[0], sizeof(ft));
        if ( bytes > 0)
        {
            std::lock_guard< std::mutex > lock(data_mutex);
            vec.append(QPointF(absSec, ft[5]));
        }

        Msg buffControl;
        bytes = pipeControl.xddp_read((uint8_t *)&buffControl, sizeof(buffControl));
        if ( bytes > 0)
        {
            switch (buffControl.id)
            {
                case ID::ID_SAMPLE:
                {
                    double sam = buffControl.curr.sample;
                    std::lock_guard< std::mutex > lock(data_mutex);
                    vec3.append(QPointF(absSec,sam));
                }
                break;
                case ID::ID_ACK:
                {
                    switch (buffControl.ack.command)
                    {
                        case Commands::CMD_STARTCALIB:
                        {
                            DPRINTF("StartCalibration OK\n");
                            emit calibration(true);
                        }
                        break;

                    }
                }
                break;
                case ID::ID_REGRESULT:
                {
                    DPRINTF("StopCalibration OK\n");
                    emit calibration(false);
                    M = buffControl.res.M;
                    emit mChanged();
                }
                break;
            }
        }
        if (absSec > axMax)
        {

            starttime = iit::ecat::get_time_ns();
            std::lock_guard< std::mutex > lock(data_mutex);
            vec = vec.mid(vec.size()-300);
            vec2 = vec2.mid(vec2.size()-300);
            vec3 = vec3.mid(vec3.size()-300);
            axMin = std::max( {vec[0].x(), vec2[0].x(), vec3[0].x()} );
            axMax=axMin+5.0;

        }

    }
}

int CalibGui::getBid() const
{
    return bid;
}

void CalibGui::setBid(int value)
{
    bid = value;
    emit bidChanged();
}

/**
 * @brief CalibGui::getCountsPerUnit
 * @return
 */
float CalibGui::getCountsPerUnit() const
{
    return countsPerUnit;
}

/**
 * @brief CalibGui::setCountsPerUnit
 * @param value
 */
void CalibGui::setCountsPerUnit(float value)
{
    countsPerUnit = value;
}

/**
 * @brief CalibGui::getNext
 * @param series
 * @param series2
 * @param series3
 * @param axis
 * @param axis2
 * @return
 */
int CalibGui::getNext(QAbstractSeries *series, QAbstractSeries *series2,
                      QAbstractSeries *series3, QAbstractAxis *axis, QAbstractAxis *axis2)
{
    std::lock_guard< std::mutex > lock(data_mutex);
    if (series) {
        QXYSeries *xySeries = static_cast<QXYSeries *>(series);
        // Use replace instead of clear + append, it's optimized for performance
        xySeries->replace(vec);
    }
    if (series2) {
        QXYSeries *xySeries = static_cast<QXYSeries *>(series2);
        // Use replace instead of clear + append, it's optimized for performance
        xySeries->replace(vec2);
    }
    if (series3) {
        QXYSeries *xySeries = static_cast<QXYSeries *>(series3);
        // Use replace instead of clear + append, it's optimized for performance
        xySeries->replace(vec3);
    }
    if (axis)
    {
        QValueAxis* valAx = static_cast<QValueAxis*>(axis);
        valAx->setMin(axMin);
        valAx->setMax(axMax);
    }
    if (axis2)
    {
        QValueAxis* valAx = static_cast<QValueAxis*>(axis2);
        valAx->setMin(axMin);
        valAx->setMax(axMax);
    }
}

/**
 * @brief CalibGui::startCalibration
 * @return
 */
int CalibGui::startCalibration()
{
    DPRINTF(" startCalibration \n");
    Msg buf;
    buf.cmd.id = ID::ID_CMD;
    buf.cmd.command = Commands::CMD_STARTCALIB;
    pipeInput.xddp_write( (uint8_t*) &buf, sizeof(buf));
}

/**
 * @brief CalibGui::startCalibration
 * @return
 */
int CalibGui::sendCFG()
{
    DPRINTF(" sendCFG \n");
    Msg buf;
    buf.cfg.id = ID::ID_CFG;
    buf.cfg.maxCurr = this->maxCurr;
    buf.cfg.maxTorque = this->maxTorque;
    buf.cfg.countsPerUnit = this->countsPerUnit;
    pipeInput.xddp_write( (uint8_t*) &buf, sizeof(buf));
}

/**
 * @brief CalibGui::startCalibration
 * @return
 */
int CalibGui::saveResult()
{
    DPRINTF(" saveResult \n");
    Msg buf;
    buf.cmd.id = ID::ID_CMD;
    buf.cmd.command = Commands::CMD_SAVERESULT;
    pipeInput.xddp_write( (uint8_t*) &buf, sizeof(buf));
}

int CalibGui::resetStiffness()
{
    DPRINTF(" resetStiffness \n");
    Msg buf;
    buf.cmd.id = ID::ID_CMD;
    buf.cmd.command = Commands::CMD_RESETSTIFFNESS;
    pipeInput.xddp_write( (uint8_t*) &buf, sizeof(buf));
}

/**
 * @brief CalibGui::stopMotors
 * @return
 */
int CalibGui::startMotors()
{
    DPRINTF(" startMotors \n");
    Msg buf;
    buf.cmd.id = ID::ID_CMD;
    buf.cmd.command = Commands::CMD_STARTMOTORS;
    pipeInput.xddp_write( (uint8_t*) &buf, sizeof(buf));
}

/**
 * @brief CalibGui::stopMotors
 * @return
 */
int CalibGui::stopMotors()
{
    DPRINTF(" stopMotors \n");
    Msg buf;
    buf.cmd.id = ID::ID_CMD;
    buf.cmd.command = Commands::CMD_STOPMOTORS;
    pipeInput.xddp_write( (uint8_t*) &buf, sizeof(buf));
}

/**
 * @brief CalibGui::stopMotors
 * @return
 */
int CalibGui::requestStiff()
{
    DPRINTF(" requestStiff \n");
    Msg buf;
    buf.cmd.id = ID::ID_CMD;
    buf.cmd.command = Commands::CMD_GETRESULT;
    pipeInput.xddp_write( (uint8_t*) &buf, sizeof(buf));
}

/**
 * @brief CalibGui::zeroOffset
 * @return
 */
int CalibGui::zeroOffset()
{
    DPRINTF(" zeroOffset \n");
    Msg buf;
    buf.cmd.id = ID::ID_CMD;
    buf.cmd.command = Commands::CMD_ZEROATI;
    pipeInput.xddp_write( (uint8_t*) &buf, sizeof(buf));
}
