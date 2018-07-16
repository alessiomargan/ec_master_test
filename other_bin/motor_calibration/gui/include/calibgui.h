#ifndef CALIBGUI_H
#define CALIBGUI_H

#include <QObject>
#include <QVector>
#include <QPointF>
#include <QDateTime>
#include <QtCharts/QAbstractSeries>
#include <QtCharts/QValueAxis>

#include <iit/ecat/utils.h>
#include <iit/advr/thread_util.h>
#include <iit/ecat/advr/pipes.h>
#include <iit/ecat/advr/ec_boards_iface.h>

#include <thread>
#include <mutex>
#include <atomic>
#include "giel.h"
#include <cinttypes>

QT_CHARTS_USE_NAMESPACE

/**
 * @brief The CalibGui class
 */
class CalibGui : public QObject
{
    Q_OBJECT
public:
    CalibGui():vec(20000),vec2(20000), vec3(20000)  {pipeInput.init("EC_board_input");  }
    ~CalibGui() override {running = false;}
    Q_INVOKABLE void startCollect() { auto th = std::thread( &CalibGui::gatherDataThread, this ); th.detach();}
    Q_INVOKABLE int getNext(QAbstractSeries *series, QAbstractSeries *series2, QAbstractSeries *series3, QAbstractAxis *axis, QAbstractAxis *axis2)  ;
    Q_INVOKABLE int startCalibration();
    Q_INVOKABLE int saveResult();
    Q_INVOKABLE int resetStiffness();
    Q_INVOKABLE int startMotors();
    Q_INVOKABLE int sendCFG();
    Q_INVOKABLE int stopMotors();
    Q_INVOKABLE int requestStiff();
    Q_INVOKABLE int zeroOffset();
    Q_PROPERTY(int bid READ getBid NOTIFY bidChanged)
    Q_PROPERTY(float M READ getM NOTIFY mChanged)
    Q_PROPERTY(float maxCurr READ getMaxCurr WRITE setMaxCurr)
    Q_PROPERTY(float maxTorque READ getMaxTorque WRITE setMaxTorque)
    Q_PROPERTY(float countsPerUnit READ getCountsPerUnit WRITE setCountsPerUnit)

    float getM() const { return M; }

    float getMaxCurr() const { return maxCurr; }
    void setMaxCurr(float maxCurr_) { maxCurr = maxCurr_; }

    float getMaxTorque() const { return maxTorque; }
    void setMaxTorque(float maxTorque_) { maxTorque = maxTorque_; }

    float getCountsPerUnit() const;
    void setCountsPerUnit(float value);

    int getBid() const;
    void setBid(int value);

signals:
    void enableGui();
    void calibration(bool ongoing);
    void mChanged();
    void bidChanged();

private:
    int gatherDataThread();
    int requestInitialData();

    mutable std::mutex data_mutex;
    QVector<QPointF> vec;
    QVector<QPointF> vec2;
    QVector<QPointF> vec3;
    float axMin{0.0};
    float axMax{5.0};
    std::atomic_bool running{true};
    bool calibrating{false};
    float M{};
    float maxCurr{};
    float maxTorque{};
    float countsPerUnit{1000.};
    int bid{-1};

    XDDP_pipe pipeMotor;
    XDDP_pipe pipeControl;
    XDDP_pipe pipeAti;
    XDDP_pipe pipeInput;

};


#endif // CALIBGUI_H
