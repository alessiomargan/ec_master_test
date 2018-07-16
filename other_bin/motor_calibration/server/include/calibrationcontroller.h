#ifndef CALIBRATIONCONTROLLER_H
#define CALIBRATIONCONTROLLER_H

#include <chrono>
#include <thread>
#include <iostream>

#include "giel.h"

struct CalResult
{
    float M;
    float B;
    float R;
};

class CalibrationController
{
public:
    CalibrationController(float current_):state(States::IDLE),
        current{current_}
    {}
    ~CalibrationController()
    { }

    CalResult getResult()
    {
        return { std::abs(static_cast<float> (lr.getM())), static_cast<float> (lr.getB()),static_cast<float> (lr.getR())  };
    }

    double sample(double seconds, float atiTz, float motorTorque);

    bool startCalibration(float current_)
    {
        if (calibrationOnGoing())
            return false;
        if ( state != States::IDLE)
            return false;

        current = current_;
        trajectory.setUpTrajectory(current);
        lastOrigin = lastTime;
        onGoing = true;
        calibrationComplete = false;
        state = States::EVOLVING1;
        return true;
    }

    bool calibrationEnded() const
    {
        return calibrationComplete;
    }

    bool calibrationOnGoing()
    {
        return onGoing;
    }

    void reset()
    {
        onGoing = false;
        actualState = 0.0;
        lr.reset();
    }

private:
    enum class States {
        IDLE,
        EVOLVING1,
        EVOLVING2,
        EVOLVING3,
    };

    States state;
    float current;
    double actualState{};
    Trajectory trajectory;
    LinearRegression<double> lr;
    double lastTime{};
    double lastOrigin{};
    bool calibrationComplete{false};
    bool onGoing{false};
};

#endif // CALIBRATIONCONTROLLER_H
