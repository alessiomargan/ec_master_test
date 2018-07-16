#include "calibrationcontroller.h"

static constexpr auto deadBandRegression{1.0f}; /// below this current it is not useful

/**
 * @brief CalibrationController::sample
 * @param seconds
 * @param atiTz
 * @param motorTorque
 * @return
 */
double CalibrationController::sample(double seconds, float atiTz, float motorTorque)
{
    lastTime = seconds;

    switch (state)
    {
        case States::IDLE:
            actualState = 0.0;
        break;
        case States::EVOLVING1:
        {
            actualState = trajectory.sample(lastTime-lastOrigin);
            if (trajectory.isCompleted(lastTime-lastOrigin))
            {
                trajectory.flush();
                lastOrigin = lastTime;
                state = States::EVOLVING2;
            }
            if ( std::abs(actualState) > deadBandRegression)
            {
                lr.addDataPoint(motorTorque, atiTz);
            }

        }
        break;
        case States::EVOLVING2:
        {
            actualState = 0.0;
            if (lastTime-lastOrigin > 0.1)
            {
                trajectory.flush();
                lastOrigin = lastTime;
                state = States::EVOLVING3;
                trajectory.setUpTrajectory(current);
            }
            if ( std::abs(actualState) > deadBandRegression )
            {
                lr.addDataPoint(motorTorque, atiTz);
            }
        }
        break;
        case States::EVOLVING3:
        {
            actualState = - trajectory.sample(lastTime-lastOrigin);
            if (trajectory.isCompleted(lastTime-lastOrigin))
            {
                trajectory.flush();
                state = States::IDLE;
                calibrationComplete = true;
                lastOrigin = lastTime;
            }
            if ( std::abs(actualState) > deadBandRegression )
            {
                lr.addDataPoint(motorTorque, atiTz);
            }

        }
        break;
    }


    return actualState;
}


