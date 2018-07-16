/**
  */

#ifndef GIEL_H
#define GIEL_H

#include <chrono>
#include <thread>
#include <fstream>
#include <vector>
#include <iostream>
#include <cmath>
#include <cstring>

/// Sleep for X Milliseconds
/// Uses OS sleep NOT Accurate!
#define MILLISLEEP(X) do { \
    std::chrono::milliseconds tim2((X));\
    std::this_thread::sleep_for(tim2);\
} while(0)


template <typename F>
class LinearRegression
{

public:

    LinearRegression() = default;

    void addDataPoint(F newX, F newY)
    {
        sumX += newX;
        sumY += newY;
        sumXX += (newX * newX);
        sumXY += (newX * newY);
        sumYY += (newY * newY);
        numPoints += 1;
    }

    F getM()
    {
        auto partA = (numPoints * sumXY) - (sumX * sumY);
        auto partB = (numPoints * sumXX) - (sumX * sumX);
        if (partB == 0.0f) {
            return 0;
        } else {
            return (partA / partB);
        }
    }

    F getB()
    {
        return (sumY - (this->getM()*sumX)) / numPoints;
    }

    F getR()
    {
        return (sumXY - sumX * sumY / numPoints) /          /* compute correlation coeff     */
            sqrt((sumXX - sumX*(sumX)/numPoints) *
            (sumYY - sumY*(sumY)/numPoints));
    }

    int getNPoints() const { return numPoints; }

    void reset()
    {
        numPoints = 0.0;
        sumX = 0.0;
        sumY = 0.0;
        sumXX = 0.0;
        sumXY = 0.0;
        sumYY = 0.0;
    }

private:
    int numPoints{};
    F sumX{};
    F sumY{};
    F sumXX{};
    F sumXY{};
    F sumYY{};
};

//
class Signal
{
public:
    Signal() = default;
    virtual ~Signal() = default;

    virtual double sample(double t) const = 0;
};
/**
 * @brief The RampSignal class
 */
class RampSignal : public Signal
{
public:
    RampSignal(double _amplitude, double _startTime, double _endTime, double _offset = 0.0):
    amplitude(_amplitude),
    startTime(_startTime),
    endTime(_endTime),
    offset(_offset),
    slope ( startTime == endTime ? 0 : amplitude / (endTime - startTime))
    {}
    ~RampSignal() = default;


    double sample(double t) const override
    {
      double retValue = offset;

      if(t > endTime)
      {
        retValue += amplitude;
      }
      else if(t > startTime)
      {
        retValue += slope * (t - startTime);

        if( ((slope) > 0 && (retValue > offset + amplitude)) ||
            ((slope) < 0 && (retValue < offset + amplitude)) )
        {
          retValue = offset + amplitude;
        }
      }

      return retValue;
    }

    double getAmplitude() const         { return amplitude;       }
    double getStartTime() const         { return startTime;       }
    double getEndTime() const           { return endTime;         }
    double getOffset() const            { return offset;          }

private:
    const double amplitude;
    const double startTime;
    const double endTime;
    const double offset;
    const double slope;
};


/**
 * @brief The Trajectory class
 */
class Trajectory
{
public:
    Trajectory() = default;

    void setUpTrajectory(float maxCurrentForMotor=4.0)
    {
        float currentStep = maxCurrentForMotor / 5.0;
        float timeStep = 0.1;

        float time{};
        for ( auto current=0.0; current < maxCurrentForMotor; current += currentStep)
        {
            this->addRamp(RampSignal{currentStep, time, time+timeStep});
            time += (2*timeStep);
        }

    }

    double sample(double t)
    {
       auto actualSample = 0.0;
       for ( auto& ramp : segments)
       {
           actualSample += ramp.sample(t);
       }
       return actualSample ;
    }

    bool isCompleted(double t)
    {
        return t>endTime;
    }

    void addRamp(const RampSignal& rs)
    {
        segments.push_back(rs);
        endTime = rs.getEndTime() > endTime ? rs.getEndTime() : endTime; // find maximum endtime
    }

    void flush()
    {
        this->segments.clear();
    }

private:

    double endTime{};
    std::vector<RampSignal> segments;

};


/**
 *
 */
template<typename timeUnit>
class ScopedDuration {
public:
    ScopedDuration(typename timeUnit::rep& ticks_):start(std::chrono::high_resolution_clock::now())
      , ticks(ticks_) {} ///< Default Constructor
    ~ScopedDuration() {
        end =  std::chrono::high_resolution_clock::now();
        ticks = std::chrono::duration_cast<timeUnit>(end-start).count();
    }
private:
    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point end;
    typename timeUnit::rep& ticks;
};


/*!
   * \brief The MemBufferLogger class
   * Use a circular buffer do mantain a log in memory and then writes it into a files
   */
class MemBufferLogger
{
public:
    MemBufferLogger(const size_t _N, const std::string& filename_) :
        N{_N}, buf{new char[N] },
        filename(filename_),index(0), full(false)
    {
    } ///< default constructor


    void addRow(const char* line)
    {
        const auto len = strlen(line); // String Length


        auto runningLen = len;
        auto readIdx=0;

        while ( runningLen > 0 )
        {
            auto space = N - index;
            auto toCpy = std::min(space, runningLen);

            memcpy(&buf[index],&line[readIdx],toCpy);

            readIdx+=toCpy;
            index += toCpy;
            runningLen -= toCpy;

            if ( index == N)
            {
                index = 0;
                full = true;
            }

        }

        buf[index]='\n';
        if ( ++index == N)
        {
            index = 0;
            full = true;
        }
    }


    /**
     * @brief write to disk the log
     * Writes the logs to disk
     */
    void commit()
    {
        std::ofstream myfile;
        myfile.open (filename.c_str());

        if (!full)
            myfile.write(buf.get(),index) ;
        else
        {
            auto ptr = buf.get();
            myfile.write(&ptr[index],N-index) ;
            myfile.write(ptr,index) ;
        }

        myfile.flush();
        myfile.close();

    }

private:
    const size_t N;
    std::unique_ptr<char[]> buf;
    const std::string filename;
    size_t index;
    bool full;
};
#endif // GIEL_H
