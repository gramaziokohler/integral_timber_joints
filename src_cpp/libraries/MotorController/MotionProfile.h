// DCMotor.h

#ifndef _MOTIONPROFILE_h
#define _MOTIONPROFILE_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif
class MotionProfile {
    public:

    virtual unsigned long  getTotalDurationMicros() = 0;
    virtual long getCurrentStep() = 0;
    virtual double getCurrentSpeed() = 0;

    void start();
    unsigned long getStartTimeMicros();
    unsigned long getElaspsedTimeMicros();

    bool isStarted();
    bool isRunning();
    bool isCompleted();

    protected:

    bool _started = false;
    bool _completed = false;
    unsigned long _startTimeMicros = 0; //micros() when the motion starts

};

class LinearMotionProfile : public MotionProfile{

    public:
    LinearMotionProfile(long startPos, long endPos, double velocity_StepPerSec);

    long getCurrentStep();
    double getCurrentSpeed();
    unsigned long getTotalDurationMicros();

    private:
    long _startPos = 0;
    long _endPos = 0;
    double _velocity_StepPerSec = 1.0;
    unsigned long _totalDurationMicros = 0;

};


class TrapezoidalMotionProfile : public MotionProfile {

    public:
    TrapezoidalMotionProfile(long startPos, long endPos, double velocityMax_StepPerSec, double accel_StepPerSecSq);

    long getCurrentStep();
    double getCurrentSpeed();
    unsigned long getTotalDurationMicros();
    int getPhase();

    unsigned long _phase1End_Micros = 0;
    unsigned long _phase2End_Micros = 0;
    unsigned long _phase3End_Micros = 0;

    private:
    long _startPos = 0;
    long _endPos = 0;
    double _velocityMax_StepPerSec = 1.0;
    double _accel_StepPerSecSq = 1.0;

    double _fullAccelTime_Sec = 0.0;
    double _fullAccelDist_Steps = 0.0;

    bool isTriangularProfile = false;


};

#endif

