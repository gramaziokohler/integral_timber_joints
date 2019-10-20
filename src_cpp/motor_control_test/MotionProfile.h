// DCMotor.h

#ifndef _MOTIONPROFILE_h
#define _MOTIONPROFILE_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class LinearMotionProfile {
    public:

    LinearMotionProfile(long startPos, long endPos, double velocity_StepPerSec);

    void start();
    long microsSinceStart();
    long getCurrentStep();
    long getStartTimeMicros();

    bool isStarted();
    bool isRunning();
    bool isCompleted();

    private:
    long _startPos = 0;
    long _endPos = 0;

    double _velocity_StepPerSec = 1.0;
    unsigned long _startTimeMicros = 0; //micros() when the motion starts
    unsigned long _durationIntervalMicros = 0; //number of microseconds for the entire motion to finish

    bool _started = false;
    bool _completed = false;
    
};

//extern DCMotor DCMotor;

#endif

