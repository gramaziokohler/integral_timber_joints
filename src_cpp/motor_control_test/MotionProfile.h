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
    long millisSinceStart();
    long getCurrentStep();
    bool isCompleted();
    bool isRunning();

    private:
    long _startPos = 0;
    long _endPos = 0;

    double _velocity_StepPerMillis = 1.0;
    long _startTime = 0; //millis() when the motion starts
    long _endTime = 0; //millis() when the motion stops
    bool _started = false;

};

//extern DCMotor DCMotor;

#endif

