/*
TA6586
Pin 1 , Pin 2 , Effect
HIGH , LOW  , Forward
LOW  , HIGH , Reverse
HIGH , HIGH , Both Low (Breaks)
LOW  , LOW  , Open

*/

#include "MotionProfile.h"

LinearMotionProfile::LinearMotionProfile( long startPos, long endPos, double velocity_StepPerSec) {
    _startPos = startPos;
    _endPos = endPos;
       
    //Just to ensure velocity is supplied as absolute value
    double absoluteVelocity = velocity_StepPerSec;
    if (absoluteVelocity < 0.0) velocity_StepPerSec = -velocity_StepPerSec;

    //Convert velocity to millis and fix sign of velocity
    if (_endPos > _startPos) _velocity_StepPerMillis = absoluteVelocity / 1000;
    else  _velocity_StepPerMillis = absoluteVelocity / -1000;
}

void LinearMotionProfile::start() {
    _started = true;
    _startTime = millis();
    //Pre compute end time
    _endTime = (_endPos - _startPos) / _velocity_StepPerMillis + _startTime;
}

long LinearMotionProfile::millisSinceStart() {
    return (millis() - _startTime);
}

long LinearMotionProfile::getCurrentStep() {
    if (!_started) return _startPos;
    if (isCompleted()) return _endPos;
    return (_velocity_StepPerMillis * millisSinceStart()) + _startPos;
}

bool LinearMotionProfile::isCompleted() {
    if (!_started) return false;
    else return (millis() >= _endTime);
}

bool LinearMotionProfile::isRunning() {
    if (!_started) return false;
    else return (millis() < _endTime);
}

