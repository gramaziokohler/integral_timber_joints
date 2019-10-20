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

    //Fix sign of velocity
    if (_endPos > _startPos) _velocity_StepPerSec = absoluteVelocity;
    else  _velocity_StepPerSec = absoluteVelocity;
}

void LinearMotionProfile::start() {
    _started = true;
    _completed = false;
    _startTimeMicros = micros();
    //Pre compute end time
    //_endTime = (_endPos - _startPos) / _velocity_StepPerMillis + _startTime;
    //Compute Interval for checking if the move is over.
    _durationIntervalMicros = (_endPos - _startPos) / _velocity_StepPerSec * 1.0e6 ;
}

long LinearMotionProfile::microsSinceStart() {
    return (micros() - _startTimeMicros);
}

long LinearMotionProfile::getCurrentStep() {
    if (!_started) return _startPos;    //Short cut for start position
    if (isCompleted()) return _endPos;  //Short cut for end position
    return (_velocity_StepPerSec * 1.0e-6 * microsSinceStart()) + _startPos;
}

long LinearMotionProfile::getStartTimeMicros() {
    return _startTimeMicros;
}

/**
  Checks if the motion is still running.
  It is only meaningful after start() had been called.

  @return true if the motion is still running
*/
bool LinearMotionProfile::isStarted() {
	  return _started;
  }
  bool LinearMotionProfile::isRunning() {
    if (!_started) return false;
    else return (! isCompleted());
}

/**
  Checks if the motion is completed.

  @return true if the motion is completed
*/
bool LinearMotionProfile::isCompleted() {
    if (_completed) return true;
    if (microsSinceStart() > _durationIntervalMicros) _completed = true;
    return _completed;
}
