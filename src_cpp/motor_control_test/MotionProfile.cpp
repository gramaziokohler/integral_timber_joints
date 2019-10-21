/*
TA6586
Pin 1 , Pin 2 , Effect
HIGH , LOW  , Forward
LOW  , HIGH , Reverse
HIGH , HIGH , Both Low (Breaks)
LOW  , LOW  , Open

*/

#include "MotionProfile.h"

void MotionProfile::start() {
    _started = true;
    _completed = false;
    _startTimeMicros = micros();
}

unsigned long MotionProfile::getElaspsedTimeMicros() {
    if (!_started) return 0;
    return (micros() - _startTimeMicros);
}

unsigned long MotionProfile::getStartTimeMicros() {
    return _startTimeMicros;
}

/**
  Checks if the motion is still running.
  It is only meaningful after start() had been called.

  @return true if the motion is still running
*/
bool MotionProfile::isStarted() {
    return _started;
}

bool MotionProfile::isRunning() {
    if (!_started) return false;
    else return (!isCompleted());
}

/**
  Checks if the motion is completed.

  @return true if the motion is completed
*/
bool MotionProfile::isCompleted() {
    if (_completed) return true;
    if (!_started) return false;
    if (getElaspsedTimeMicros() > getTotalDurationMicros()) _completed = true;
    return _completed;
}

LinearMotionProfile::LinearMotionProfile( long startPos, long endPos, double velocity_StepPerSec) {
    _startPos = startPos;
    _endPos = endPos;
       
    //Just to ensure velocity is supplied as absolute value
    double absoluteVelocity = velocity_StepPerSec;
    if (absoluteVelocity < 0.0) velocity_StepPerSec = -velocity_StepPerSec;

    //Fix sign of velocity
    if (_endPos > _startPos) _velocity_StepPerSec = absoluteVelocity;
    else  _velocity_StepPerSec = absoluteVelocity;

    //Compute Interval for checking if the move is over.
    _totalDurationMicros = (_endPos - _startPos) / _velocity_StepPerSec * 1.0e6;
}

long LinearMotionProfile::getCurrentStep() {
    if (!_started) return _startPos;    //Short cut for start position
    if (isCompleted()) return _endPos;  //Short cut for end position
    return (_velocity_StepPerSec * 1.0e-6 * getElaspsedTimeMicros()) + _startPos;
}

double LinearMotionProfile::getCurrentSpeed() {
    return _velocity_StepPerSec;
}

unsigned long LinearMotionProfile::getTotalDurationMicros() {
    return _totalDurationMicros;
}

TrapezoidalMotionProfile::TrapezoidalMotionProfile(long startPos, long endPos, double velocityMax_StepPerSec, double accel_StepPerSecSq) {
    _startPos = startPos;
    _endPos = endPos;
    _velocityMax_StepPerSec = abs(velocityMax_StepPerSec);
    _accel_StepPerSecSq = abs(accel_StepPerSecSq);

    //Compute steps needed for full acceleration
    _fullAccelTime_Sec =  _velocityMax_StepPerSec / _accel_StepPerSecSq;
    _fullAccelDist_Steps = _velocityMax_StepPerSec * _fullAccelTime_Sec * 0.5;
    
    //Decide if this is a trapezoidal or triangular profile
    
    isTriangularProfile = (abs(endPos - startPos) <= _fullAccelDist_Steps * 2);

    if (isTriangularProfile) {
        // In a Triangular Profile, velocityMax is reduced, AccelTime and AccelDist is also reduced.
        _velocityMax_StepPerSec = sqrt(abs(endPos - startPos) * 2 * _accel_StepPerSecSq);
        _fullAccelTime_Sec = _velocityMax_StepPerSec / _accel_StepPerSecSq;
        _fullAccelDist_Steps = _velocityMax_StepPerSec * _fullAccelTime_Sec * 0.5;
    }

    //Compute Total Duration for each phase 
    _phase1End_Micros = _fullAccelTime_Sec * 1000000L;
    if (isTriangularProfile) {
        _phase2End_Micros = _phase1End_Micros;
        _phase3End_Micros = _phase1End_Micros * 2;
    } else {
        _phase3End_Micros = _phase1End_Micros * 2 + (abs(endPos - startPos) - 2 * _fullAccelDist_Steps) / _velocityMax_StepPerSec * 1000000L;
        _phase2End_Micros = _phase3End_Micros - _phase1End_Micros;
    }
}

long TrapezoidalMotionProfile::getCurrentStep() {
    if (getPhase() == 0) return _startPos;
    if (getPhase() == 4) return _endPos;
    // Calculate the integrated steps for each phase
    double currentStepFromStart = 0;
    double t = getElaspsedTimeMicros() * 1e-6;
    if (getPhase() >= 1) {
        double t1 = min(t, (_phase1End_Micros* 1e-6));
        currentStepFromStart += 0.5 * _accel_StepPerSecSq * t1 * t1; //x = 0.5 a t^2
    }
    if (getPhase() >= 2) {
        double t2 = min((t - _phase1End_Micros * 1e-6), (_phase2End_Micros - _phase1End_Micros)* 1e-6);
        currentStepFromStart += _velocityMax_StepPerSec * t2; //x = v t
    }
    if (getPhase() >= 3) {
        double t3 = t - _phase2End_Micros * 1e-6;
        currentStepFromStart += (_velocityMax_StepPerSec * t3 ) - ( 0.5 * _accel_StepPerSecSq * t3 * t3);//x = v t - 0.5 a t^2
    }

    //Return the correct direction.
    if (_endPos > _startPos) {
        return _startPos + currentStepFromStart;
    } else {
        return _startPos - currentStepFromStart;
    }

}

double TrapezoidalMotionProfile::getCurrentSpeed() {
    double directionSign = (_endPos > _startPos) ? 1.0 : -1.0;
    if (getPhase() == 0 || getPhase() == 4) return 0.0;
    if (getPhase() == 1)  return getElaspsedTimeMicros() * _accel_StepPerSecSq * directionSign;
    if (getPhase() == 2)  return _velocityMax_StepPerSec * directionSign;
    if (getPhase() == 3)  return (_phase3End_Micros - getElaspsedTimeMicros()) * _accel_StepPerSecSq * directionSign;
    else return 0.0;
}

unsigned long TrapezoidalMotionProfile::getTotalDurationMicros() {
    return _phase3End_Micros;
}

int TrapezoidalMotionProfile::getPhase() {
    if (!_started) return 0;
    if (getElaspsedTimeMicros() < _phase1End_Micros) return 1;
    if (getElaspsedTimeMicros() < _phase2End_Micros) return 2;
    if (getElaspsedTimeMicros() < _phase3End_Micros) return 3;
    return 4;
}
