/*
TA6586
Pin 1 , Pin 2 , Effect
HIGH , LOW  , Forward
LOW  , HIGH , Reverse
HIGH , HIGH , Both Low (Breaks)
LOW  , LOW  , Open

12/24V/7A160W Dual HBridge L298 Logic:

Pin 1 , Pin 2 , Ena, Effect
HIGH , LOW  , PWM   , Forward
LOW  , HIGH , PWM   , Reverse
LOW  , LOW  , X     , Both Low (Breaks)
HIGH , HIGH , X     , Open

*/ 

#include "DCMotor.h"

DCMotor::DCMotor(int pin_ena, int pin_in1, int pin_in2) {
    _pin_ena = pin_ena;
    _pin_in1 = pin_in1;
    _pin_in2 = pin_in2;
    pinMode(_pin_ena, OUTPUT);
    pinMode(_pin_in1, OUTPUT);
    pinMode(_pin_in2, OUTPUT);
}

void DCMotor::setSpeed(int speed) {
    if (speed == 0) stop();
    else if (speed > 0) {
        digitalWrite(_pin_in1, LOW);
        digitalWrite(_pin_in2, HIGH);
        analogWrite(_pin_ena, speed);
        _running = true;
    }
    else if (speed < 0) {
        digitalWrite(_pin_in1, HIGH);
        digitalWrite(_pin_in2, LOW);
        analogWrite(_pin_ena, -speed);
        _running = true;
    }
}
/*
@param: int PWMDeadband : Acceptable range 0 to 250,

Set this to the lowest positive PWM value, just before the motor starts to spin at no load.
*/
void DCMotor::setPWM_Deadband(int PWMDeadband) {
    _PWMDeadband = PWMDeadband;
}

/*
@param: double speedPercent : Acceptable range -1.0 to 1.0
*/
void DCMotor::setSpeedPercent(double speedPercent) {
    if (speedPercent == 0) stop();

    else if (speedPercent > 0) {
        digitalWrite(_pin_in1, LOW);
        digitalWrite(_pin_in2, HIGH);
    } else if (speedPercent < 0) {
        digitalWrite(_pin_in1, HIGH);
        digitalWrite(_pin_in2, LOW);
    }
    //Make speed absolute value, map the value to avoid deadband
    double hamonizedSpeed = abs(speedPercent);
    hamonizedSpeed = min(hamonizedSpeed, 1.0);
    int pwm = (255 - _PWMDeadband) * hamonizedSpeed + _PWMDeadband;
    analogWrite(_pin_ena, pwm);
    _running = true;
}

void DCMotor::stop() {
    digitalWrite(_pin_in1, LOW);
    digitalWrite(_pin_in2, LOW);
    digitalWrite(_pin_ena, LOW);
    _running = false;
}
