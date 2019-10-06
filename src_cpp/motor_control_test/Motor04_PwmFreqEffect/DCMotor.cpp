/*
TA6586
Pin 1 , Pin 2 , Effect
HIGH , LOW  , Forward
LOW  , HIGH , Reverse
HIGH , HIGH , Both Low (Breaks)
LOW  , LOW  , Open

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

void DCMotor::stop() {
    digitalWrite(_pin_ena, LOW);
    _running = false;
}
