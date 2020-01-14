/*
 Name:		Motor04c_PWM_Different_Motor_Speed.ino
 Created:	12/6/2019 11:23:42 AM
 Author:	leungp

 This script can work with any motors gearbox combination.
 Default is to measure no load speed. Report in Steps / Second
*/

#include "DCMotor.h"

const uint8_t m1_driver_ena_pin = 9;             // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t m1_driver_in1_pin = 8;             // the pin the motor driver IN1 is attached to
const uint8_t m1_driver_in2_pin = 7;             // the pin the motor driver IN2 is attached to
DCMotor Motor1(m1_driver_ena_pin, m1_driver_in1_pin, m1_driver_in2_pin);

#include "Encoder.h"
//constexpr int stepPerRev = (60 * 49);
constexpr long reportInterval = 50; //millis
Encoder myEnc(2, 3);

void perform_one_test(int pwm, double durationSec = 1.0) {
    // Set Motor pwm to test value
    Motor1.setSpeed(pwm);
    // Print the first variable
    Serial.print(pwm);
    Serial.print(' ');
    // Wait until motor speed settle
    delay(1000);
    // Perform step counting for a duration
    unsigned long startTimeMicros = micros();
    long startPos = myEnc.read();
    while (micros() - startTimeMicros < durationSec * 1000000) {

    }
    long endPos = myEnc.read();
    // Print the result back + newline
    Serial.println((endPos - startPos) / durationSec);
}

void perform_multiple_test(int pwmStart, int pwmEnd, int step, double durationSec = 1.0) {
    for (int pwm = pwmStart; pwm < pwmEnd; pwm = pwm + step) {
        perform_one_test(pwm, durationSec);
    }
}

void setup() {

    Serial.begin(115200);
    Serial.setTimeout(10);
    //Serial.println("Motor RevPerSec Test:");

    Motor1.setSpeed(0);

    TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of      3921.16 Hz

    Serial.println("result = [");
    perform_multiple_test(-255, 255, 10, 1.0);
    Serial.println("] % result");

    Motor1.setSpeed(0);

}


void loop() {

}
