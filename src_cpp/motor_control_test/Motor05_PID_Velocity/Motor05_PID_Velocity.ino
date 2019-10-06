/*
 Name:		Motor05_PID_Velocity.ino
 Created:	10/6/2019 6:34:52 PM
 Author:	leungp

 This code allow setting the target velocity in rev/ms.
 A PID Controller will adjust the PWM output to maintain the set speed.
 Plots the output 
*/

// 

#include "DCMotor.h"

const uint8_t m1_driver_ena_pin = 9;             // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t m1_driver_in1_pin = 8;             // the pin the motor driver IN1 is attached to
const uint8_t m1_driver_in2_pin = 7;             // the pin the motor driver IN2 is attached to
DCMotor Motor1(m1_driver_ena_pin, m1_driver_in1_pin, m1_driver_in2_pin);

#include "Encoder.h"
constexpr int stepPerRev = (60 * 49);
constexpr long reportInterval = 200; //millis
Encoder myEnc(2, 3);

long current_rev_per_ms = 0;

void setup() {
    TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to 8 for PWM frequency of 3921.16 Hz
    Serial.begin(115200);
    Serial.setTimeout(10);
    Serial.println("Motor RevPerSec Test:");
    Motor1.setSpeed(128);
}



// Unfinished function
long compute_rev_per_ms() {
    unsigned long nextReportTime = 0;
    static long oldPosition = -999;
    long newPosition = myEnc.read();
    if (millis() > nextReportTime) {
        long deltaPosition = newPosition - oldPosition;
        long result = deltaPosition / stepPerRev / reportInterval;
        oldPosition = newPosition;
        nextReportTime = millis() + reportInterval;
    }
}
void loop() {
    
    current_rev_per_ms = compute_rev_per_ms();
    Serial.println(current_rev_per_ms);
    if (Serial.available()) {
        int value = Serial.parseInt();
        if (value >= -255 && value <= 255) {
            Serial.print("New PWM:");
            Serial.println(value);
            Motor1.setSpeed(value);
        }

    }
}
