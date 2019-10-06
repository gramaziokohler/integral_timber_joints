/*
 Name:		Motor04_SpeedControl_OpenLoop.ino
 Created:	9/27/2019 5:26:56 PM
 Author:	leungp

 This code allow setting the PWM speed and PWM frequency for a motor.
Monitoring the speed via the encoder

*/

// Test Result for 775 (1:49) No Load. Input voltage 13V
// stepPerRev = (60*49);
// | PWM Freq | PWM Value                 |
// | -------- | ---- | ---- | ---- | ---- |
// |          | 64   | 128  | 192  | 255  |
// | -------- | ---- | ---- | ---- | ---- |
// | 122 Hz   | 1.32 | 1.45 | 1.5  | 1.52 |
// | 490 Hz   | 1.11 | 1.39 | 1.47 | 1.52 |
// | 3921 Hz  | 0.39 | 0.94 | 1.22 | 1.53 |
// | 31372 Hz | 0    | 0.39 | 0.91 | 1.53 | (Burnt the motor line capacitor)



#include "DCMotor.h"

const uint8_t m1_driver_ena_pin = 9;             // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t m1_driver_in1_pin = 8;             // the pin the motor driver IN1 is attached to
const uint8_t m1_driver_in2_pin = 7;             // the pin the motor driver IN2 is attached to
DCMotor Motor1(m1_driver_ena_pin, m1_driver_in1_pin, m1_driver_in2_pin);

#include "Encoder.h"
constexpr int stepPerRev = (60 * 49);
constexpr long reportInterval = 200; //millis
Encoder myEnc(2, 3);

void setup() {
    //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to     1024 for PWM frequency of   30.64 Hz
    //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to     256 for PWM frequency of    122.55 Hz
    //TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to     64 for PWM frequency of     490.20 Hz (The DEFAULT)
    TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of      3921.16 Hz
    //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of      31372.55 Hz
    Serial.begin(115200);
    Serial.setTimeout(10);
    Serial.println("Motor RevPerSec Test:");
    Motor1.setSpeed(128);
}

long oldPosition = -999;
unsigned long nextReportTime = 0;

void loop() {
    long newPosition = myEnc.read();
    if (millis() > nextReportTime) {
        long deltaPosition = newPosition - oldPosition;
        Serial.println((float)deltaPosition / stepPerRev / reportInterval * 1000);
        oldPosition = newPosition;
        nextReportTime = millis() + reportInterval;
    }
    if (Serial.available()) {
        int value = Serial.parseInt();
        if (value >= -255 && value <= 255) {
            Serial.print("New PWM:");
            Serial.println(value);
            Motor1.setSpeed(value);
        }

    }
}
