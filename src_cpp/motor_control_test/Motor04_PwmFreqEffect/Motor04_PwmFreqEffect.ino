/*
 Name:		Motor04_SpeedControl_OpenLoop.ino
 Created:	9/27/2019 5:26:56 PM
 Author:	leungp

 This code allow setting the PWM speed and PWM frequency for a motor.
Monitoring the speed via the encoder

*/

// Test Result for 775 (1:49) No Load. Input voltage 13V
// 0.1u Capcacitor attached across motor
// stepPerRev = (60*49);
// | PWM Freq | PWM Value                 |
// | -------- | ---- | ---- | ---- | ---- |
// |          | 64   | 128  | 192  | 255  |
// | -------- | ---- | ---- | ---- | ---- |
// | 122 Hz   | 1.32 | 1.45 | 1.5  | 1.52 |
// | 490 Hz   | 1.11 | 1.39 | 1.47 | 1.52 |
// | 3921 Hz  | 0.39 | 0.94 | 1.22 | 1.53 |
// | 31372 Hz | 0    | 0.39 | 0.91 | 1.53 | (Burnt the motor line capacitor)


// No more capcacitor attached across motor
// | PWM Freq | PWM Value                 |
// | -------- | ---- | ---- | ---- | ---- |
// |          | 64   | 128  | 192  | 255  |
// | -------- | ---- | ---- | ---- | ---- |
// | 30.6 Hz  | 1.35 | 1.44 | 1.48 | 1.5  |
// | 122  Hz  | 1.30 | 1.43 | 1.47 | 1.5  |
// | 490  Hz  | 1.06 | 1.35 | 1.44 | 1.5  |
// | 3921 Hz  | 0.39 | 0.93 | 1.21 | 1.5  |
// | 31372 Hz | Not dare to test anymore  |

// Conclusion: In both test 3912 Hz has the the most linear response.
// We should use this frequency in the future for the PID control

// Note on the 31372 Hz burnt the first cap
// reactance Xc = 1/(2*Pi*f*C)
// The 0.1uF cap has Reactance Xc = 160000/f
// In the case of 31372Hz, the Reactance is 5 Ohms.
// When 6V to 9V is applied, the current is more than 1A, thus the cap exploded. :(
// In the case of 3921 Hz, the Reactance is about 40 Ohms. should be fine.

#include "DCMotor.h"

const uint8_t m1_driver_ena_pin = 9;             // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t m1_driver_in1_pin = 8;             // the pin the motor driver IN1 is attached to
const uint8_t m1_driver_in2_pin = 7;             // the pin the motor driver IN2 is attached to
DCMotor Motor1(m1_driver_ena_pin, m1_driver_in1_pin, m1_driver_in2_pin);

#include "Encoder.h"
constexpr int stepPerRev = (60 * 49);
constexpr long reportInterval = 50; //millis
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
    static unsigned long lastReportTime = 0;
    long deltaTime = millis() - lastReportTime;
    if (deltaTime > reportInterval) {
        nextReportTime = millis() + reportInterval;
        lastReportTime = millis();
        long deltaPosition = newPosition - oldPosition;
        Serial.print(deltaPosition);
        Serial.print(',');
        Serial.print(deltaTime);
        Serial.print(',');
        Serial.println((float)deltaPosition / stepPerRev / deltaTime * 1000);
        oldPosition = newPosition;
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
