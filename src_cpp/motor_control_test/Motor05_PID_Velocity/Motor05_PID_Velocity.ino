/*
 Name:		Motor05_PID_Velocity.ino
 Created:	10/6/2019 6:34:52 PM
 Author:	leungp

 This code allow setting the target velocity in rev/s.
 A PID Controller will adjust the PWM output to maintain the set speed.
 Open Serial at 115200. Send a value between -2.0 to 2.0 to set target velocity.

*/

// There is no specific test in this script
// Manually try to come up with a set of PID values that work in maintaining speed.
// A tune that doesn't generate much oscillation, not too long ramp up etc.
// Observation:
//      50,600,0 Seems to be a stable set of values
//      Very slow speed is movable such as 0.05 , thanks to the I term of the controller.

//(Later further tuning results)
// Increasing the ki terms appear to have faster settling time when the target speed go from 0 to 1.0.
// 50,1000,0 Produce a quick stop and quick start for 0 to 1.0 speed
// Observation:
//      It is very hard to quantify the result.
// Conclusion:
//      A better graphing script is necessary to see the actual difference in settling time and occilation.
//      Higher plotting sampling point necessary



#include "DCMotor.h"

const uint8_t m1_driver_ena_pin = 9;             // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t m1_driver_in1_pin = 8;             // the pin the motor driver IN1 is attached to
const uint8_t m1_driver_in2_pin = 7;             // the pin the motor driver IN2 is attached to
DCMotor Motor1(m1_driver_ena_pin, m1_driver_in1_pin, m1_driver_in2_pin);

#include "Encoder.h"
constexpr int stepPerRev = (60 * 49);
constexpr long speedEvalInterval = 20; //millis
Encoder myEnc(2, 3);


double current_rev_per_s = 0.0;
double target_rev_per_s = 1.0;
double motorPwmValue = 0;

#include <PID_v1.h>
PID myPID(&current_rev_per_s, &motorPwmValue, &target_rev_per_s, 50, 1000, 5, DIRECT); 

//Serial Reporting / Debug
constexpr long serialReportInterval = 100; //millis

void setup() {
    TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to 8 for PWM frequency of 3921.16 Hz
    Serial.begin(115200);
    Serial.setTimeout(10);
    Serial.println("Motor RevPerSec Test:");
    Motor1.setSpeed(0);

    //Configure PID controller
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-255, 255);
    myPID.SetSampleTime(20); //20 ms = 50Hz

}

void compute_rev_per_s() {
    long newPosition = myEnc.read();
    static long oldPosition = 0;
    static unsigned long lastReportTime = 0;
    long deltaTime = millis() - lastReportTime;
    if (deltaTime > speedEvalInterval) {
        lastReportTime = millis();
        long deltaPosition = newPosition - oldPosition;
        current_rev_per_s = (float)deltaPosition / stepPerRev / deltaTime * 1000;
        oldPosition = newPosition;
        //Serial
        //Serial.print(deltaPosition);
        //Serial.print(',');
        //Serial.print(deltaTime);
        //Serial.print(',');
        //Serial.println(current_rev_per_s);
    }
}

void reporting() {
    static unsigned long lastReportTime = 0;
    long deltaTime = millis() - lastReportTime;
    if (deltaTime > serialReportInterval) {
        lastReportTime = millis();
        Serial.print(millis());
        Serial.print(',');
        Serial.print(target_rev_per_s);
        Serial.print(',');
        Serial.print(current_rev_per_s);
        Serial.print(',');
        Serial.print(motorPwmValue);
        Serial.print('\n');
    }

}
void loop() {
    //Listen for Serial command
    if (Serial.available()) {
        double value = Serial.parseFloat();
        if (value >= -2.0 && value <= 2.0) {
            Serial.print("New Target setpoint:");
            Serial.println(value);
            target_rev_per_s = value;
        }
    }

    //Compute RPM (for PID Input)
    compute_rev_per_s();

    //Compute PID
    myPID.Compute();

    //Set Motor PWM based on PID Output
    Motor1.setSpeed(motorPwmValue);

    //Report
    reporting();



}
