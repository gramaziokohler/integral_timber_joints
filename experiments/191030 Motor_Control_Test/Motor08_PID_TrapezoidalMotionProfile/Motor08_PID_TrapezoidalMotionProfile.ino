/*
 Name:		Motor08_PID_TrapezoidalMotionProfile.ino
 Created:	21/10/2019
 Author:	leungp

*/


#include "DCMotor.h"

const uint8_t m1_driver_ena_pin = 9;             // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t m1_driver_in1_pin = 8;             // the pin the motor driver IN1 is attached to
const uint8_t m1_driver_in2_pin = 7;             // the pin the motor driver IN2 is attached to
DCMotor Motor1(m1_driver_ena_pin, m1_driver_in1_pin, m1_driver_in2_pin);

#include "Encoder.h"
Encoder myEnc(3, 2);

#include <PID_v1.h>

#include "MotionProfile.h"

void perform_one_test(double kp, double ki, double kd, double velocityStepsPerSec, double accelStepsPerSecSq, double preDurationSec, double runDurationSec, double postDuration_Sec) {
    //Print Octave matrix format header to start:
    String testResultName = "result_" + String(kp, 4);
    testResultName += "_" + String(ki, 4);
    testResultName += "_" + String(kd, 4);
    testResultName += "_" + String(velocityStepsPerSec, 0);
    testResultName += "_" + String(accelStepsPerSecSq, 0);
    testResultName += "_" + String(runDurationSec, 0);
    testResultName.replace('.', 'p');
    testResultName.replace(" ", "");
    Serial.print(testResultName);
    Serial.println(" = [");

    // Compute steps
    double total_steps = runDurationSec * velocityStepsPerSec;


    // Reset Encoder
    myEnc.write(0);

    // Setup new motion profile
    //LinearMotionProfile profile = LinearMotionProfile(0, total_steps, velocityStepsPerSec);
    TrapezoidalMotionProfile profile = TrapezoidalMotionProfile(0, total_steps, velocityStepsPerSec, accelStepsPerSecSq);

    //Serial.println("_phase1End_Micros=" + String(profile._phase1End_Micros));
    //Serial.println("_phase2End_Micros=" + String(profile._phase2End_Micros));
    //Serial.println("_phase3End_Micros=" + String(profile._phase3End_Micros));

    // Setup PID Positional Control Variables
    double current_position_step = 0;
    double target_position_step = 0;
    double motorSpeedPercentage = 0.0;

    // Create PID, Configure PID controller
    PID myPID(&current_position_step, &motorSpeedPercentage, &target_position_step, kp, ki, kd, DIRECT);
    myPID.SetOutputLimits(-1.0, 1.0);
    myPID.SetSampleTime(2);
    myPID.SetMode(1); //Turn on PID

    // Compute Duration
    long total_duration_micros = preDurationSec * 1e6  + profile.getTotalDurationMicros() + postDuration_Sec * 1e6 ;
    unsigned long testStartTimeMicros = micros();

    while (micros() - testStartTimeMicros < total_duration_micros) {
        if ((!profile.isStarted()) && ((micros() - testStartTimeMicros) > (preDurationSec * 1e6))) {
            // Start Motion Profile
            profile.start();
        }
        //Read encoder
        current_position_step = myEnc.read();

        //Read motion profile
        target_position_step = profile.getCurrentStep();

        //Compute PID
        myPID.Compute();

        //Set Motor PWM based on PID Output
        Motor1.setSpeedPercent(motorSpeedPercentage);

        //Report
        reporting(5, (long)micros() - testStartTimeMicros - (preDurationSec * 1e6), target_position_step, current_position_step, motorSpeedPercentage);


    }

    //Finalize the test (stop motor if PID didn't stop it perfectly)
    Motor1.stop();
    myPID.SetMode(0);

    //Print Octave matrix format footer to start:
    Serial.println("]; % " + testResultName);
}

void reporting(long ReportIntervalMillis, long time, double target_position_step, double current_position_step, double motorSpeedPercentage) {
    static unsigned long lastReportTime = 0;
    long deltaTime = millis() - lastReportTime;
    if (deltaTime > ReportIntervalMillis) {
        lastReportTime = millis();
        Serial.print(time);
        Serial.print(' ');
        Serial.print(target_position_step, 1);
        Serial.print(' ');
        Serial.print(current_position_step, 0);
        Serial.print(' ');
        Serial.print(motorSpeedPercentage);
        Serial.print('\n');
    }
}

void setup() {
    TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to 8 for PWM frequency of 3921.16 Hz
    Serial.begin(115200);
    Serial.setTimeout(10);
    Motor1.setSpeedPercent(0.0);

    perform_one_test(0.040, 0.200, 0.0002, 2970, 1000, 0.2, 4.0, 1.0);

}

void loop() {

}
