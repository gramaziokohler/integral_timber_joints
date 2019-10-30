/*
 Name:		Motor07_PID_Motion Profile.ino
 Created:	14/10/2019
 Author:	leungp

 This code performs a motion profile / positional error feedback control.
 A number of PID settings are tested and best effort is used to tune this PID control.

 A simple rectangular speed profile will be used.
 The error and settling time of the system during acceleration and deceleration is analysed.

The 775 Motor used for testing has 2940 steps.

*/


#include "DCMotor.h"

const uint8_t m1_driver_ena_pin = 9;             // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t m1_driver_in1_pin = 8;             // the pin the motor driver IN1 is attached to
const uint8_t m1_driver_in2_pin = 7;             // the pin the motor driver IN2 is attached to
DCMotor Motor1(m1_driver_ena_pin, m1_driver_in1_pin, m1_driver_in2_pin);

#include "Encoder.h"
//constexpr int stepPerRev = (60 * 49);
constexpr long speedEvalInterval = 20; //millis
Encoder myEnc(2, 3);

#include <PID_v1.h>
//PID myPID(&current_rev_per_s, &motorPwmValue, &target_rev_per_s, 10, 20, 0.02, DIRECT);

#include "MotionProfile.h"
//
//double compute_step_per_s(Encoder encoder) {
//    //Static Variables Declaration
//    static long lastPosition = 0;
//    static unsigned long lastReportTimeMicros = 0;
//
//    //Compute Delta Step
//    long deltaPosition = myEnc.read() - lastPosition;
//    lastPosition = myEnc.read();
//
//    //Compute Delta Time
//    long deltaTimeMicros = micros() - lastReportTimeMicros;
//    lastReportTimeMicros = micros();
//
//    //Compute Speed
//    return ((double) deltaPosition) / (deltaTimeMicros * 1e-6);
//}

void perform_one_test(double kp, double ki, double kd, double velocityStepsPerSec, double preDurationSec, double runDurationSec, double postDuration_Sec) {
    //Print Octave matrix format header to start:
    String testResultName = "result_" + String(kp, 4);
    testResultName += "_" + String(ki, 4);
    testResultName += "_" + String(kd, 4);
    testResultName += "_" + String(velocityStepsPerSec, 0);
    testResultName += "_" + String(runDurationSec, 0);
    testResultName.replace('.', 'p');
    testResultName.replace(" ", "");
    Serial.print(testResultName);
    Serial.println(" = [");

    // Compute steps
    double total_steps = runDurationSec * velocityStepsPerSec;
    long total_duration_micros = (preDurationSec + runDurationSec + postDuration_Sec) * 1e6;

    // Reset Encoder
    myEnc.write(0);

    // Setup new motion profile
    LinearMotionProfile profile = LinearMotionProfile(0, total_steps, velocityStepsPerSec);

    // Setup PID Positional Control Variables
    double current_position_step = 0;
    double target_position_step = 0;
    double motorSpeedPercentage = 0.0;

    // Create PID, Configure PID controller
    PID myPID(&current_position_step, &motorSpeedPercentage, &target_position_step, kp, ki, kd, DIRECT);
    myPID.SetOutputLimits(-1.0, 1.0);
    myPID.SetSampleTime(2);
    myPID.SetMode(1); //Turn on PID


    unsigned long testStartTimeMicros = micros();
    while (micros()- testStartTimeMicros < total_duration_micros) {
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
        reporting(5,(long) micros() - testStartTimeMicros - (preDurationSec * 1e6), target_position_step, current_position_step, motorSpeedPercentage);

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
        Serial.print(target_position_step,1);
        Serial.print(' ');
        Serial.print(current_position_step,0);
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

    //perform_one_test(0.002, 0.006, 0, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.004, 0.006, 0, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.006, 0.006, 0, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.008, 0.006, 0, 2000, 0.2, 2.0, 1.0);

    //perform_one_test(0.010, 0.006, 0, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.010, 0.010, 0, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.010, 0.020, 0, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.010, 0.030, 0, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.010, 0.040, 0, 2000, 0.2, 2.0, 1.0);

    //perform_one_test(0.010, 0.040, 0, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.010, 0.040, 0.0001, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.010, 0.040, 0.0002, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.010, 0.040, 0.0004, 2000, 0.2, 2.0, 1.0);

    //perform_one_test(0.010, 0.040, 0.0001, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.020, 0.040, 0.0001, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.040, 0.040, 0.0001, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.080, 0.040, 0.0001, 2000, 0.2, 2.0, 1.0);

    //perform_one_test(0.040, 0.040, 0.0001, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.040, 0.100, 0.0001, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.040, 0.200, 0.0001, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.040, 0.400, 0.0001, 2000, 0.2, 2.0, 1.0);

    //perform_one_test(0.040, 0.200, 0.0008, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.040, 0.200, 0.0004, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.040, 0.200, 0.0002, 2000, 0.2, 2.0, 1.0);
    //perform_one_test(0.040, 0.200, 0.0001, 2000, 0.2, 2.0, 1.0);

    perform_one_test(0.040, 0.200, 0.0002, 1000, 0.2, 2.0, 1.0);
    perform_one_test(0.040, 0.200, 0.0002, 2000, 0.2, 2.0, 1.0);
    perform_one_test(0.040, 0.200, 0.0002, 3000, 0.2, 2.0, 1.0);
    perform_one_test(0.040, 0.200, 0.0002, 3500, 0.2, 2.0, 1.0);
    perform_one_test(0.040, 0.200, 0.0002, 4000, 0.2, 2.0, 1.0);
}

void loop() {
    //Listen for Serial command
    //if (Serial.available()) {
    //    delay(1);
    //    double kp = Serial.parseFloat();
    //    double ki = Serial.parseFloat();
    //    double kd = Serial.parseFloat();
    //    double velocity = Serial.parseFloat();
    //    double steps = Serial.parseInt();
    //   
    //}

}
