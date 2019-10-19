/*
 Name:		Motor07_PID_Velocity.ino
 Created:	14/10/2019
 Author:	leungp

 This code allow setting the target velocity in rev/s.
 A PID Controller will adjust the PWM output to maintain the set speed.
 Open Serial at baud 115200. Send 5 CSV values.
    Format: kp,ki,kd,velocity,steps
    e.g.: 10,20,0.02,2000,5880
    Note: No line ending

The Motor used for testing has 2940 steps.

*/




#include "DCMotor.h"

const uint8_t m1_driver_ena_pin = 9;             // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t m1_driver_in1_pin = 8;             // the pin the motor driver IN1 is attached to
const uint8_t m1_driver_in2_pin = 7;             // the pin the motor driver IN2 is attached to
DCMotor Motor1(m1_driver_ena_pin, m1_driver_in1_pin, m1_driver_in2_pin);

#include "Encoder.h"
constexpr int stepPerRev = (60 * 49);
constexpr long speedEvalInterval = 20; //millis
Encoder myEnc(2, 3);


double current_position_step = 0;
double current_rev_per_s = 0.0;
double target_position_step = 0;
double target_rev_per_s = 1.0;
double motorPwmValue = 0;

#include <PID_v1.h>
//PID myPID(&current_rev_per_s, &motorPwmValue, &target_rev_per_s, 50, 1000, 5, DIRECT);
PID myPID(&current_position_step, &motorPwmValue, &target_position_step, 10, 20, 0.02, DIRECT);

#include "MotionProfile.h"

//Serial Reporting / Debug
//constexpr long serialReportInterval = 100; //millis
long reportTimeOffsetMicros = 0;

void setup() {
    TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to 8 for PWM frequency of 3921.16 Hz
    Serial.begin(115200);
    Serial.setTimeout(10);
    Serial.println("Motor RevPerSec Test:");
    Motor1.setSpeed(0);

    //Configure PID controller
    myPID.SetMode(0);
    myPID.SetOutputLimits(-255, 255);
    //myPID.SetSampleTime(20); //20 ms = 50Hz
    myPID.SetSampleTime(2); //20 ms = 50Hz

}

void compute_rev_per_s() {
    current_position_step = myEnc.read();
    static long oldPosition = 0;
    static unsigned long lastReportTime = 0;
    long deltaTime = millis() - lastReportTime;
    if (deltaTime > speedEvalInterval) {
        lastReportTime = millis();
        long deltaPosition = current_position_step - oldPosition;
        current_rev_per_s = (float)deltaPosition / stepPerRev / deltaTime * 1000;
        oldPosition = current_position_step;
        //Serial
        //Serial.print(deltaPosition);
        //Serial.print(',');
        //Serial.print(deltaTime);
        //Serial.print(',');
        //Serial.println(current_rev_per_s);
    }
}

void reporting(long ReportInterval) {
    static unsigned long lastReportTime = 0;
    long deltaTime = millis() - lastReportTime;
    if (deltaTime > ReportInterval) {
        lastReportTime = millis();
        Serial.print(micros()- reportTimeOffsetMicros);
        Serial.print(' ');
        Serial.print(target_position_step,0);
        Serial.print(' ');
        Serial.print(current_position_step,0);
        Serial.print(' ');
        Serial.print(motorPwmValue);
        Serial.print('\n');
    }

}

void loop() {
    //Listen for Serial command
    if (Serial.available()) {
        delay(1);
        double kp = Serial.parseFloat();
        double ki = Serial.parseFloat();
        double kd = Serial.parseFloat();
        double velocity = Serial.parseFloat();
        double steps = Serial.parseInt();
        //Serial.print(">>> Test Steps: ");
        //Serial.print(steps);
        //Serial.print(" Velocity (Step/Sec): ");
        //Serial.println(velocity);
        double continueReportAfterProfileMillis = 1000;
        //Reset Encoder
        myEnc.write(0);

        //Configure PID , reset
        myPID.SetTunings(kp, ki, kd);
        myPID.SetMode(1);

        //setup new motion profile
        LinearMotionProfile profile = LinearMotionProfile(0, steps, velocity);

        //Print Octave matrix format header to start:
        Serial.print("result_");
        Serial.print(kp, 0);
        Serial.print("_");
        Serial.print(ki, 0);
        Serial.print("_");
        Serial.print(kd, 0);
        Serial.print("_");
        Serial.print(velocity, 0);
        Serial.print("_");
        Serial.print(steps, 0);
        Serial.println(" = [");

        profile.start();
        reportTimeOffsetMicros = profile.getStartTimeMicros();
        while (profile.isRunning()) {
            //Read encoder and compute RPM
            compute_rev_per_s();

            //Read motion profile
            target_position_step = profile.getCurrentStep();

            //Compute PID
            myPID.Compute();

            //Set Motor PWM based on PID Output
            Motor1.setSpeed(motorPwmValue);

            //Report
            reporting(0);
        }

        //Serial.println(">>> Profile Ends");
        long end_time_micros = micros();

        // Continue to run PID until target position is reached
        //while (current_position_step != target_position_step) {
        while (micros() - end_time_micros < continueReportAfterProfileMillis * 1000){
            //Read encoder and compute RPM
            compute_rev_per_s();

            //Compute PID
            myPID.Compute();

            //Set Motor PWM based on PID Output
            Motor1.setSpeed(motorPwmValue);

            //Report
            reporting(0);

            //Break out after 1 sec
            //if (micros() - end_time_micros > continueReportAfterProfileMillis * 1000) break;
        }

        //Finalize the test (stop motor etc)

        Motor1.stop();
        myPID.SetMode(0);

        //Print Octave matrix format footer to start:
        Serial.print("]; % Motion Profile Ends at  ");
        Serial.print(end_time_micros - profile.getStartTimeMicros());
        Serial.println("microseconds.");
    }






}
