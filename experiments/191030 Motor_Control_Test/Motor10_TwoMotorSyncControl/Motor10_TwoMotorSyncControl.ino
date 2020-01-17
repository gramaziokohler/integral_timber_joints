
#include "DCMotor.h"

const uint8_t m1_driver_ena_pin = 9;             // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t m1_driver_in1_pin = 8;             // the pin the motor driver IN1 is attached to
const uint8_t m1_driver_in2_pin = 7;             // the pin the motor driver IN2 is attached to
const uint8_t m2_driver_ena_pin = 10;             // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t m2_driver_in1_pin = 11;             // the pin the motor driver IN1 is attached to
const uint8_t m2_driver_in2_pin = 12;             // the pin the motor driver IN2 is attached to
DCMotor Motor1(m1_driver_ena_pin, m1_driver_in1_pin, m1_driver_in2_pin);
DCMotor Motor2(m2_driver_ena_pin, m2_driver_in1_pin, m2_driver_in2_pin);

#include "Encoder.h"
Encoder encoder1(3, A2);
Encoder encoder2(2, A1);

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
    encoder1.write(0);
    encoder2.write(0);

    // Setup new motion profile
    //LinearMotionProfile profile = LinearMotionProfile(0, total_steps, velocityStepsPerSec);
    TrapezoidalMotionProfile profile = TrapezoidalMotionProfile(0, total_steps, velocityStepsPerSec, accelStepsPerSecSq);

    //Serial.println("_phase1End_Micros=" + String(profile._phase1End_Micros));
    //Serial.println("_phase2End_Micros=" + String(profile._phase2End_Micros));
    //Serial.println("_phase3End_Micros=" + String(profile._phase3End_Micros));

    // Setup PID Positional Control Variables
    double current_position_step_1 = 0;
    double current_position_step_2 = 0;
    double target_position_step = 0;
    double m1_SpeedPercentage = 0.0;
    double m2_SpeedPercentage = 0.0;

    // Create PID, Configure PID controller
    PID myPID1(&current_position_step_1, &m1_SpeedPercentage, &target_position_step, kp, ki, kd, DIRECT);
    myPID1.SetOutputLimits(-1.0, 1.0);
    myPID1.SetSampleTime(2);
    myPID1.SetMode(1); //Turn on PID

    PID myPID2(&current_position_step_2, &m2_SpeedPercentage, &target_position_step, kp, ki, kd, DIRECT);
    myPID2.SetOutputLimits(-1.0, 1.0);
    myPID2.SetSampleTime(2);
    myPID2.SetMode(1); //Turn on PID

    // Compute Duration
    long total_duration_micros = preDurationSec * 1e6 + profile.getTotalDurationMicros() + postDuration_Sec * 1e6;
    unsigned long testStartTimeMicros = micros();

    while (micros() - testStartTimeMicros < total_duration_micros) {
        if ((!profile.isStarted()) && ((micros() - testStartTimeMicros) > (preDurationSec * 1e6))) {
            // Start Motion Profile
            profile.start();
        }
        //Read encoder
        current_position_step_1 = encoder1.read();
        current_position_step_2 = encoder2.read();

        //Read motion profile
        target_position_step = profile.getCurrentStep();

        //Compute PID
        myPID1.Compute();
        myPID2.Compute();

        //Set Motor PWM based on PID Output
        Motor1.setSpeedPercent(m1_SpeedPercentage);
        Motor2.setSpeedPercent(m2_SpeedPercentage);

        //Report
        reporting(5, (long)micros() - testStartTimeMicros - (preDurationSec * 1e6), target_position_step, current_position_step_1, current_position_step_2, m1_SpeedPercentage, m2_SpeedPercentage);


    }

    //Finalize the test (stop motor if PID didn't stop it perfectly)
    Motor1.stop();
    myPID1.SetMode(0);
    Motor2.stop();
    myPID2.SetMode(0);

    //Print Octave matrix format footer to start:
    Serial.println("]; % " + testResultName);
}

void reporting(long ReportIntervalMillis, long time, double target_position_step, double current_position_step_1, double current_position_step_2, double m1_SpeedPercentage, double m2_SpeedPercentage) {
    static unsigned long lastReportTime = 0;
    long deltaTime = millis() - lastReportTime;
    if (deltaTime > ReportIntervalMillis) {
        lastReportTime = millis();
        Serial.print(time);
        Serial.print(' ');
        Serial.print(target_position_step, 1);
        Serial.print(' ');
        Serial.print(current_position_step_1, 0);
        Serial.print(' ');
        Serial.print(current_position_step_2, 0);
        Serial.print(' ');
        Serial.print(m1_SpeedPercentage);
        Serial.print(' ');
        Serial.print(m2_SpeedPercentage);
        Serial.print('\n');
    }
}

void setup() {
    TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to 8 for PWM frequency of 3921.16 Hz
    Serial.begin(115200);
    Serial.setTimeout(10);
    Motor1.setSpeedPercent(0.0);
    Motor2.setSpeedPercent(0.0);

    //double kp
    //double ki
    //double kd
    //double velocityStepsPerSec
    //double accelStepsPerSecSq
    //double preDurationSec
    //double runDurationSec
    //double postDuration_Sec
    perform_one_test(0.040, 0.200, 0.0002, 4000, 3000, 0.2, 4.0, 1.0);
    perform_one_test(0.040, 0.200, 0.0002, 3000, 3000, 0.2, 4.0, 1.0);
    perform_one_test(0.040, 0.200, 0.0002, 2000, 3000, 0.2, 4.0, 1.0);
    perform_one_test(0.040, 0.200, 0.0002, 1000, 3000, 0.2, 4.0, 1.0);

}

void loop() {

}
