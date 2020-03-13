/*
Sketch to test the usage of MotorController class.
Test was performed on a free spinning motor.
*/

#include <DCMotor.h>        // ITJ library
#include <Encoder.h>        // Encoder library from PJRC.COM, LLC - Paul Stoffregen http://www.pjrc.com/teensy/td_libs_Encoder.html
#include <MotorController.h>    //New Class in Development


//Pins for Motor Driver M1
const uint8_t m1_driver_ena_pin = 9;             // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t m1_driver_in1_pin = 8;             // the pin the motor driver IN1 is attached to
const uint8_t m1_driver_in2_pin = 7;             // the pin the motor driver IN2 is attached to

const double m1_kp = 0.005;                 // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_kp = 0.040
const double m1_ki = 0.200;                 // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_ki = 0.200
const double m1_kd = 0.0002;                // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_kd = 0.0002
const double m1_accel = 3000;               // Tuning based on result from Motor08_PID_TrapezoidalMotionProfile m1_accel = 3000

//Pins for Motor feedback M1 / M2
const uint8_t m1_encoder1_pin = 2;             // Motor encoder channel C1 (typically the Interrupt pin)
const uint8_t m1_encoder2_pin = 3;             // Motor encoder channel C2 

//Pins for Homing Switch
const uint8_t m1_home_pin = A6;

//Pins for Battery Monitor
const uint8_t battery_monitor_pin = A7;

// Initialize motion control objects
DCMotor Motor1(m1_driver_ena_pin, m1_driver_in1_pin, m1_driver_in2_pin);
Encoder Encoder1(m1_encoder1_pin, m1_encoder2_pin);
MotorController MotorController1(&Motor1, &Encoder1, m1_kp, m1_ki, m1_kd, m1_accel, 10, false, false);


// the setup function runs once when you press reset or power the board
void setup() {
    //Setup Serial Port
    Serial.begin(115200);
    Serial.println(F("(_04_MotorController_UnitTest)"));

    //Initialize battery monitor pin
    pinMode(battery_monitor_pin, INPUT);

}

void loop() {

    MotorController1.moveToPosition(5000, 1000);
    block_run_motor_with_report();
    Serial.println(F("(Target 1 Reached)"));
    delay(500);

    MotorController1.moveToPosition(0, 3000);
    block_run_motor_with_report();
    Serial.println(F("(Target 2 Reached)"));

    delay(500);

}

void block_run_motor_with_report() {
    while (!MotorController1.isTargetReached()) {
        if (MotorController1.run()) {
            Serial.print(F("CurPos: "));
            Serial.print(MotorController1.currentPosition());
            Serial.print(F(" Error: "));
            Serial.print(MotorController1.currentTarget() - MotorController1.currentPosition());
            Serial.print(F(" PWM: "));
            Serial.println(MotorController1.currentMotorPowerPercentage());
        }
        if (!MotorController1.isMotorRunning() && !MotorController1.isTargetReached()) {
            Serial.println(F("(Jammed :( )"));
            delay(500);
            break;
        }
    }
}
