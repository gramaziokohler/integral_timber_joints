/*
 Name:		Motor05_PID_Velocity.ino
 Created:	10/6/2019 6:34:52 PM
 Author:	leungp

 This code allow setting the target velocity in rev/ms.
 A PID Controller will adjust the PWM output to maintain the set speed.
 Plots the output
*/

// There is no specific test in this script
// Manually try to come up with a set of PID values that work in maintaining speed.
// 50,600,0 was determined prevously as a stable set of values, and is used as initial values
// |aTuneStep | aTuneNoise | aTuneStartValue | aTuneLookBack | kp | ki | kd |
// |50      |0.4    |128    |10     |275.60 |10.33  |  0.0  | (Takes ~30 sec, doesn't seem to have adjusted output
// |50      |0.4    |128    | 5     |275.60 |45.61  |  0.0  | (Slightly faster, still no output adj
// (aTuneNoise Band reduced)
// |50      |0.1    |128    | 5     |128.82 | 8.59  |  0.0  | (Multiple adjustment of output observed, Noise values is important)
// |50      |0.2    |128    | 5     | 90.07 |10.29  |  0.0  | (Multiple adjustment of output observed, Different Noise Value, different result)
// |80      |0.2    |128    | 5     | 99.45 |13.26  |  0.0  | (Faster Switch betwen the two extreme, process is shorter)
// |30      |0.2    |128    | 5     |       |       |       | (Does not seems to have the output adjusted)
// |30      |0.1    |128    | 5     | 89.84 | 6.63  |  0.0  | (Noise Band reduced: behaviour normal again. 
// |30      |0.05   |128    | 5     |102.23 | 6.13  |  0.0  | (Noise Band reduced: behaviour normal again. 
//(Try to see what happens when direction reverse)
// |128     |0.2    |128    | 5     |       |       |       | (Not working.)
// |100     |0.2    |155    | 5     |108.00 | 5.08  |       | (Seems to be the biggest range this is tunnable)
// Adjusted the target_rev_per_s from 1.0 to 0.0
// |128     |0.2    |0      | 5     | 80.00 |  64.4 |       | (Motor goes both ways multiple times and results are returned quickly)
// |64      |0.2    |0      | 5     |102.96 |  4.58 |       | (Motor goes both ways multiple times and very different result)
// |192     |0.2    |0      | 5     | 82.39 | 13.64 |       | (Motor goes both ways multiple times and very different result)




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

PID myPID(&current_rev_per_s, &motorPwmValue, &target_rev_per_s, 50, 600, 0, DIRECT);

#include <PID_AutoTune_v0.h>
byte ATuneModeRemember = 2;
double kp = 2, ki = 0.5, kd = 2;
double kpmodel = 1.5, taup = 100, theta[50];
double outputStart = 5;
double aTuneStep = 100, aTuneNoise = 0.2, aTuneStartValue = 155;
unsigned int aTuneLookBack = 5;
boolean tuning = false;
unsigned long  modelTime, serialTime;
PID_ATune aTune(&current_rev_per_s, &motorPwmValue);
//set to false to connect to the real world
boolean useSimulation = false;



//Serial Reporting / Debug
constexpr long serialReportInterval = 100; //millis

void setup() {
    TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to 8 for PWM frequency of 3921.16 Hz
    Serial.begin(115200);
    Serial.setTimeout(10);
    Serial.println("Motor RevPerSec Test:");
    Motor1.setSpeed(0);

    //Configure PID controller
    //myPID.SetMode(AUTOMATIC);
    //myPID.SetOutputLimits(-255, 255);
    //myPID.SetSampleTime(20); //20 ms = 50Hz

    //Configure Autotune
    if (useSimulation) {
        for (byte i = 0; i < 50; i++) {
            theta[i] = outputStart;
        }
        modelTime = 0;
    }
    //Setup the pid 
    myPID.SetMode(AUTOMATIC);

    if (tuning) {
        tuning = false;
        changeAutoTune();
        tuning = true;
    }

    serialTime = 0;
   
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

    unsigned long now = millis();

    if (!useSimulation) { //pull the input in from the real world
        //Compute RPM (for PID Input)
        compute_rev_per_s();
    }

    if (tuning) {
        byte val = (aTune.Runtime());
        if (val != 0) {
            tuning = false;
        }
        if (!tuning) { //we're done, set the tuning parameters
            kp = aTune.GetKp();
            ki = aTune.GetKi();
            kd = aTune.GetKd();
            myPID.SetTunings(kp, ki, kd);
            AutoTuneHelper(false);
        }
    } else myPID.Compute();

    if (useSimulation) {
        theta[30] = motorPwmValue;
        if (now >= modelTime) {
            modelTime += 100;
            DoModel();
        }
    } else {
        //Set Motor PWM based on PID Output
        Motor1.setSpeed(motorPwmValue);
    }

    //send-receive with processing if it's time
    if (millis() > serialTime) {
        SerialReceive();
        SerialSend();
        serialTime += 500;
    }
}

void changeAutoTune() {
    if (!tuning) {
        //Set the output to the desired starting frequency.
        motorPwmValue = aTuneStartValue;
        aTune.SetNoiseBand(aTuneNoise);
        aTune.SetOutputStep(aTuneStep);
        aTune.SetLookbackSec((int)aTuneLookBack);
        AutoTuneHelper(true);
        tuning = true;
    } else { //cancel autotune
        aTune.Cancel();
        tuning = false;
        AutoTuneHelper(false);
    }
}

void AutoTuneHelper(boolean start) {
    if (start)
        ATuneModeRemember = myPID.GetMode();
    else
        myPID.SetMode(ATuneModeRemember);
}


void SerialSend() {
    Serial.print("setpoint: "); Serial.print(target_rev_per_s); Serial.print(" ");
    Serial.print("input: "); Serial.print(current_rev_per_s); Serial.print(" ");
    Serial.print("output: "); Serial.print(motorPwmValue); Serial.print(" ");
    if (tuning) {
        Serial.println("tuning mode");
    } else {
        Serial.print("kp: "); Serial.print(myPID.GetKp()); Serial.print(" ");
        Serial.print("ki: "); Serial.print(myPID.GetKi()); Serial.print(" ");
        Serial.print("kd: "); Serial.print(myPID.GetKd()); Serial.println();
    }
}

void SerialReceive() {
    if (Serial.available()) {
        char b = Serial.read();
        Serial.flush();
        if ((b == '1' && !tuning) || (b != '1' && tuning))changeAutoTune();
    }
}

void DoModel() {
    //cycle the dead time
    for (byte i = 0; i < 49; i++) {
        theta[i] = theta[i + 1];
    }
    //compute the input
    current_rev_per_s = (kpmodel / taup) *(theta[0] - outputStart) + current_rev_per_s * (1 - 1 / taup) + ((float)random(-10, 10)) / 100;

}
