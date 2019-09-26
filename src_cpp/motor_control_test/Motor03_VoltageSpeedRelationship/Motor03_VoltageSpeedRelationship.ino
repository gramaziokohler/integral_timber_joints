/*
 Name:		Motor03_VoltageSpeedRelationship.ino
 Created:	9/26/2019 10:50:37 PM
 Author:	leungp
*/

/*
This script is used to determine the speed characteristic of the motors.
*/

// Test Result for 775 (1:49) No Load Speed
// stepPerRev = (60*49);
// Voltage : Speed (RPS) : Current : Freq (deduced)
// 2V = 0.21 (0.16A)
// 3V = 0.32
// 4V = 0.44
// 5V = 0.55 (0.19A)
// 6V = 0.67
// 7V = 0.79
// 8V = 0.90
// 9V = 1.02
// 10V = 1.14 (0.27A)
// 11V = 1.26 (0.28A)
// 12V = 1.37 (0.30A) (4027Hz)
// 13V = 1.49 (0.31A) (4380Hz)

// Test for 555 is not yet performed.



constexpr int stepPerRev = (60*49);
constexpr long reportInterval = 200; //millis

#include "Encoder.h"

Encoder myEnc(2, 3);

void setup() {
    Serial.begin(115200);
    Serial.println("Motor RevPerSec Test:");
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
}
