/*
 Name:		Motor01_Encoder.ino
 Created:	9/26/2019 9:17:34 PM
 Author:	leungp
*/

// Test Result about 22428 steps for 10 turns on 555 motor 1:51 (should be 22440)
// 2V to 13 V all similar results.

// Test Result about 29528 steps for 10 turns on 775 motor 1:49 (should be 29400)
// 2V to 7 V all similar results.
// 13V have a different result = 25252 (Suspect the Serial writting is inturrupting too much)

// After adding an interval betweening serial write, the situation is better
// 13V have a different result = 29597
// So this is probably acceptable.

// Manually turned result can operate both pos and neg direction.


#include "Encoder.h"

Encoder myEnc(2, 3);

void setup() {
    Serial.begin(115200);
    Serial.println("Basic Encoder Test:");
}

long oldPosition = -999;
unsigned long nextReportTime = 0;

void loop() {
    long newPosition = myEnc.read();
    if (millis() > nextReportTime) {
        if (newPosition != oldPosition) {
            oldPosition = newPosition;
            Serial.println(newPosition);
        }
        nextReportTime = millis() + 1000; //Newly added report interval
    }
}
