/*
 Name:		messenger_tester.ino
 Created:	9/12/2019 4:06:07 PM
 Author:	leungp
*/
#include <ReliableMessenger.h>

SerialTransport transport (Serial, 64);

void setup() {
    Serial.begin(115200);
    char messageBody[] = "Transport Ready\n";
    Message msp;
    msp.body = messageBody;
    transport.sendMessage(msp);
}

// the loop function runs over and over again until power down or reset
void loop() {
    if (transport.available()) {
        Message newMessage = transport.receiveMessage();
        Serial.print("MessageReceved:");
        Serial.println(newMessage.body);
        Serial.print("Length:");
        Serial.println(strlen(newMessage.body));
    }

}
