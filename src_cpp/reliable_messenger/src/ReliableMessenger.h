/*
 Name:		reliable_messenger.h
 Created:	9/11/2019 7:30:59 PM
 Author:	leungp
 Editor:	http://www.visualmicro.com
*/

#ifndef _reliable_messenger_h
#define _reliable_messenger_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define BUFFER_LENGTH 64



// Message struct to hold a incoming or outgoing message.
// Body must not contain '\0' symbol because it is reserved for the Transport layer .
struct Message {
    public:
        byte receiverAddress;
        byte senderAddress;
        char* body; // Pointer to a char[] that contains an message body only.
        int length();
};

// The transport class take care of transmitting a message from specific sender to specific receiver.
// It needs to reassemble the message from potentially intermitent transmission

class Transport {
    public:
        // Send a message to a specific sender
        virtual void sendMessage(Message * message) = 0;  // Send a message
        virtual Message * receiveMessage() = 0;
        virtual boolean available() = 0;
        void setAddress(byte selfAddress);
        byte getAddress();
    protected:
        byte _address;
};

//SerialTransport is implementation to use a Serial Stream
//Since Serial is point to point by nature. It will ignores the sender and receiver address
class SerialTransport : public Transport {
    public:
        SerialTransport(Stream &stream, unsigned int bufferLength);
        SerialTransport(Stream & stream, unsigned int bufferLength, byte address);
        void sendMessage(Message * message);  // Send a message
        boolean available();
        Message * receiveMessage();
        const char serialEndOfMessage = '\004';

    private:
        Stream *_stream;
        char* _receiveBuffer;
        unsigned int _receiveIndex;
        Message _receivedMessage;
};

class ReliableMessenger {
    public:
        ReliableMessenger(Transport &transport);
        boolean available();
        Message * receiveMessage();

        int sendMessage(Message * message);
        boolean sendMessageAsyc(void(*callback)(boolean));
        unsigned int discardUnreadMessages();
        void setRetry(int retrymax);
        void setSendTimeout(unsigned long millisec);

    private:
        void sendACK(Message * message);
        char * computeACKString(Message * message);
        byte stringChecksum(char * s);
        Transport *_transport;
        //settings
        int _retryMax = 3;
        int _sendSendTimeoutMillis = 300;
        //variables for sending messages
        void(*_receiveCallback)(char*);
        void(*_sendCallback)(boolean);
        //variables for receiving messages
        Message * incomingMessage;
        boolean _newMessageFlag = false;
};
#endif

