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

class ReliableMessenger {
    public:
        ReliableMessenger(Stream &stream, void(*receiveCallback)(char*));
        void listen();
        boolean sendMessage();
        boolean sendMessageAsyc(void(*callback)(boolean));
        unsigned int discardUnreadMessages();
        void setRetry(int retrymax);
        void setSendTimeout(unsigned long millisec);
        void setEscapeSymbol(byte symbol);
        void setmessageEndSymbol(byte symbol);

        void setPin(int pin);
        void dot();
        void dash();

    private:
        Stream *_stream;
        char _receiveBuffer[BUFFER_LENGTH];
        unsigned int _receivedLength;
        byte _lastReceivedByte;
        int _retrymax;
        int _messageEndSymbol;
        int _escapeSymbol;
        int _sendTimeoutMillis;
        //variables for sending messages
        int _retrycount;
        void(*_receiveCallback)(char*);
        void(*_sendCallback)(boolean);

        int _pin;
};

struct Message {
    public:
        byte receiverAddress;
        byte senderAddress;
        char* body; // Pointer to a char[] that contains an message body only. 
};

// The transport class take care of transmitting a message from specific sender to specific receiver.
// It needs to reassemble the message from potentially intermitent transmission
class Transport {
    public:
        virtual void sendMessage(Message message) = 0;  // Send a message
        virtual Message receiveMessage() = 0;
        virtual boolean available() = 0;
        void setMessageEndSymbol(char symbol);
        void setAddress(byte selfAddress);
    protected:
        int _messageEndSymbol;
        byte _address;
};

//SerialTransport ignores the sender and receiver
class SerialTransport : public Transport {
    public:
        SerialTransport(Stream &stream, unsigned int bufferLength);
        void sendMessage(Message message);  // Send a message
        boolean available();
        Message receiveMessage();

    private:
        Stream *_stream;
        char* _receiveBuffer;
        //char _receiveBuffer[BUFFER_LENGTH];
        unsigned int _receiveIndex;
        Message _receivedMessage;
};
#endif

