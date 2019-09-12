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

class ReliableMessenger {
    public:
        ReliableMessenger(Stream &stream, void(*receiveCallback)(String));
        void listen();
        boolean sendMessage();
        boolean sendMessageAsyc(void(*callback)(String));
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
        String _receiveBuffer;
        byte _lastReceivedByte;
        int _retrymax;
        int _messageEndSymbol;
        int _escapeSymbol;
        int _sendTimeoutMillis;
        //variables for sending messages
        int _retrycount;
        void(*_receiveCallback)(String);
        void(*_sendCallback)(boolean);

        int _pin;
};


//class ReliableMessenger {
//    public:
//    Morse(int pin);
//    void dot();
//    void dash();
//    private:
//    int _pin;
//};

#endif

