/*
 Name:		reliable_messenger.h
 Created:	9/15/2019 12:49:36 PM
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

#include "ccpacket.h"
#include "cc1101.h"

// Message struct to hold a incoming or outgoing message.
// Body must not contain '\004' symbol because it is reserved for the Transport layer.
struct Message {
    public:
    byte receiverAddress;
    byte senderAddress;
    char* body; // Pointer to a char[] that contains an message body only.
    int length() const;
};






// Transport class take care of transmitting a message from specific sender to specific receiver.
// It needs to reassemble the message from potentially intermitent transmission
class Transport {
    public:
    // Send a message to a specific sender
    virtual void sendMessage(Message const * message) = 0;  // Send a message
    virtual Message const * const receiveMessage() = 0;
    virtual boolean available() = 0;
    // Default self address is 1
    void setAddress(byte selfAddress);
    byte getAddress();
    protected:
    byte _address = 1; // Default self address is 1
};


//SerialTransport class is inhereted from Transport to use a Serial Stream
//Since Serial is point to point by nature. It will ignores the sender and receiver address
class SerialTransport : public Transport {
    public:
    SerialTransport(Stream &stream, unsigned int bufferLength);
    SerialTransport(Stream & stream, unsigned int bufferLength, byte address);
    void sendMessage(Message const * message);  // Send a message
    boolean available();
    Message const * const receiveMessage();
    char getEndOfMessageChar();
    void setEndOfMessageChar(char endOfMessageChar);

    protected:
    //End of message character. Default is \n
    char _endOfMessageChar = '\n';
    // Pointer to the Serial Stream object assigned at construction
    Stream *_stream;
    // Buffer (memory allocatied at constructor) to store incoming message.
    char* _receiveBuffer;
    // Number of received character received so far
    unsigned int _receiveIndex = 0;
    // ReceivedMessage (memory allocatied at constructor) to hold received message.
    Message _receivedMessage;
};

//SerialTransport class is inhereted from Transport to use a Serial Radio via a Serial Stream
//The Receiver and Sender Address is inserted in the header
class SerialRadioTransport : public SerialTransport {
    public:
    SerialRadioTransport(Stream &stream, unsigned int bufferLength);
    SerialRadioTransport(Stream & stream, unsigned int bufferLength, byte address);
    void sendMessage(Message const * message);  // Send a message
    boolean available();
    Message const * const receiveMessage();
    void setRadioAddress(byte address);
    void setRadioFrequency(byte address);
    void setRadioChannel(byte address);
    void setAddress(byte address);

    private:
    void setRadioSettings(byte settingAddress, byte value);
    boolean messageForMe = true; //Flag used by available() to keep track of unrelated emssages.
        //End of message character. Default is \n
    char _radioSettingChar = 7U;
};

//RadioTransport class is inhereted from Transport to use a Serial Stream
//Since Serial is point to point by nature. It will ignores the sender and receiver address
//class CC1101RadioTransport : public Transport {
//    public:
//    CC1101RadioTransport(byte address);
//    boolean begin();
//    void sendMessage(Message const * message);  // Send a message
//    boolean available();
//    Message const * const receiveMessage();
//
//    // Function for Interrupt call back when message is received
//    static void _messageReceived();
//
//    protected:
//
//    // Pointer to the Serial Stream object assigned at construction
//    CC1101 *_radio;
//    // Stores the state set by radio inturrupt. If true, a packet is ready to be recieved from radio.
//    static bool _packetWaiting;
//
//
//    uint8_t _interruptPin = 0;
//    uint8_t _gdo0Pin = 2;
//
//       
//    // Buffer (memory allocatied at constructor) to store incoming message.
//    char* _receiveBuffer;
//    // Number of received character received so far
//    unsigned int _receiveIndex = 0;
//    // ReceivedMessage (memory allocatied at constructor) to hold received message.
//    Message _receivedMessage;
//};
//


// Reliable Messenger class transmit messages (Message objects) via a Transport object.
// It performs validation of reception by waiting for a ACK message. Upon timeout, it will automatically resend the message.
// It also automatically respond to received messages with an ACK message.
class ReliableMessenger {
    public:
    ReliableMessenger(Transport &transport);
    boolean available();
    const Message * receiveMessage();

    int sendMessage(Message const * message);
    //boolean sendMessageAsyc(void(*callback)(boolean));
    unsigned int discardUnreadMessages();
    void setRetry(int retrymax);
    void setSendTimeout(unsigned long millisec);

    private:
    void sendACK(Message * message);
    void computeACKString(char * originalMessage, char * ACKString);
    byte stringChecksum(char * s);
    Transport *_transport;
    //settings
    int _retryMax = 3;
    int _sendSendTimeoutMillis = 300;
    //variables for sending messages
    void(*_receiveCallback)(char*);
    void(*_sendCallback)(boolean);
    //variables for receiving messages
    Message incomingMessage;
    boolean _newMessageFlag = false;
};

#endif

