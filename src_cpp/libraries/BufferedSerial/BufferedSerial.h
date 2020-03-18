#pragma once

#ifndef _BufferedSerial_h
#define _BufferedSerial_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

const unsigned int MAX_INPUT = 50;

class BufferedSerial {


    public:
    BufferedSerial(int x) {

        buffer = x;
    }

    void serialInit() {
        //Setup Serial Port
        Serial.begin(115200);
        Serial.println(F("(Motor controller startup)"));
    }

    // here to process incoming serial data after a terminator received
    void process_data(const char * data) {
        // for now just display it
        // (but you could compare it to some value, convert to an integer, etc.)
        Serial.println(data);
    }  // end of process_data

    // Returns true if the byte is line terminator.
    boolean processIncomingByte(const byte inByte) {

        static unsigned int input_pos = 0;

        switch (inByte) {

        case '\n':   // end of text
            _rx_buffer[input_pos] = 0;  // terminating null byte

            // terminator reached! process input_line here ...
            //process_data(_rx_buffer);

            //Raise available flag
            _available = true;
            // reset buffer for next time
            input_pos = 0;
            return true;
            break;

        case '\r':   // discard carriage return
            break;

        default:
            // keep adding if not full ... allow for terminating null byte
            if (input_pos < (MAX_INPUT - 1))
                _rx_buffer[input_pos++] = inByte;
            break;

        }  // end of switch

        return false; // Return false when line term is not found.
    } // end of processIncomingByte

    // returns true if new bytes have been received.
    boolean run() {
        // Do not continue reading if previous flag is not cleared.
        if (_available) return false;

        // Quit if no serial is available.
        if (Serial.available() == 0) return false;

        // Process as much bytes as possible until line end.
        while (Serial.available() > 0)
            if (processIncomingByte(Serial.read())) {
                // If a line end is found, the run function will terminate.
                return true;
            }

        return true;
    }

    // Queries if line is available.
    boolean available() {
        run();
        return _available;
    }

    // Returns pointer to a constant char[] // Returns NULL if 
    const char * read() {
        if (_available) {
            _available = false;
            return _rx_buffer;
        } else {
            return NULL;
        }
    }

    private:
    int buffer;

    char _rx_buffer[MAX_INPUT];
    boolean _available = false;

};

#endif
#pragma once
