#pragma once

/*
This class help manage the use of a resistor ladder attached DIP switch as an address (ID) input
This class help read the address (unsigned int) set by the DIP Switch by matching its analogread() against a list of known values (int)
The matches need not be exact, the closest value is choosen. If two matches are equally close, the smaller index is returned.

The index (zero counting) of the value is returned.

The list of known values do not have to be sorted.
Many implementation of resistor networks are possible.

This class is inspired by a post from Chris in http://www.ignorantofthings.com/2018/07/the-perfect-multi-button-input-resistor.html?m=1

*/
#ifndef _ResistorLadderID_h
#define _ResistorLadderID_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif


class ResistorLadderID {


    public:
    ResistorLadderID(const uint8_t analogPinNumber, int const * const valueArray, const unsigned int arrayLength):
        _arrayLength(arrayLength),
        _valueArray(valueArray),
        _analogPinNumber(analogPinNumber)
        // Remember to call init() after constructor.
    {

    }

    void init() {
        //Setup Serial Port
        pinMode(_analogPinNumber, INPUT); //High Impedeance input used to avoid drawing current.
        //Note that the ladder implementation must self pull towards a voltage reference.
    }

    int readRawValue() {
        return analogRead(_analogPinNumber);
    }

    unsigned int read() {
        unsigned int bestGuess = 0;


        // Compute first error term
        int rawValue = readRawValue();
        auto error = rawValue - _valueArray[0];
        auto leastError = abs(error);

        // Compute and compare the other errors
        for (unsigned int i = 1; i < _arrayLength; i++) {
            error = rawValue - _valueArray[i];
            if (abs(error) < leastError) {
                bestGuess = i;
                leastError = abs(error);
            }
        }

        return bestGuess;
    }


    private:
    const unsigned int _arrayLength;
    int const * const _valueArray;
    const uint8_t _analogPinNumber;

};

#endif
#pragma once
