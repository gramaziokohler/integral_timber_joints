/* Encoder Library - SpeedTest - for measuring maximum Encoder speed
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *

This script is used to determine if the encoder is fast enough to read .
*/

// Motor Voltage vs Pin 12 Voltage

// Test Result for 555 motor Encoder using 2 inturupt pins myEnc(2,3)
// 0V : 2.22,V
// 12V : 2.13V

// Test Result for 555 motor Encoder using 1 inturupt pins myEnc(2,4)
// 0V : 0.798V
// 12V : 0.781V


 // This SpeedTest example provides a simple way to verify how much
 // CPU time Encoder is consuming.  Connect a DC voltmeter to the
 // output pin and measure the voltage while the encoder is stopped
 // or running at a very slow speed.  Even though the pin is rapidly
 // pulsing, a DC voltmeter will show the average voltage.  Due to
 // software timing, it will read a number much less than a steady
 // logic high, but this number will give you a baseline reading
 // for output with minimal interrupt overhead.  Then increase the
 // encoder speed.  The voltage will decrease as the processor spends
 // more time in Encoder's interrupt routines counting the pulses
 // and less time pulsing the output pin.  When the voltage is
 // close to zero and will not decrease any farther, you have reached
 // the absolute speed limit.  Or, if using a mechanical system where
 // you reach a speed limit imposed by your motors or other hardware,
 // the amount this voltage has decreased, compared to the baseline,
 // should give you a good approximation of the portion of available
 // CPU time Encoder is consuming at your maximum speed.

 // Encoder requires low latency interrupt response.  Available CPU
 // time does NOT necessarily prove or guarantee correct performance.
 // If another library, like NewSoftSerial, is disabling interrupts
 // for lengthy periods of time, Encoder can be prevented from
 // properly counting the intput signals while interrupt are disabled.


 // This optional setting causes Encoder to use more optimized code,
 // but the downside is a conflict if any other part of your sketch
 // or any other library you're using requires attachInterrupt().
 // It must be defined before Encoder.h is included.
 //#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Encoder.h>
#include "pins_arduino.h"

// Change these two numbers to the pins connected to your encoder
// or shift register circuit which emulates a quadrature encoder
//  case 1: both pins are interrupts
//  case 2: only first pin used as interrupt
Encoder myEnc(2,4);

// Connect a DC voltmeter to this pin.
const int outputPin = 12;

/* This simple circuit, using a Dual Flip-Flop chip, can emulate
   quadrature encoder signals.  The clock can come from a fancy
   function generator or a cheap 555 timer chip.  The clock
   frequency can be measured with another board running FreqCount
   http://www.pjrc.com/teensy/td_libs_FreqCount.html

                        +5V
                         |        Quadrature Encoder Signal Emulator
 Clock                   |
 Input o----*--------------------------      ---------------------------o Output1
            |            |14           |    |
            |     _______|_______      |    |     _______________
            |    |    CD4013     |     |    |    |    CD4013     |
            |  5 |               | 1   |    |  9 |               | 13
        ---------|  D         Q  |-----|----*----|  D         Q  |------o Output2
       |    |    |               |     |         |               |
       |    |  3 |               |     |      11 |               |
       |     ----|> Clk          |      ---------|> Clk          |
       |         |               |               |               |
       |       6 |               |             8 |               |
       |     ----|  S            |           ----|  S            |
       |    |    |               |          |    |               |
       |    |  4 |            _  | 2        | 10 |            _  | 12
       |    *----|  R         Q  |---       *----|  R         Q  |----
       |    |    |               |          |    |               |    |
       |    |    |_______________|          |    |_______________|    |
       |    |            |                  |                         |
       |    |            | 7                |                         |
       |    |            |                  |                         |
        --------------------------------------------------------------
            |            |                  |
            |            |                  |
          -----        -----              -----
           ---          ---                ---
            -            -                  -
*/


void setup() {
    pinMode(outputPin, OUTPUT);
}

#if defined(__AVR__) || defined(TEENSYDUINO)
#define REGTYPE unsigned char
#else
#define REGTYPE unsigned long
#endif

void loop() {
    volatile int count = 0;
    volatile REGTYPE *reg = portOutputRegister(digitalPinToPort(outputPin));
    REGTYPE mask = digitalPinToBitMask(outputPin);

    while (1) {
        myEnc.read();	// Read the encoder while interrupts are enabled.
        noInterrupts();
        *reg |= mask;	// Pulse the pin high, while interrupts are disabled.
        count = count + 1;
        *reg &= ~mask;
        interrupts();
    }
}

