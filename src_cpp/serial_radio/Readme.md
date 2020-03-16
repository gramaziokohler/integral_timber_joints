# CC1101 Radio - Serial Bridge

This Serial Bridge code abstracts the [CC1101 Radio](http://www.ti.com/product/CC1101) (operates in [ISM Band](https://en.wikipedia.org/wiki/ISM_band) ) from a serial-capable device (such as a computer, another Arduino board or RPi board) that uses the radio functions. 

It presents the radio functions to send and receive messages via a Serial Port interface.

### Main Features

The Serial2Radio code runs on **Arduino Uno or Nano** attached to CC1101 Radio

The message sending device can connect to the Serial Port via pin 0 and 1 or the USB interface.

**Multiple radios** can be used for point-to-point wireless communication using an addressing scheme. Broadcasting is also possible by using a special broadcast address.

Networking protocol is not implemented and is up to the upper layer to slice long messages into packages, flow control, how they are routed and how a network can be formed. 

### Other features

If a I2C LCD screen is attached, it will display on (top row) Rssi and Lqi, (bottom row) last sent or received message. 

Two test button (Pushbutton connected to Ground) can be attached to A0 and A1 for testing purpose

### Limitations

Maximum message length is **61 bytes** where the first byte is reserved for receiver address. This limitation can be changed by editing `ccpacket.h`

## Compile and upload

The Serial2Radio.ino is the only Arduino File that contains code logic and is compiled and uploaded to the Arduino Uno or Nano. Compilation can be performed by [Arduino IDE](https://www.arduino.cc/en/main/software) or Visual Studio with [Visual Micro](https://www.visualmicro.com/).

It has a number of dependency (which are distributed together with this repo)

1. https://github.com/veonik/arduino-cc1101
2. https://github.com/mathertel/LiquidCrystal_PCF8574 (For debug LCD Screen )

### Compile options

**#define IncludeRadioQualityInSerialComment**

If defined, the received radio message will be followed by a Serial message that contains the radio signal **Received Signal Strength Indicator** (first value) and **Link Quality Indicator** (second value).

It will be in a bracketed message for example: `(22,-50)`

**#define SerialComment**

If defined, Radio will print debug message to Serial for debug purpose.

## Electrical Connections / Wiring

###  CC1101 Radio Module:

 CC1101 Pins => Arduino Pins
 5V+ => 5V+
 GND => GND
 CSN (SS) => 10
 MOSI => 11
 MISO => 12
 SCK => 13
 GD0 => 2

(The pin assignment is Native Arduino Uno/Nano [SPI interface](https://www.arduino.cc/en/reference/SPI), apart of the Slave Select (SS) pin, others should not be changed)

###  1602 LCD Screen Expanded to I2C via PCF8574 IO Expander 

 The LCD screen connection is mainly for debug purpose and it is recommended to disconnect the screen during high speed debugging or during normal operation. The radio will detect the presence of the screen during power on.

Radio Pin => Arduino Pins
 Vcc => 5V+
 GND => GND
 SDA => A4 (SDA)
 SCL => A5 (SCL)

(The pin assignment is Native Arduino Uno/Nano I2C port, this should not be changed)

## Typical Use Case

Computer to Arduino Communication can be achieved by two radio bridges. One attached to the computer via USB cable. Another bridge connected to another Arduino via Tx Rx (Pin 0,1) cross wire connection.

PC Side: (PC) <> (USB) <> (CC1101 Radio - Serial Bridge)
Arduino Side: (Arduino) <> (Tx Rx Gnd) <> (CC1101 Radio - Serial Bridge) 

When the radio is powered on. It is set into default Radio Configuration. These configuration can be changed with serial commands.

| Radio Address         | Channel Number           | Frequency        |
| --------------------- | ------------------------ | ---------------- |
| (int) 89 / (char) 'Y' | (int) 19 / (char) '\x13' | (int) 1 = 915MHz |

The serial communication can be established via USB port of the Arduino or the Tx Rx pins. For testing purpose, it is possible to use a Serial terminal to communicate with the radio. Remember to turn off automatic line feed.

| Baud Rate | Setting                            |
| --------- | ---------------------------------- |
| 115200    | SERIAL_8N1 (*the Arduino default*) |



### Message Structure

The radio accepts a standard message structure (setting change commands are also a message) that contains 4 parts.

| Receiver Address Byte | Sender Address Byte   | Message Body Byte(s)                                         | End of Message Byte     |
| --------------------- | --------------------- | ------------------------------------------------------------ | ----------------------- |
| (int) 16 to (int) 127 | (int) 16 to (int) 127 | Any number of bytes (but should not contain (int) 4 / (char) '\x04' | (int) 4 / (char) '\x04' |

Note that message body should not contain the designated end of message char: **'\x04'**. This char is chosen instead of '\n' or '\r' such that messages can contain multiple lines.

It is not tested whether char '\0' will work or not. As this radio is intended for printable character transmission.

### Sending Message 

~~For example sending (from Serial) to radio \x30\x31\x50\x60\x04~~ 

~~Send the target address as first byte to radio, for example  `A` followed by the message to send, followed by ␊(LineFeed).~~

~~**Example Code :** (in Ascii Text) `ABCD␊` will send the message `ABCD␊`  to the radio whose address is `A`. Note that radio A will receive the whole message `ABCD␊` and appear on the Serial port.~~

~~**Sender address** is optional in the message but can typically be the second byte in the message (e.g. `AB0123␊` can be the message from `radio B` to `radio A` that contain the message 0123)~~

For example receiver address is char '1'

From Visual Studio Interface:

| Receiver Address Byte                  | Sender Address Byte | Message Body 2 Bytes | End of Message Byte          |
| -------------------------------------- | ------------------- | -------------------- | ---------------------------- |
| **(int) 49<br />(char) '\x07' or '1'** | your own address    | Message Body         | (int) 4 /<br />(char) '\x04' |



Example from another Arduino

Example from a Python Script

### Receiving Message

As long as the **Radio Channel** and **Frequency** are the same between transmitting and receiving radio. All incoming messages which has a *Receiver Address* matching the Radio Address of the connected Radio, it will receive the message and pass it to the Serial Port.

### Bus Contention / Flow Control

There is *no flow control* involved in this radio. If two devices are transmitting at the same time, the radio message is likely to be garbage.

This radio is also not capable of receiving messages while it is transferring. (Because the radio can only talk  to or listen from one channel at a time.) If you need bi-directional, two radios on different frequency/channel will be a solution.

## Changing Radio Settings

Radio settings can be changed in two ways: by local Serial message, or by remote radio message (sent from another radio). Both have similar structure. 

by local Serial message:

| Receiver Address Byte          | Sender Address Byte        | Message Body 2 Bytes                         | End of Message Byte          |
| ------------------------------ | -------------------------- | -------------------------------------------- | ---------------------------- |
| **(int) 7<br />(char) '\x07'** | (int) 7<br />(char) '\x07' | Setting Number ID + Setting Value (Except 4) | (int) 4 /<br />(char) '\x04' |

by remote Radio message (sent from another radio):

| Receiver Address Byte          | Sender Address Byte        | Message Body 2 Bytes                         | End of Message Byte          |
| ------------------------------ | -------------------------- | -------------------------------------------- | ---------------------------- |
| **address of receiving radio** | (int) 7<br />(char) '\x07' | Setting Number ID + Setting Value (Except 4) | (int) 4 /<br />(char) '\x04' |

### Example Code

The following code are equivalent. They set Radio Address to (int) 89 (char) Y:

**Serial command example code (from another Arduino connected via Tx Rx):** 

```c++
Serial.write(7); // Receiver Address
Serial.write(7); // Sender Address
Serial.write(48); //Setting ID '0'
Serial.write(89); //Setting Value 90
Serial.write(4); //End of message
```

**Serial command code (from computer connected via USB):** 

```python
from serial import Serial
serial_port = Serial('COM7', 115200, timeout=1)
serial_port.reset_input_buffer()
time.sleep(3) # Wait for Arduino to power up and go through setup()
serial_port.write([7,7,48,89,4])
```

**Serial command (from Visual Micro Serial Terminal via USB):** 

```c#
\a\a0Y\x04
```



### Setting '0' - To Change the address of the radio

Setting Number ID = (int) 48 / (char) '0'

Setting Value = Radio Address between (int) 16 and (int) 127 (Default is (default) 97)

*Note: Do not use address (int) 4 because the system will consider your message to have ended.*
*Note: Do not use address (int) 0 because it does work as broadcst as you would expect

### To Change the frequency of the radio

Setting Number ID = (int) 49 / (char) '1'

Setting Value = the integer that represent the frequency to set. (Default Frequency is CFREQ_915)

| Integer | Char | Frequency |
| ------- | ---- | --------- |
| 48      | 0    | CFREQ_868 |
| 49      | 1    | CFREQ_915 |
| 50      | 2    | CFREQ_433 |
| 51      | 3    | CFREQ_918 |

For example in visual studio: '\x0007\x000700\x04' will change the address to char '0'

### To Change the channel of the radio

Setting Number ID = (int) 50 / (char) '2'

Setting Value = Radio Channel between (int) 0 and (int) 127 (See note) (Default is 19)

*Note: I'm not sure what are the valid channels but I think range between 0 to 120 works*

*Note: Do not use address (int) 4 because the system will consider your message to have ended.*



## License

This project uses code adapted from [veonik/arduino-cc1101](https://github.com/veonik/arduino-cc1101) library example.ino

veonik's library is a fork from [panStamp/arduino_avr](https://github.com/panStamp/arduino_avr)

This project also uses the [mathertel/LiquidCrystal_PCF8574](https://github.com/mathertel/LiquidCrystal_PCF8574)

This project adopt a GNU Lesser General Public License v3.0 license.
