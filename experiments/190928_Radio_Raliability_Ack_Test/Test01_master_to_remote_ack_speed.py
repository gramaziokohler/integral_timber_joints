# This file tests the entire stack of Radio Messenging
# This file is run from a computer connected to a SerialRadio.
# A remote radio (of the same kind) is set up to listen at freq 2 (433Mhz), address 'b'
# The remote radio needs to respond with ACK message upon receiving a message

# Remote radio distance 1 meters:
#   Success not all the time
#   Seems like radio signal wont work if too close.

# Test Goal: This simple ACK test test the entire stack of communication layers from 
#   python on host computer -> USB -> local arduino -> radio sending -> radio receiveing ->
#   -> remote arduino -> remote Arduino dummy controller -> (and the entire reverse trip) 
#   This validates the entire stack is functional.

# Test Goal: Figure the time it takes from [before sending a string] and [received the ACK]
# Testing parameters: 
#   Remote radio distance 2 meters separated by two doors
#   ack_timeout = 40
#   trying 100 times and average out the result
#   only counting the result if there is no retrys involved.
# Result:
# String Length =  1 Average time = 12.38ms
# String Length =  3 Average time = 13.28ms
# String Length =  5 Average time = 14.06ms
# String Length =  7 Average time = 14.90ms
# String Length =  9 Average time = 15.67ms
# String Length = 11 Average time = 16.40ms
# String Length = 13 Average time = 17.15ms
# String Length = 15 Average time = 18.05ms
# String Length = 17 Average time = 18.82ms
# String Length = 19 Average time = 19.52ms
# String Length = 21 Average time = 20.28ms
# String Length = 23 Average time = 21.05ms
# String Length = 25 Average time = 21.88ms
# String Length = 27 Average time = 22.72ms
# String Length = 29 Average time = 23.37ms
# String Length = 31 Average time = 24.21ms
# String Length = 33 Average time = 24.92ms
# String Length = 35 Average time = 25.73ms
# String Length = 37 Average time = 26.63ms
# String Length = 39 Average time = 27.42ms
# String Length = 41 Average time = 28.12ms
# String Length = 43 Average time = 28.88ms
# String Length = 45 Average time = 29.70ms
# String Length = 47 Average time = 30.46ms
# String Length = 49 Average time = 31.28ms
# String Length = 51 Average time = 31.96ms
# String Length = 53 Average time = 32.79ms
# String Length = 55 Average time = 33.65ms
# String Length = 57 Average time = 34.41ms
# String Length = 59 Average time = 35.09ms
# A linear line fits very well: y = 0.3906x + 12.108
# Conclusion: Total time is rather linear relationship with the message size

test_ack_timeout = 40

from serial import Serial
import time
from datetime import datetime, timedelta
import helper_functions as fx

from serial_radio_transport_driver.Message import Message
from serial_radio_transport_driver.SerialTransport import SerialRadioTransport
from serial_radio_transport_driver.ReliableMessenger import ReliableMessenger

#Initialize Serial Port
serial_port = Serial('COM3', 115200, timeout=1)
time.sleep(2)

#Initialize SerialTransport
transport = SerialRadioTransport(serial_port,chr(4),'a', 2, 19)

#Initialize ReliableMessenger
messenger = ReliableMessenger(transport,5,test_ack_timeout)

#Send message

for test_string_length in range (1,60,2) :
    tries = 100
    i = 0
    total_time = timedelta()

    while i < tries:
        #Send Message
        msg = Message('b','a', fx.randomString(test_string_length))
        start_time = datetime.utcnow()
        result = messenger.send_message(msg)
        delta_time = datetime.utcnow() - start_time
        #Counting only if send is a success without retry
        if (0 == result):
            i += 1
            total_time += delta_time
            #print (str(i) ,end='\r')
    print("# String Length = " + '%2i'%(test_string_length) + " Average time = " + '%.2f' % (total_time.total_seconds() * 1000 / tries) + "ms")

