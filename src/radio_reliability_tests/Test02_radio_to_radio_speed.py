# (Still in progress)
# Test test two radios connected to the computer
# This will test the transport layer's receive message capability

# Goal: Test if each channel work
# | Integer | Char | Frequency |
# | ------- | ---- | --------- |
# | 48      | 0    | CFREQ_868 |
# | 49      | 1    | CFREQ_915 |
# | 50      | 2    | CFREQ_433 |
# | 51      | 3    | CFREQ_918 |
# Not sure why only frequency 1 works. Others doesn't work 
# This doesnt happen in previous test where one radio is remote.

# Upon further test:
# Channel 2 also works when the transmitting radio has a wire antenna
#       and the receiving radio is held in fist. It is not realiable.

# Test parameter: 
#       Frequency 1, Channel 19, Stock antenna.
#       2 radios 1 meters apart connected to same USB hub.
# Result: 
# String Length =  1 Average time =  6.57ms
# String Length =  3 Average time =  7.43ms
# String Length =  5 Average time =  8.21ms
# String Length =  7 Average time =  9.18ms
# String Length =  9 Average time = 10.03ms
# String Length = 11 Average time = 10.90ms
# String Length = 13 Average time = 11.78ms
# String Length = 15 Average time = 12.70ms
# String Length = 17 Average time = 13.53ms
# String Length = 19 Average time = 14.34ms
# String Length = 21 Average time = 15.10ms
# String Length = 23 Average time = 16.10ms
# String Length = 25 Average time = 17.06ms
# String Length = 27 Average time = 17.82ms
# String Length = 29 Average time = 17.36ms
# String Length = 31 Average time = 18.17ms
# String Length = 33 Average time = 19.00ms
# String Length = 35 Average time = 19.81ms
# String Length = 37 Average time = 20.63ms
# String Length = 39 Average time = 21.54ms
# String Length = 41 Average time = 22.37ms
# String Length = 43 Average time = 23.32ms
# String Length = 45 Average time = 24.22ms
# String Length = 47 Average time = 24.90ms
# String Length = 49 Average time = 25.84ms
# String Length = 51 Average time = 26.73ms
# String Length = 53 Average time = 27.58ms
# String Length = 55 Average time = 28.53ms
# String Length = 57 Average time = 29.33ms
# String Length = 59 Average time = 30.13ms

# Conclusion: There is a weird bump between 27 and 29 characters.
# 29 is actually faster than 27. Not sure why.
# Regression of  1 to 27 gives y = 0.4341x + 6.1197
# Regression of 29 to 59 gives y = 0.4295x + 4.8195


test_string_length = 10

from serial import Serial
import time
from datetime import datetime, timedelta
import helper_functions as fx

from serial_radio_transport_driver.Message import Message
from serial_radio_transport_driver.SerialTransport import SerialRadioTransport
from serial_radio_transport_driver.ReliableMessenger import ReliableMessenger

#Initialize Serial Port
serial_port1 = Serial('COM7', 115200, timeout=1)
serial_port2 = Serial('COM6', 115200, timeout=1)
time.sleep(4)

#Initialize SerialTransport
transport1 = SerialRadioTransport(serial_port1,chr(4),'a', 1, 19)
transport2 = SerialRadioTransport(serial_port2,chr(4),'b', 1, 19)

#Initialize ReliableMessenger
messenger1 = ReliableMessenger(transport1,5,40)
messenger2 = ReliableMessenger(transport2,5,40)

#Send message

for test_string_length in range (1,60,2) :
        tries = 100
        i = 0
        total_time = timedelta()

        while i < tries:
                #Send Message on radio 1
                msg = Message('b','a', fx.randomString(test_string_length))
                print("Sending Message: " + msg.body,end='\r')
                start_time = datetime.utcnow()
                transport1.send_message(msg)

                #Listen for message on radio2
                received = False
                success = False
                while (received == False and datetime.utcnow() < start_time + timedelta(milliseconds=100)):
                        if (transport2.available()):
                                received = True
                                received_msg  = transport2.receive_message()
                                success =  (received_msg.body == msg.body)
                                #print("Received: " + received_msg.body)

                delta_time = datetime.utcnow() - start_time
                #Counting only if send is a success without retry
                if (success):
                        i += 1
                        total_time += delta_time
                        print (str(i) ,end='\r')
        print("# String Length = " + '%2i'%(test_string_length) + " Average time = " + '%5.2f' % (total_time.total_seconds() * 1000 / tries) + "ms")

