test_string_length = 10

from serial import Serial
import time
from datetime import datetime, timedelta
import helper_functions as fx

from serial_radio_transport_driver.Message import Message
from serial_radio_transport_driver.SerialTransport import SerialRadioTransport
from serial_radio_transport_driver.ReliableMessenger import ReliableMessenger

#Initialize Serial Port
serial_port1 = Serial('COM3', 115200, timeout=1)
serial_port2 = Serial('COM5', 115200, timeout=1)
time.sleep(2)

#Initialize SerialTransport
transport1 = SerialRadioTransport(serial_port1,chr(4),'a', 2, 19)
transport2 = SerialRadioTransport(serial_port2,chr(4),'b', 2, 19)

#Initialize ReliableMessenger
messenger1 = ReliableMessenger(transport1,5,40)
messenger2 = ReliableMessenger(transport2,5,40)

#Send message

# for test_string_length in range (1,60,2) :
tries = 10
i = 0
total_time = timedelta()

while i < tries:
        #Send Message on radio 1
        msg = Message('b','a', fx.randomString(test_string_length))
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
                        print("Received")

        delta_time = datetime.utcnow() - start_time
        #Counting only if send is a success without retry
        if (success):
                i += 1
                total_time += delta_time
                print (str(i) ,end='\r')
print("# String Length = " + '%2i'%(test_string_length) + " Average time = " + '%.2f' % (total_time.total_seconds() * 1000 / tries) + "ms")

