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

# Test Goal: Figure out a functional ack_timeout
# Testing parameters: 
# Remote radio distance 2 meters
# String length = 40, ack_timeout = 50
#   Success: All OK
# String length = 40, ack_timeout = 40 <- Safe to use
#   Success: All OK
# String length = 40, ack_timeout = 30 <- Seems to be the limit. 
#   Success: 1000 / 1000 Resend: 1
# String length = 40, ack_timeout = 20
#   Success: 996 / 1000 Resend: 1858
#   Success: Not all the time
# String length = 40, ack_timeout = 15
#   Success: All success but only after 2 retries.
#   Some weird shit, why after 2 retry? It is consistent
# String length = 40, ack_timeout = 10
#   No success
#   Probably remote radio / controller did not respond fast enough.
# Conclusion: ack_timeout = 40 seems to be stable for long messages.

# Test Goal: 
#   Validate ack_timeout = 40 in other message length.
# Testing parameters: 
#   Remote radio distance 5 meters with door in between. All stock antenna
# String length = 10, ack_timeout = 40
#   Success: All OK
# String length = 50, ack_timeout = 40
#   Success: All OK
# String length = 59, ack_timeout = 40
#   Success: All OK
# String length = 60, ack_timeout = 40
#   Success: All fail (Not sure what is the bottle neck for 60chars)
# Conclusion: ack_timeout = 40 seems to be stable for up to 59 char messages


# Test Goal: 
#   Check if small obstacle is a problem to the radio
#   All stock antenna
# Testing parameters: 
#   String length = 20, ack_timeout = 40
# Remote radio distance 5 meters with door in between.
#   Success: 1000 / 1000 Resend: 1
# Remote radio distance 5 meters with 2 doors in between. Local radio in a pot (no lid).
#   Success: 1000 / 1000 Resend: 1
# Remote radio distance 5 meters with 2 doors in between. Local radio antenna held in fist
#   Success: 1000 / 1000 Resend: 7
# Remote radio distance 5 meters with 2 doors in between. Local radio no antenna
#   Success: 0 / 100
# Remote radio distance 5 meters with 2 doors in between. Local radio wire antenna vertical
#   Success: 1000 / 1000 Resend: 2
# Remote radio distance 5 meters with 2 doors in between. Local radio wire antenna horizontal
#   Success: 1000 / 1000 Resend: 1



test_string_length = 20
test_ack_timeout = 40


from serial import Serial
import time
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
messenger = ReliableMessenger(transport)
messenger.set_retry(5)
messenger.set_ack_timeout(test_ack_timeout)

#Send message

resend_count = 0
success_count = 0
tries = 1000
for i in range(tries):
    #Send Message
    msg = Message('b','a', fx.randomString(test_string_length))
    result = messenger.send_message(msg)
    #Counting if send is a success
    if (result >= 0):
        resend_count += result
        success_count +=1
    #print (i,end='\r')
    print ("Loop "+ str(i) +", Sending message : "+ str (msg.body) +" retrys: " + str(result) + "      ",end='\r')
    # #Receive Reply
    # correct_reply = False
    # time_out = datetime.utcnow() + timedelta(seconds=1)

    # while (datetime.utcnow() < time_out):
    #     if (correct_reply): break
    #     if (messenger.available()):
    #         reply = messenger.receive_message()
    #         print ("Reply: " + str(reply.body))
    #         if (reply.body == msg.body[::-1]):
    #             success_count +=1
    #             correct_reply = True
print("")
print("Success: " + str(success_count) + " / "+ str(tries) + " Resend: " + str(resend_count))

