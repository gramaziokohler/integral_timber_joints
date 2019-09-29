# This file tests the entire stack of Radio Messenging
# This file is run from a computer connected to a SerialRadio.
# A remote radio (of the same kind) is set up to listen at freq 2 (433Mhz), address 'b'
# The remote radio needs to respond with ACK message upon receiving a message

# Test Goal: See if any particular frequency have least amount of resend

# | Integer | Char | Frequency |
# | ------- | ---- | --------- |
# | 48      | 0    | CFREQ_868 |
# | 49      | 1    | CFREQ_915 |
# | 50      | 2    | CFREQ_433 |
# | 51      | 3    | CFREQ_918 |

# Testing parameters: 
#   Remote radio distance 2 meters with door in between. All stock antenna
# Frequency 1 (915Mhz / 327)
#   Very bad connection. Not sure what happens. 
#   Remote radio is placed on battery power and I walk around. 
#   Radio only works in close proximity < 2m and in direct sight.
#   SImilar to previous tests, very close proximity also works. As close as touching.
#   When changed to wire antenna on local radio, the range seems to improve by 1 meter. But still line of sight

# Frequency 0 (868Mhz / 345mm)
#   very similar to Freq 1, but slightly more range and can go through a wall. But range is not good after a few meters.
#   Changeing to long wire antenna doesn't help.

# Frequency 2 (433Mhz / 692mm)
#   The best frequency, the reach can go from my room to all of the house. 
#   One of the result: Success: 1000 / 1000 Resend: 2
# Second test to see how far I can go (Wire antenna):
#   Going 2 stories above to the roof was still a successful connection. 
#   Occational is resend is necessary but was generally good connection.
# Third test to see how far I can go (Stock antenna):
#   Going down to the street, I can still have reception all the way acrocc the street.
#   Connection stopped before I reach to kaiser franz

# Frequency 3 (918MHz / 326mm)
#   very similar to Freq 1.

# Conclusion:
#   Frequency 2 clearly out performs all other frequency.
#   I'm not sure why this happens, perhaps the lower frequency is much better at non-direct propagation
#   Or maybe the radio module and the stock antenna is designed to operate in this frequency region.
# 
 
test_frequency = 2
test_string_length = 40
test_ack_timeout = 100


from serial import Serial
import time
import helper_functions as fx

from serial_radio_transport_driver.Message import Message
from serial_radio_transport_driver.SerialTransport import SerialRadioTransport
from serial_radio_transport_driver.ReliableMessenger import ReliableMessenger

#Initialize Serial Port
serial_port = Serial('COM5', 115200, timeout=1)
time.sleep(2)

#Initialize SerialTransport
transport = SerialRadioTransport(serial_port,chr(4),'a', test_frequency, 19)

#Initialize ReliableMessenger
messenger = ReliableMessenger(transport)
messenger.set_retry(5)
messenger.set_ack_timeout(test_ack_timeout)

#Send message

resend_count = 0
success_count = 0
tries = 100000
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

