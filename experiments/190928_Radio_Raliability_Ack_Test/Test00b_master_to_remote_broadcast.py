# Testing to see if I can do broadcast messengine (Not completed)


test_string_length = 20
test_ack_timeout = 40


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
transport = SerialRadioTransport(serial_port,chr(4),'a', 2, 19)

#Initialize ReliableMessenger
messenger = ReliableMessenger(transport)
messenger.set_retry(5)
messenger.set_ack_timeout(test_ack_timeout)

#Send message

resend_count = 0
success_count = 0
tries = 100
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

