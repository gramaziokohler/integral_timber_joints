'''
    File Name: E01_Frequency_Test.py
    Purpose: To Test which frequency has better reception between the two radios.
    Setup: Uses Reliable Messenger Library in Python.



'''
from serial import Serial
import time

from serial_radio_transport_driver.ReliableMessenger import ReliableMessenger
from serial_radio_transport_driver.SerialTransport import SerialRadioTransport
from serial_radio_transport_driver.Message import Message

# Remote radio setting
remote_radio_address:str = 'b' #Probably not changed in code
remote_radio_channel:int = 19
remote_radio_freq:int = 1

#Select Serial Port
import serial.tools.list_ports
ports = list(serial.tools.list_ports.comports())
for p in ports:
    print (p)
if len(ports) == 0 :
    print ("No COM Ports available. Terminating")
    exit()
print("Connecting to " + str(ports[0][0]))

#Connect to Serial Port
serial_port = Serial(ports[0][0], 115200, timeout=1)
serial_port.reset_input_buffer()
time.sleep(3)

#Prepare SerialTransport
transport = SerialRadioTransport(serial_port)
transport.set_end_of_msg_char('\x04')

#Prepare Messenger
messenger = ReliableMessenger(transport)

#Function to change remote (and then local) radio frequency
def change_communication_freq(freq:int) -> None:
    assert freq >= 0
    assert freq <= 3

    #Remember original freq
    original_freq = freq

    #Construct Setting Message
    msg = Message()
    msg.receiver_address = remote_radio_address
    msg.sender_address = '\x07'
    msg.body = '1'              #'0'addr '1'freq '2'channel
    msg.body += chr(freq + 48)

    #Prepare for multiple retry just to be sure.
    for i in range(5):
        #Change remote setting
        transport.send_message(msg)
        time.sleep(1)

        # Change local radio
        change_local_freq(freq)

        # Test if the
    #Send a confirm message and wait for ACK to confirm success
    raise NotImplementedError


def change_local_freq(freq:int) -> None:
    assert freq >= 0
    assert freq <= 3
    msg = Message()
    msg.receiver_address = '\x07'
    msg.sender_address = '\x07'
    msg.body = '1'              #'0'addr '1'freq '2'channel
    msg.body += chr(freq + 48)
    for _ in range(5):
        transport.send_message(msg)
        time.sleep(0.2)


if __name__ == "__main__":
    #Test change local freq function

    #Test change remote change freq function

    #


    pass
