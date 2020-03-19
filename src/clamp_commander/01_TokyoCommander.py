from serial import Serial
import time
import datetime
current_milli_time = lambda: int(round(time.time() * 1000))

from serial_radio_transport_driver.ReliableMessenger import ReliableMessenger
from serial_radio_transport_driver.SerialTransport import SerialRadioTransport
from serial_radio_transport_driver.Message import Message

# This script runs with the Serial2Radio_tokyo Radio implementation.
# Remember not to connect the LCD screen to the dongo, otherwise the delay is huge.
# The clamp controller: 07_TokyoController

# Select Serial Port
import serial.tools.list_ports
ports = list(serial.tools.list_ports.comports())
for p in ports:
    print (p)
if len(ports) == 0 :
    print ("No COM Ports available. Terminating")
    exit()
selected_port = ports[0][0]

# Override
selected_port = "COM12"

# Connect to Serial Port
print("Connecting to " + str(selected_port))
serial_port = Serial(selected_port, 115200, timeout=1)
serial_port.reset_input_buffer()
time.sleep(3)
print("Connected to " + str(selected_port))

# Prepare SerialTransport
transport = SerialRadioTransport(serial_port, address='0', eom_char='\n')



def try_receive_status_blocking (transport, address, time_out_millis = 100):
    transport.reset_input_buffer()
    transport.send_message(Message(address,'0', '_'))
    attempt_ends = current_milli_time() + 100 # 100 millis to wait for response
    while (current_milli_time() < attempt_ends):
        received_message = transport.receive_message()
        if received_message is not None:
            print ("Clamp%s : %s" % (received_message.sender_address, received_message.body))
            return True
    return False


command = input("Please enter a command:\n")
while (command != ""):
    # Prepare messages
    #targetPosition = command
    # msg1 = Message('1','0',"g%s"%targetPosition)
    # msg2 = Message('2','0',"g%s"%targetPosition)
    msg1 = Message('1','0', command)
    msg2 = Message('2','0', command)

    # Send messages
    transport.send_message(msg1)
    try_receive_status_blocking(transport, '1', 50)
    transport.send_message(msg2)

    print("Message sent")
    
    count = 0
    # Monitor
    for i in range(10):
        time.sleep(0.1)
        count += try_receive_status_blocking(transport, '1', 150)
        count += try_receive_status_blocking(transport, '2', 150)

    print("# of response received: %s" % count)
    command = input("Please enter a command:\n")


print("Serial Port Ends")

#Prepare Messenger
#messenger = ReliableMessenger(transport)
