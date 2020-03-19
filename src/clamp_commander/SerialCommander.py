from serial import Serial
import time
import datetime
import serial.tools.list_ports

from serial_radio_transport_driver.ReliableMessenger import ReliableMessenger
from serial_radio_transport_driver.SerialTransport import SerialRadioTransport
from serial_radio_transport_driver.Message import Message

from ClampModel import ClampModel

current_milli_time = lambda: int(round(time.time() * 1000))

class SerialCommander(object):
    
    def __init__(self):
        self.clamps = {}
        pass

    def start(self):
        # Check connected COM ports
        ports = list(serial.tools.list_ports.comports())
        if len(ports) == 0 :
            print ("No COM Ports available. Terminating")
            exit()
        
        # If there is more than one list available, list the Serial Ports to user
        if len(ports) > 1 :
            print ("The following COM Ports are connected. Please select the USB Radio Dongle:")
            for i, p in enumerate(ports):
                print ("<%s> %s" % (i,p))
            selected_index = int(input("- Which port <?>")) 
            selected_port = ports[selected_index][0]

        # If there is only one COM Port select that
        else:
            selected_port = ports[0][0]

        # Connect to Serial Port
        print("Connecting to %s ..." % (selected_port))
        serial_port = Serial(selected_port, 115200, timeout=1)
        serial_port.reset_input_buffer()
        # Need to wait for a while for ARduino to Reset
        time.sleep(3) 
        self.serial_port = serial_port
        # Prepare SerialTransport
        self.transport = SerialRadioTransport(serial_port, address='0', eom_char='\n')

        print("Connected to %s."% (selected_port))


    def add_clamp(self, clamp:ClampModel):
        clamp.last_comm_time = None
        #clamp.last_comm_lqi = None
        #clamp.last_comm_rssi = None
        clamp.last_comm_latency = 0
        self.clamps[clamp.receiver_address] = clamp
        

    def update_clamps_status(self, time_out_millis:int = 40):
        for address, clamp in self.clamps.items():
            self.update_clamp_status(clamp, time_out_millis)


    # def update_clamp_status (self, clamp:ClampModel, time_out_millis:int = 100):
    #     self.transport.reset_input_buffer()
    #     self.transport.send_message(Message(clamp.receiver_address,'0', '_'))
    #     attempt_ends = current_milli_time() + time_out_millis # 100 millis to wait for response
    #     while (current_milli_time() < attempt_ends):
    #         received_message = self.transport.receive_message()
    #         if received_message is not None:
    #             print ("Clamp%s : %s" % (received_message.sender_address, received_message.body))
    #             clamp.last_comm_time = time.time()
    #             return clamp.update_status(received_message)
    #     print ("Clamp%s : No Response" % (clamp.receiver_address))
    #     return False

    def update_clamp_status (self, clamp:ClampModel, time_out_millis:int = 40):
        # Send empty message to clamp , without auto status update
        response = self.message_clamp(clamp, "_", retry = 0, time_out_millis = time_out_millis, update_clamp_status = False)
        if response is not None:
            # Update clamp status
            update_result = clamp.update_status(response)
            if update_result:
                print ("Clamp%s Status Updated: %s" % (clamp.receiver_address, response))
                return True
            else:
                print ("Clamp%s Bad Response: %s" % (clamp.receiver_address, response))
                return False
        else:
            print ("Clamp%s : No Response" % (clamp.receiver_address))
            return False


    def message_clamp (self, clamp:ClampModel, message:str, retry:int = 0, time_out_millis:int = 100, update_clamp_status:bool = True):
        self.transport.reset_input_buffer()
        transmit_start_millis = current_milli_time() 
        # Multiple sending attempts
        for tries in range(retry + 1):
            self.transport.send_message(Message(clamp.receiver_address,'0', message))
            receive_timeout = current_milli_time() + time_out_millis 
            # Allow time to receive status response
            while (current_milli_time() < receive_timeout):
                received_message = self.transport.receive_message()
                if received_message is None: continue
                if (received_message.sender_address != clamp.receiver_address): continue
                # Save communication statistics in ClampModel
                clamp.last_comm_time = time.time()
                clamp.last_comm_latency = current_milli_time() - transmit_start_millis
                # Update the status of the clamp
                if update_clamp_status:
                    update_success = clamp.update_status(received_message.body)
                return (received_message.body)
        # In case there are no response after all attempts 
        return None
