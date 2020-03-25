from serial import Serial
import time
import datetime
import serial.tools.list_ports

from serial_radio_transport_driver.ReliableMessenger import ReliableMessenger
from serial_radio_transport_driver.SerialTransport import SerialRadioTransport
from serial_radio_transport_driver.Message import Message

from ClampModel import ClampModel
from typing import Optional, Dict, List
current_milli_time = lambda: int(round(time.time() * 1000))

class SerialCommander(object):
    
    def __init__(self):
        self.clamps : Dict[str, ClampModel] = {}
        self.serial_port = None
        self.status_update_interval_ms: int = 500 # ms
        self.serial_default_retry = 3
        self.serial_default_timeout = 100

        pass

    @property
    def is_connected(self) -> bool:
        return ((self.serial_port is not None) and (self.serial_port.isOpen()))

    ''' Launch a GUI to allow user to select a COM PORT.
    Then, connect to that COM port
    return true if success
    '''
    def connect_with_CLI (self):
        # Check connected COM ports
        ports = list(serial.tools.list_ports.comports())
        if len(ports) == 0 :
            print ("No COM Ports available. Terminating")
            return False
        
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

        return self.connect(selected_port)

    ''' Connect to a specified COM port
    return true if success
    '''
    def connect(self, port_name:str):
        # Check if existing port is connected, if so, disconned first:
        if (self.serial_port is not None):
            if (self.serial_port.isOpen()):
                print("CMD: Closing existing port: %s" % (self.serial_port.port))
                self.serial_port.close()

        # Connect to Serial Port
        print("CMD: Connecting to %s ..." % (port_name))
        try:
            serial_port = Serial(port_name, 115200, timeout=1)
            serial_port.reset_input_buffer()
        except:
            print("CMD: Connection failed. Check if other termianals are already connected to %s" % (port_name))
            return False

        # Need to wait for a while for ARduino to Reset
        time.sleep(3) 
        self.serial_port = serial_port
        # Prepare SerialTransport
        self.transport = SerialRadioTransport(serial_port, address='0', eom_char='\n')
        print("CMD: Connected to %s."% (serial_port.port))
        return True

    def add_clamp(self, clamp:ClampModel):
        clamp.last_comm_time = None
        #clamp.last_comm_lqi = None
        #clamp.last_comm_rssi = None
        clamp.last_comm_latency = 0
        self.clamps[clamp.receiver_address] = clamp
        
    def update_clamps_status(self, retry:int = 0, time_out_millis:int = 40):
        results = []
        for address, clamp in self.clamps.items():
            result = self.update_clamp_status(clamp, retry = retry, time_out_millis = time_out_millis)
            results.append(result)
        return results

    def update_clamp_status (self, clamp:ClampModel, retry:int = 0, time_out_millis:int = 40) -> bool:
        # Send empty message to clamp , without auto status update
        response = self.message_clamp(clamp, "_", retry = retry, time_out_millis = time_out_millis, update_clamp_status = False)
        if response is not None:
            # Update clamp status
            update_result = clamp.update_status(response)
            if update_result:
                print ("CMD: Clamp%s Status Updated: %s" % (clamp.receiver_address, response))
                return True
            else:
                print ("CMD: Clamp%s Bad Response: %s" % (clamp.receiver_address, response))
                return False
        else:
            print ("CMD: Clamp%s : No Response" % (clamp.receiver_address))
            return False


    def message_clamp (self, clamp:ClampModel, message:str, retry:int = -1, time_out_millis:int = -1, update_clamp_status:bool = True) -> str:
        # Get default communication values
        if (retry == -1): retry = self.serial_default_retry
        if (time_out_millis == -1): time_out_millis = self.serial_default_timeout

        self.transport.reset_input_buffer()
        transmit_start_millis = current_milli_time() 
        # Multiple sending attempts
        for tries in range(retry + 1):
            try:
                #Sometimes send_message() can fail. if that is the case, we skip the rest of this try loop 
                self.transport.send_message(Message(clamp.receiver_address,'0', message))
            except:
                continue
            # Allow time to receive status response
            receive_timeout = current_milli_time() + time_out_millis 
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

    # If velocity_mm_sec is None, velocity will not be set.
    # Return True if all messages are successfully sent
    def send_clamp_to_jaw_position(self, clamp: ClampModel, jaw_position_mm:float, velocity_mm_sec:float = None):
        # Check position min max
        if (jaw_position_mm < clamp.SoftLimitMin_mm):
            raise ValueError("Target Position (%s) < Limit (%s)"%(jaw_position_mm, clamp.SoftLimitMin_mm))
        if (jaw_position_mm > clamp.SoftLimitMax_mm):
            raise ValueError("Target Position (%s) > Limit (%s)"%(jaw_position_mm, clamp.SoftLimitMax_mm))

        # Convert velocity and target into step units
        motor_position = int(clamp.to_motor_position(jaw_position_mm))
        print("g" + str(motor_position))
        if (velocity_mm_sec is not None):
            velocity_step_sec = int(velocity_mm_sec * clamp.StepPerMM)
            print("v" + str(velocity_step_sec))
        
        # Set Speed
        if (velocity_mm_sec is not None):
            response1 = self.message_clamp(clamp, "v" + str(velocity_step_sec))
            # Stop and Returns False if the send is not successful
            if response1 is None: return False
        # Send Target to Go
        response2 = self.message_clamp(clamp, "g" + str(motor_position))
        # Returns True if the send is successful
        if (response2 is not None):
            clamp._last_set_position = jaw_position_mm
            return True
        else:
            return False

    def send_clamps_to_jaw_position(self, clamps:List[ClampModel], jaw_position_mm:float, velocity_mm_sec:float):
        processed_clamps = []
        for clamp in clamps:
            send_success = self.send_clamp_to_jaw_position(clamp, jaw_position_mm, velocity_mm_sec)
            # Keep track of messages sent to clamps, stop them if any one of the messaging failed.
            processed_clamps.append(clamp)
            if not send_success:
                self.stop_clamps(processed_clamps)
                return False
        return True

    def send_all_clamps_to_jaw_position(self, jaw_position_mm:float, velocity_mm_sec:float):
        return self.send_clamps_to_jaw_position(self.clamps.values(), jaw_position_mm, velocity_mm_sec)

    def stop_clamps(self, clamps:List[ClampModel]) -> List[bool]:
        successes = []
        for address, clamp in self.clamps.items():
            response = self.message_clamp(clamp, "s")
            successes.append(response is not None)
        return successes

    def stop_all_clamps(self) -> List[bool]:
        return self.stop_clamps(self.clamps.values())

    def set_clamp_velocity(self, clamp: ClampModel, velocity_mm_sec:float) -> bool:
        velocity_step_sec = int(velocity_mm_sec * clamp.StepPerMM)
        response = self.message_clamp(clamp, "v" + str(velocity_step_sec))
        result = (response is not None)
        if result: 
            clamp._last_set_velocity = velocity_mm_sec
        return result
