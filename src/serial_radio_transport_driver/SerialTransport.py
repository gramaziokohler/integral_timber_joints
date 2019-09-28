from serial import Serial
from serial_radio_transport_driver.Message import Message

class SerialRadioTransport(object):

    def __init__(self, _serial:Serial, eom_char:str = '\n', address:str = 'a', frequency:int = 2, channel:int = 19):
        # Handel the creation of sub class objects
        self.serial = _serial
        
        self._receive_buffer = ""
        self._messageWiatingFlag = False
        self._messageForMe = True
        
        self._eom_char = eom_char
        self.set_end_of_msg_char(eom_char)

        self._address = None
        self._freq = None
        self._channel = None
        # Clear Serial port just in case
        self.serial.reset_input_buffer()
        # Configure Radio
        self.set_address(address)
        self.set_frequency(frequency)
        self.set_channel(channel)

    def send_message(self, message:Message):
        self.serial.reset_input_buffer()    #Flush buffer before sending new text
        #print ("Sending Message:" + msg.receiver_address+msg.sender_address+msg.body)
        self.serial.write((message.receiver_address + message.sender_address + message.body + self._eom_char).encode("ascii", "replace"))
        
    def receive_message(self):
        # Attempt to read the message if it exist
        if ( self.available() == False): return None

        #Reconstruct the message
        received_message = Message()
        received_message.receiver_address = self._receive_buffer[0]
        received_message.sender_address = self._receive_buffer[1]
        received_message.body = self._receive_buffer[2:]
        self._receive_buffer = ""       #Destroy the buffer contents
        self._messageWiatingFlag = False     #Lower the message waiting flag
        return received_message


    def available(self) -> bool:
        #If the _messageWiatingFlag is up, then we should immediately return true without reading more data.
        if (self._messageWiatingFlag == True): return True

        #Receive as much character as possible and return true if EOM is detected.
        while (self.serial.inWaiting()>0):

            #Read one available char
            newChar = self.serial.read(1).decode('ascii')

            #If this is the first character, check if message is for me.
            if (len(self._receive_buffer) == 0):
                if (newChar != self._address):
                    self._messageForMe  = False   #Set messageForMe Flag . This stops subsquent storage.

            # End of message character - end char array with 0 and return true
            if (newChar == self._eom_char):
                if (self._messageForMe):
                    self._messageWiatingFlag = True
                    return True
                else:                           # Message is not for me but now it ends.
                    self._receive_buffer = ""       #Reset Receive Index (This will consume the message)
                    self._messageForMe = True      #Reset messageForMe Flag
                    continue                        #Skip the following storage in buffer

            # Normal character - proceed to store in buffer
            else:
                if (self._messageForMe):
                    self._receive_buffer += newChar
        # Reachable if no more characters to read
        return False

    def set_address(self, address:str):
        assert type(address) is str
        assert (len(address) == 1)
        self._address = address
        # Configure Radio Address
        msg = Message(chr(7),chr(7),'0'+ address)
        self.send_message(msg)

    def set_frequency(self, frequency:int):
        assert type(frequency) is int
        assert (frequency >= 0 & frequency <=3)
        self._freq = frequency
        # Configure Radio Address
        msg = Message(chr(7),chr(7),'1'+ str(frequency))
        self.send_message(msg)

    def set_channel(self, channel:int):
        assert type(channel) is int
        assert (channel >= 0 & channel <=127)
        assert (channel != 4)
        self._channel = channel
        # Configure Radio Address
        msg = Message(chr(7),chr(7),'2'+ chr(channel))
        self.send_message(msg)

    def get_address(self) ->str:
        return self._address

    def set_end_of_msg_char(self, char):
        assert type(char) is str
        assert (len(char) == 1)
        assert ord(char) != 0
        self._eom_char = char

    def get_end_of_msg_char(self):
        return self._eom_char

if __name__ == "__main__":
    import time
    from datetime import datetime, timedelta

    #Prepare Serial Port
    serial_port = Serial('COM3', 115200, timeout=1)
    serial_port.reset_input_buffer()
    time.sleep(2) # 2 Second is needed

    #Prepare SerialTransport
    transport = SerialRadioTransport(serial_port)
    transport.set_end_of_msg_char(chr(4))

    #Prepare Message 1 - Configure Radio Address
    msg = Message(chr(7),chr(7),'0a')
    transport.send_message(msg)

    #Prepare Message 2 - Configure Radio Freq
    msg = Message(chr(7),chr(7),'12')
    transport.send_message(msg)

    time.sleep(0.2)

    # #Prepare Message 3 - Sending message
    # msg = Message()
    # msg.receiver_address = 'd'
    # msg.sender_address = 'c'
    # msg.body = ""
    # #Send message
    # transport.send_message(msg)

    serial_port.write([66,65,4])
    
    time.sleep(1)
    def send_one_test():
        #Prepare Message 3 - Sending message
        msg = Message('b','a','123')
        #Send message
        transport.send_message(msg)

        #Try to receive some message
        loopTimeOut = datetime.utcnow() + timedelta(seconds=0.5)
        while (datetime.utcnow() < loopTimeOut):
            if (transport.available()):
                msg = transport.receive_message()
                print ("Message Received:" + str(msg.body) + " from: " + str(msg.sender_address))

        print ("Terminating")
    
    for i in range(10):
        send_one_test()


# Configure Radio Manually: \a\a0a\x0004\a\a12\x0004


