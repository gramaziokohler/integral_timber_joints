from datetime import datetime, timedelta
from serial_radio_transport_driver.SerialTransport import SerialRadioTransport
from serial_radio_transport_driver.Message import Message

class ReliableMessenger(object):

        def __init__(self, transport:SerialRadioTransport, retry_max:str = 4, ack_timeout_millis:int = 100):
            self._transport = transport
            self.set_retry(retry_max)
            self.set_ack_timeout(ack_timeout_millis)
            self._incomingMessage : Message = False
            self._newMessageFlag = False

        def available(self):
            #The flag is to ensure multiple calls to available() will not corrupt message
            if (self._newMessageFlag): return True

            #Receive the message and immediately send ACK.
            if (self._transport.available()):
                self._incomingMessage = self._transport.receive_message()
                self._newMessageFlag = True
                self._send_ack(self._incomingMessage)
                return True

            #Reachable if no message is available
            return False

        def receive_message(self) -> Message :
            if (self.available() == False): return None
            self._newMessageFlag = False
            return self._incomingMessage

        def send_message(self, message:Message) -> int:
            # Send the message
            self._transport.send_message(message)
            # Compute the expected ACK Checksum String
            expected_string = self._computeACKString(message)

            # Wait for returned ACK nessage and automatic retry
            # Try for max_retry number of times
            for retry_count in range(self._retry_max):
                # Within timeout peirod, try to read an incoming message
                time_out = datetime.utcnow() + timedelta(milliseconds=self._ack_timeout_millis)
                while (datetime.utcnow() < time_out):
                    if (self._transport.available()):
                        msg = self._transport.receive_message()
                        # If successful, return the number of retrys
                        if (msg.body == expected_string):
                            return retry_count
                        else:
                        # If unsuccessful, keep waiting for message
                            pass
                #Retry sending message again.
                if (retry_count < self._retry_max - 1 ):
                    self._transport.send_message(message)
            # When exhausted number of retrys, return -1
            return - self._retry_max

        def discard_unread_message(self, message:Message) -> None:
            _newMessageFlag = False
            while (self._transport.available()):
                self._transport.receiveMessage()

        def set_retry(self, retry_max:int) -> None:
            self._retry_max = retry_max

        def set_ack_timeout(self, millisec:int) -> None:
            self._ack_timeout_millis = millisec

        def _send_ack(self, original_message:Message) -> None:
            # Prepare the ack message
            ack_message = Message()
            ack_message.sender_address = self._transport.get_address()
            ack_message.receiver_address = original_message.sender_address
            ack_message.body = self._computeACKString(original_message)
            # Send message directly by transport
            self._transport.send_message(ack_message)

        def _computeACKString(self, original_message:Message) -> str:
            return "ACK" + chr(self.stringChecksum(original_message.body))


        def stringChecksum(self, original_string:str) -> int:
            c = 0
            index = 0
            for char in original_string:
                c ^= (ord(char) + index)    #Repeated Bitwise OR with (the char + index)
                index +=1
            return (c % 25) + 97            #Limits the returned byte to be ASCII a-z

if __name__ == "__main__":
    from serial import Serial
    import time

    #Prepare Serial Port
    serial_port = Serial('COM3', 115200, timeout=1)
    serial_port.reset_input_buffer()
    time.sleep(2)

    #Prepare SerialTransport
    transport = SerialRadioTransport(serial_port)
    transport.set_end_of_msg_char(chr(4))

    #Prepare Message 1 - Configure Radio Address
    msg = Message(chr(7),chr(7),'0a')
    transport.send_message(msg)

    #Prepare Message 2 - Configure Radio Freq
    msg = Message(chr(7),chr(7),'12')
    transport.send_message(msg)

    #Prepare ReliableMessenger
    messenger = ReliableMessenger(transport)

    #Send message
    messenger.set_retry(5)
    msg = Message('b','a','')
    resend_count = 0
    success_count = 0
    tries = 100
    for i in range(tries):
        #Send Message
        msg.body = "|abcd" + str(i) + "|"
        result = messenger.send_message(msg)
        #Counting if send is a success
        if (result >= 0):
            resend_count += result
            success_count +=1
        #print (i,end='\r')
        print ("Send message : "+ str (msg.body) +" retrys: " + str(result),end='\r')
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
    print("Success: " + str(success_count) + " / "+ str(tries))
    print("Resend: " + str(resend_count))

