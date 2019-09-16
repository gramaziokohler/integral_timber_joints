
class Message(object):

    def __init__(
        self,
        receiver_address:str ='b',
        sender_address:str ='a',
        body:str = ""):

        assert type(receiver_address) == str
        assert type(sender_address) == str
        assert type(body) == str

        assert len(receiver_address) == 1
        assert len(sender_address) == 1

        self._receiver_address = receiver_address
        self._sender_address = sender_address
        self._body = body


    @property
    def length(self):
        return len(self.body)

    @property
    def receiver_address(self):
        return self._receiver_address

    @receiver_address.setter
    def receiver_address(self,value:str):
        assert type(value) == str
        assert len(value) == 1
        self._receiver_address = value

    @property
    def sender_address(self):
        return self._sender_address

    @sender_address.setter
    def sender_address(self,value:str):
        assert type(value) == str
        assert len(value) == 1
        self._sender_address = value

    @property
    def body(self):
        return self._body

    @body.setter
    def body(self,value:str):
        assert type(value) == str
        self._body = value


if __name__ == "__main__":

    #Construct message object using default values.
    msg1 = Message()
    msg1.receiver_address = 'b'
    msg1.sender_address = 'a'
    msg1.body = "HellowWorld"
    print (msg1.receiver_address, msg1.sender_address, msg1.body)

    #Construct message object by supplying values.
    msg2 = Message('b','a',"HellowWorld")
    print (msg2.receiver_address, msg2.sender_address, msg2.body)

    assert msg1.receiver_address == msg2.receiver_address
    assert msg1.sender_address == msg2.sender_address
    assert msg1.body == msg2.body
