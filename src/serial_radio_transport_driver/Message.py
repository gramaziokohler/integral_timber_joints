
class Message(object):

    def __init__(
        self,
        _receiver_address='b',
        _sender_address='a',
        _body = ""):

        self.receiver_address = _receiver_address
        self.sender_address = _sender_address
        self.body = _body
        pass

    @property
    def length(self):
        return len(self.body)


if __name__ == "__main__":

    #Construct message object using default values.
    msg = Message()
    msg.body = "HellowWorld"
    print (msg.sender_address,msg.receiver_address,msg.body)

    #Construct message object by supplying values.
    msg = Message('b','a',"HellowWorld")
    print (msg.sender_address,msg.receiver_address,msg.body)
