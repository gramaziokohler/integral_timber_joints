SerialCommandLibrary

G01X6Y3
$3=0
$2
G54

h
?
+
-
g9
e5
t0,50
$



// Receive sequenced messages and buffered them. This reassembles bytes from Arduino Serial.read()

// Send ACK message when message is received

// Escape and Unescape NewLine character for Serial transfer. This allows a single message packet to contain binary data.
// NL character is used for termination. ESC character is used to escape.

// Send message and wait for ACK confirmation (Blocking or Nonblocking)
// Automatic resend after timeout or NACK 

// Only one command can be in the buffer at a single time.
// Newly received command 


//Create object.
SoftwareSerial mySerial(10, 11);
SerialCommand myMessenger(mySerial,parseCommand());

//Service the object as often as possible.
myMessenger.listen();

// Reading message
if(myMessenger.messageAvailable()){
	String msg;
	msg = myMessenger.readMessage();
	// Discard unread message
	msg = myMessenger.discardUnreadMessages();
}

// Send Message (Blocking Till Ack)
myMessenger.setRetry(4);
myMessenger.setAckTimeout(4);

myMessenger.sendMessage("hello")

// Send Message (NonBlocking Till Ack)

boolean sendMessageComplete = false;
boolean sendMessageSuccess = false;

void sendMessageFunction(boolean success){
	sendMessageComplete = true;
	sendMessageSuccess = success;
}

myMessenger.sendMessageAsyc("hello",callback)
while(!sendMessageComplete){
	//Do something else
}
if (!sendMessageSuccess) // Do something to deal with the lost partner.








