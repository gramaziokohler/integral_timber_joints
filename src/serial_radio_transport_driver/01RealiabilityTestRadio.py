import threading
import serial       #imports the pyserial module
import time         #imports time module required for delays (sleep functions)
import datetime
from datetime import timedelta
import random

# This script is a mess*
# Combining srial functions and test into one file to validate the C1101 radio function.
#
# This scripts implemented a simple serial handle to read one single line from Serial Port
#
# readline_from_port function reads available characters from the serialport and trys to link together a line of string.
# (This function needs to be improved for the final release)
#
# test_one_addition performs the test to validate the radio funcion.
# (Test by sending a numerical number in ASCII format and expect a returned value +1 larger)
#
# test_many_addition performs many tests to benchmark the performance of the system.


# Global settings
connected = False
port = 'COM5'
baud = 115200
SerialHeader = "ba"
SerialTernimation =  '\x04' # // 10 = LineFeed '\n' , 13 = CarrageReturn '\r'

def try_parse_int(s, base=10, val=None):
  try:
    return int(s, base)
  except ValueError:
    return val

def readline_from_port(serial_port,timeout = 1):
    loopTimeOut = datetime.datetime.utcnow() + timedelta(seconds=timeout)
    escaping = False
    serial_string_builder = '' #String Builder
    # Infinite loop until a non-empty line is found.
    # Each read attempt reads all available chars before adding them one by one.
    # Current implementation strips the () Bracket comments from the serial line.
    # Fucute implementation should separate this into a different function.
    # Current implementation discards the remaining bytes after a SerialTermination is found.
    # Future should save this information for fetching next line.
    while (True):

        #Read all available chars
        chars_to_read = serial_port.inWaiting()
        if (chars_to_read>0): #if incoming bytes are waiting to be read from the serial input buffer
            data_str = serial_port.read(chars_to_read).decode('ascii') #read the bytes and convert from binary array to ASCII
            #serial_string_builder += data_str
            #print(data_str, end='') #print the incoming string without putting a new-line ('\n') automatically after every print()

        #Examine each char before adding it into the string builder
        for i in range(chars_to_read):
            if (data_str[i] == '('):
                escaping = True
                continue
            if (data_str[i] == ')'):
                escaping = False
                continue
            if (data_str[i] == SerialTernimation):
                if (len(serial_string_builder) > 0):
                    return serial_string_builder
                else:
                    continue
            if (escaping == False):
                serial_string_builder += data_str[i]

        #If timeout, the current string build is returned.
        if (datetime.datetime.utcnow() > loopTimeOut):
            return serial_string_builder
        #Slight wait before peeping into the Serial Buffer again.
        time.sleep(0.01) # Optional: sleep 10 ms (0.01 sec)

def test_one_addition(serial_port,val):
    text = str(val)
    print ("Sending message: " + text , end='')
    serial_port.reset_input_buffer()    #Flush buffer before sending new text
    serial_port.write((text + SerialTernimation).encode("ascii", "replace"))

    #Read a new line
    lineString = readline_from_port(serial_port,timeout=0.1)
    #print ("Received:" + lineString)

    #Parse the string to integer
    receivedIntValue = try_parse_int(lineString)

    #Initiate one retry if necessary
    if (receivedIntValue == None):
            print (" - (Try 1) Int not received.  Message: " + lineString)
            serial_port.reset_input_buffer()    #Flush buffer before sending new text
            print ("Sending message: " + text , end='')
            serial_port.write((text + SerialTernimation).encode("ascii", "replace"))
            lineString = readline_from_port(serial_port)
            receivedIntValue = try_parse_int(lineString)

            # If the value is still not integer, print error and continue
            if (receivedIntValue == None):
                print (" - (Try 2) Int not received.  Message: " + lineString)
                return False

    #Check if the value is correct as expected
    if (receivedIntValue == val + 1 ):
        print (" - OK")
        return True
    else:
        print (" - Not Correct: " + str(receivedIntValue) + "(Expected=" + str(val) +")")
        return False


def test_many_addition(serial_port,range_start,range_end,count):
    successCount = 0
    testStartTime = datetime.datetime.utcnow()

    for i in range(count):
        randomNumber = random.randrange(range_start, range_end)
        result = test_one_addition(serial_port,randomNumber)
        if (result):
            successCount += 1
        #Small pause between tests
        time.sleep(0.0002)
    print("Test Completed: " + str(successCount) + " / " + str(count) + " test successful (" + str(successCount * 100 / count) + "%)")
    testDuration = datetime.datetime.utcnow() - testStartTime
    print ("Total Test Time: " + str(testDuration.total_seconds()) + "s . (" + str(testDuration.total_seconds()/count) + ")")


def reverse_string(s):
    return s[::-1]

def test_one_string_reverse(serial_port,val):

    text = str(val)
    text_reversed = reverse_string(text)
    print ("Sending message: " + text , end='')
    serial_port.reset_input_buffer()    #Flush buffer before sending new text
    serial_port.write((text + SerialTernimation).encode("ascii", "replace"))

    #Read a new line
    lineString = readline_from_port(serial_port,timeout=0.1)
    #print ("Received:" + lineString)

    #Initiate one retry if necessary
    if (lineString != text_reversed):
            print (" - (Try 1) String not correct.  Message: " + lineString)
            serial_port.reset_input_buffer()    #Flush buffer before sending new text
            print ("Sending message: " + text , end='')
            serial_port.write((text + SerialTernimation).encode("ascii", "replace"))
            lineString = readline_from_port(serial_port)
            receivedIntValue = try_parse_int(lineString)

            # If the value is still not integer, print error and continue
            if (lineString != text_reversed):
                print (" - (Try 2) String not correct.  Message: " + lineString)
                return False

    #Check if the value is correct as expected
    if (lineString == text_reversed):
        print (" - OK")
        return True
    else:
        print (" - Not Correct: " + str(receivedIntValue) + "(Expected=" + str(val) +")")
        return False


def test_many_string_reverse(serial_port,string_length,count):

    # Prepare a set of letters in ascii valid range
    import string
    letters = string.ascii_letters + string.digits

    successCount = 0
    testStartTime = datetime.datetime.utcnow()

    for i in range(count):
        random_string = ''.join(random.choice(letters) for j in range(string_length))
        result = test_one_string_reverse(serial_port,random_string)
        if (result):
            successCount += 1
        #Small pause between tests
        time.sleep(0.0002)
    print("Test Completed: " + str(successCount) + " / " + str(count) + " test successful (" + str(successCount * 100 / count) + "%)")
    testDuration = datetime.datetime.utcnow() - testStartTime
    print ("Total Test Time: " + str(testDuration.total_seconds()) + "s . (" + str(testDuration.total_seconds()/count) + ")")


def main():
    # Setup Serial Port for communication
    serial_port = serial.Serial(port, baud, timeout=1)
    time.sleep(2) #Waits for Arduino to reset and connection to establish.

    #Configure Radio
    serial_port.write("0a\x04")
    time.sleep(1) #Waits for Arduino to reset and connection to establish.

    #test_many_addition(serial_port,1000,9999,500)
    test_many_string_reverse(serial_port,60,500)

### Test Parameters:
# test_many_addition can go up to 10 digit long
# test_many_string_reverse can go up to 61 char long

if (__name__ == '__main__'):
    main()
