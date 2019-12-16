import sys
import time
import os
from Phidget22.PhidgetException import *
from Phidget22.ErrorCode import *
from Phidget22.Phidget import *
from Phidget22.Net import *

class NetInfo():    
    def __init__(self):
        self.isRemote = None
        self.serverDiscovery = None
        self.hostname = None
        self.port = None
        self.password = None

class ChannelInfo():
    def __init__(self):
        self.serialNumber = -1
        self.hubPort = -1
        self.isHubPortDevice = 0
        self.channel = -1
        self.isVINT = None
        self.netInfo = NetInfo()

class EndProgramSignal(Exception):
    def __init__(self, value):
        self.value = str(value )   

class InputError(Exception):
    """Exception raised for errors in the input.

    Attributes:
        msg  -- explanation of the error
    """

    def __init__(self, msg):
        self.msg = msg

# Returns None if an error occurred, True for 'Y' and False for 'N'  
def ProcessYesNo_Input(default):

    strvar = sys.stdin.readline(100)
    if not strvar:
        raise InputError("Empty Input String")

    strvar = strvar.replace('\r\n', '\n') #sanitize newlines for Python 3.2 and older
    if (strvar[0] == '\n'):
        if (default == -1):
            raise InputError("Empty Input String")
        return default

    if (strvar[0] == 'N' or strvar[0] == 'n'):
        return False

    if (strvar[0] == 'Y' or strvar[0] == 'y'):
        return True

    raise InputError("Invalid Input")

def DisplayError(e):
    sys.stderr.write("Desc: " + e.details + "\n")
    
    if (e.code == ErrorCode.EPHIDGET_WRONGDEVICE):
        sys.stderr.write("\tThis error commonly occurs when the Phidget function you are calling does not match the class of the channel that called it.\n"
                        "\tFor example, you would get this error if you called a PhidgetVoltageInput_* function with a PhidgetDigitalOutput channel.")
    elif (e.code == ErrorCode.EPHIDGET_NOTATTACHED):
        sys.stderr.write("\tThis error occurs when you call Phidget functions before a Phidget channel has been opened and attached.\n"
                        "\tTo prevent this error, ensure you are calling the function after the Phidget has been opened and the program has verified it is attached.")
    elif (e.code == ErrorCode.EPHIDGET_NOTCONFIGURED):
        sys.stderr.write("\tThis error code commonly occurs when you call an Enable-type function before all Must-Set Parameters have been set for the channel.\n"
                        "\tCheck the API page for your device to see which parameters are labled \"Must be Set\" on the right-hand side of the list.")

def DisplayLocatePhidgetsLink():
    print("\n  | In the following example, you will be asked to provide information that specifies which Phidget the program will use. "
          "\n  | If you are unsure of any of these parameters, be sure to check www.phidgets.com/docs/Finding_The_Addressing_Information "
          "\n  | Press ENTER once you have read this message.")
    readin = sys.stdin.readline(100)
    
    print("\n--------------------")
    
def InputSerialNumber(channelInfo):
    print("\nFor all questions, enter the value, or press ENTER to select the [Default]")

    print("\n--------------------------------------")
    print("\n  | Some Phidgets have a unique serial number, printed on a white label on the device.\n"
      "  | For Phidgets and other devices plugged into a VINT Port, use the serial number of the VINT Hub.\n"
      "  | Specify the serial number to ensure you are only opening channels from that specific device.\n"
      "  | Otherwise, use -1 to open a channel on any device.")
    while (True):
        print("\nWhat is the Serial Number? [-1] ")
        strvar = sys.stdin.readline(100)
        if not strvar:
            continue

        strvar = strvar.replace('\r\n', '\n') #sanitize newlines for Python 3.2 and older
        if (strvar[0] == '\n'):
            deviceSerialNumber = -1
            break

        try:
            deviceSerialNumber = int(strvar)
        except ValueError as e:
            continue

        if (deviceSerialNumber >= -1 and deviceSerialNumber != 0):
            break

    channelInfo.deviceSerialNumber = deviceSerialNumber
    return


def InputIsHubPortDevice(channelInfo):
    isHubPortDevice = -1

    while (True):
        print("\nIs this a \"HubPortDevice\"? [y/n] ")
        try:
            isHubPortDevice = ProcessYesNo_Input(-1)
            break
        except InputError as e:
            pass

    channelInfo.isHubPortDevice = isHubPortDevice

    return

def InputVINTProperties(channelInfo, ph):
    canBeHubPortDevice = 0
    pcc = -1
    hubPort = -1
    isVINT = 0

    print("\n--------------------------------------")

    while (True):
        print("\nDo you want to specify the hub port that your device is plugged into?\n"
            "Choose No if your device is not plugged into a VINT Hub. (y/n) ")
        try:
            isVINT = ProcessYesNo_Input(-1)
            break
        except InputError as e:
            pass

    channelInfo.isVINT = isVINT
            
    # Don't ask about the HubPort and the HubPortDevice if it's not a VINT device
    if (not isVINT):
        return

    print("\n--------------------------------------")
    print("\n  | VINT Hubs have numbered ports that can be uniquely addressed.\n"
        "  | The HubPort# is identified by the number above the port it is plugged into.\n"
        "  | Specify the hub port to ensure you are only opening channels from that specific port.\n"
        "  | Otherwise, use -1 to open a channel on any port.")
    while (True):
        print("\nWhat HubPort is the device plugged into? [-1] ")
        strvar = sys.stdin.readline(100)
        if not strvar:
            continue

        strvar = strvar.replace('\r\n', '\n') #sanitize newlines for Python 3.2 and older
        if (strvar[0] == '\n'):
            hubPort = -1
            break

        try:
            hubPort = int(strvar)
        except ValueError as e:
            continue

        if (hubPort >= -1 and hubPort <= 5):
            break

    channelInfo.hubPort = hubPort

    try:
        pcc = ph.getChannelClass()
    except PhidgetException as e:
        sys.stderr.write("Runtime Error -> Getting ChannelClass: \n\t")
        DisplayError(e)
        raise

    if (pcc == ChannelClass.PHIDCHCLASS_VOLTAGEINPUT):
        print("\n--------------------------------------")
        print("\n  | A VoltageInput HubPortDevice uses the VINT Hub's internal channel to measure the voltage on the white wire.\n"
          "  | If the device you are trying to interface returns an analog voltage between 0V-5V, open it as a HubPortDevice.")
        canBeHubPortDevice = 1
    elif (pcc == ChannelClass.PHIDCHCLASS_VOLTAGERATIOINPUT):
        print("\n--------------------------------------")
        print("\n  | A VoltageRatioInput HubPortDevice uses the VINT Hub's internal channel to measure the voltage ratio on the white wire.\n"
          "  | If the device you are trying to interface returns an ratiometric voltage between 0V-5V, open it as a HubPortDevice.")
        canBeHubPortDevice = 1
    elif (pcc == ChannelClass.PHIDCHCLASS_DIGITALINPUT):
        print("\n--------------------------------------")
        print("\n  | A DigitalInput HubPortDevice uses the VINT Hub's internal channel to detect digital changes on the white wire.\n"
          "  | If the device you are trying to interface outputs a 5V digital signal, open it as a HubPortDevice.")
        canBeHubPortDevice = 1
    elif (pcc == ChannelClass.PHIDCHCLASS_DIGITALOUTPUT):
        print("\n--------------------------------------")
        print("\n  | A DigitalOutput HubPortDevice uses the VINT Hub's internal channel to output a 3.3V digital signal on the white wire.\n"
          "  | If the device you are trying to interface accepts a 3.3V digital signal, open it as a HubPortDevice.")
        canBeHubPortDevice = 1

    if (canBeHubPortDevice):
        InputIsHubPortDevice(channelInfo)

    return

def InputChannel(channelInfo):
    isHubPortDevice = 0
    channel = 0

    # Hub port devices only have a single channel, so don't ask for the channel
    if (channelInfo.isHubPortDevice):
        return

    print("\n--------------------------------------")
    print("\n  | Devices with multiple inputs or outputs of the same type will map them to channels.\n"
      "  | The API tab for the device on www.phidgets.com shows the channel breakdown.\n"
      "  | For example, a device with 4 DigitalInputs would use channels [0 - 3]\n"
      "  | A device with 1 VoltageInput would use channel 0")
    while (True):
        print("\nWhat channel# of the device do you want to open? [0] ")
        strvar = sys.stdin.readline(100)
        if not strvar:
            continue

        strvar = strvar.replace('\r\n', '\n') #sanitize newlines for Python 3.2 and older
        if (strvar[0] == '\n'):
            channel = 0
            break

        try:
            channel = int(strvar)
        except ValueError as e:
            continue

        if (channel >= 0):
            break

    channelInfo.channel = channel

    return

def SetupNetwork(channelInfo):
    hostname = ""
    password = ""
    discovery = 0
    isRemote = 0
    port = 0

    print("\n--------------------------------------")
    print("\n  | Devices can either be opened directly, or over the network.\n"
      "  | In order to open over the network, the target system must be running a Phidget Server.")
    while (True):
        print("\nIs this device being opened over the network? [y/N] ")
        try:
            isRemote = ProcessYesNo_Input(0)
            break
        except InputError as e:
            pass

    channelInfo.netInfo.isRemote = isRemote

    # if it's not remote, don't need to ask about the network
    if (not isRemote):
        return

    print("\n--------------------------------------")
    print("\n  | Server discovery enables the dynamic discovery of Phidget servers that publish their identity to the network.\n"
          "  | This allows you to open devices over the network without specifying the hostname and port of the server.")
    while (True):
        print("\nDo you want to enable server discovery? [Y/n] ")
        try:
            discovery = ProcessYesNo_Input(1)
            break
        except InputError as e:
            pass

    channelInfo.netInfo.serverDiscovery = discovery
    
    if (discovery):
        return

    print("\n--------------------------------------")
    print("\nPlease provide the following information in order to open the device")

    while (True):
        print("\nWhat is the Hostname (or IP Address) of the server? [localhost] ")
        hostname = sys.stdin.readline(100)
        if not hostname:
            continue

        hostname = hostname.replace('\r\n', '\n') #sanitize newlines for Python 3.2 and older
        if (hostname[0] == '\n'):
            hostname = "localhost"
            break

        # Remove trailing newline
        hostname = hostname.split('\n')[0]
        break

    print("\n--------------------------------------")
    while (True):
        print("\nWhat port is the server on? [5661] ")
        strvar = sys.stdin.readline(100)
        if not strvar:
            continue

        strvar = strvar.replace('\r\n', '\n') #sanitize newlines for Python 3.2 and older
        if (strvar[0] == '\n'):
            port = 5661
            break

        try:
            port = int(strvar)
        except ValueError as e:
            continue

        if (port <= 65535 and port > 0):
            break
    
    print("\n--------------------------------------")
    while (True):
        print("\nWhat is the password of the server? [] ")
        password = sys.stdin.readline(100)
        if not password:
            continue
        # Remove trailing newline
        password = password.replace('\r\n', '\n') #sanitize newlines for Python 3.2 and older

        password = password.split('\n')[0]
        break
    
    print("\n--------------------------------------")

    channelInfo.netInfo.hostname = hostname
    channelInfo.netInfo.port = port
    channelInfo.netInfo.password = password

    return
    
def PrintOpenErrorMessage(e, ph):
    sys.stderr.write("Runtime Error -> Opening Phidget Channel: \n\t")
    DisplayError(e)
    if(e.code == ErrorCode.EPHIDGET_TIMEOUT):
        sys.stderr.write("\nThis error commonly occurs if your device is not connected as specified, "
                         "or if another program is using the device, such as the Phidget Control Panel.\n")
        sys.stderr.write("\nIf your Phidget has a plug or terminal block for external power, ensure it is plugged in and powered.\n")
        if(     ph.getChannelClass() != ChannelClass.PHIDCHCLASS_VOLTAGEINPUT
            and ph.getChannelClass() != ChannelClass.PHIDCHCLASS_VOLTAGERATIOINPUT
            and ph.getChannelClass() != ChannelClass.PHIDCHCLASS_DIGITALINPUT
            and ph.getChannelClass() != ChannelClass.PHIDCHCLASS_DIGITALOUTPUT
        ):
            sys.stderr.write("\nIf you are trying to connect to an analog sensor, you will need to use the "
                              "corresponding VoltageInput or VoltageRatioInput API with the appropriate SensorType.\n")
                              
        if(ph.getIsRemote()):
            sys.stderr.write("\nEnsure the Phidget Network Server is enabled on the machine the Phidget is plugged into.")

def PrintEnableServerDiscoveryErrorMessage(e):
    sys.stderr.write("Runtime Error -> Enable Server Discovery: \n\t")
    DisplayError(e)
    if(e.code == ErrorCode.EPHIDGET_UNSUPPORTED):
        sys.stderr.write("\nThis error commonly occurs if your computer does not have the required mDNS support. "
                         "We recommend using Bonjour Print Services on Windows and Mac, or Avahi on Linux.\n")
        
    
def AskForDeviceParameters(ph):
    channelInfo = ChannelInfo()
    DisplayLocatePhidgetsLink()
    InputSerialNumber(channelInfo)
    InputVINTProperties(channelInfo, ph)    
    InputChannel(channelInfo)    
    SetupNetwork(channelInfo)
    return channelInfo



"""Manager for controller phidget device and getting a data log

Returns
-------
[type]
    [description]
"""

   
"""
* Prints descriptions and wait for user input
"""


   
"""
* Prints descriptions and wait for user input
"""

    