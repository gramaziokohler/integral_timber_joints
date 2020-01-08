import sys
import time
import os
import traceback
from Phidget22.Devices.VoltageRatioInput import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *


# Sensor Settings
#       b and a are defined as calibration as follows:
#       Assuming force Value (N) = bx+a
#       b is the slope, a is the intercept
#       x is the raw data V/V ratio
SENSOR_CAL_SLOPE = 3.6304e6   # Slope
SENSOR_CAL_INTERCEPT = -4.2711e1    # Intercept

# Logging variable setting
time.sleep(0.1)
param_motor_gear = 72
print("Enter Voltage (V)")
param_voltage_mv = float(sys.stdin.readline(100).rstrip()) * 1000
param_current_ma = float(10000)

# Log Session Settings
LOG_FOLDER_NAME = "data"
LOG_SESSION_FILENAME = "result_w" + str(int(param_motor_gear)) + "_mv" + str(int(param_voltage_mv)) + "_ma" + str(int(param_current_ma)) + ".csv"
LOG_MULTIPLE_RESULT_FILENAME = "results.csv"

HERE = os.path.dirname(__file__)
FILE_LOG_SESSION = os.path.join(HERE, LOG_FOLDER_NAME, LOG_SESSION_FILENAME)
FILE_MULTIPLE_RESULT = os.path.join(HERE, LOG_FOLDER_NAME, LOG_MULTIPLE_RESULT_FILENAME)

# Sensor update peirod in millisec
LOG_UPDATE_INTERVAL_MS = 50



try:
    from PhidgetHelperFunctions import *
except ImportError:
    sys.stderr.write("\nCould not find PhidgetHelperFunctions. Either add PhdiegtHelperFunctions.py to your project folder "
                      "or remove the import from your project.")
    sys.stderr.write("\nPress ENTER to end program.")
    readin = sys.stdin.readline()
    sys.exit()


""" Prints Phidget board info when it is started. """
def onAttachHandler(self):
    
    ph = self
    try:
        
        print("\nSensor Attached:")
        
        """
        * Get device information and display it.
        """
        channelClassName = ph.getChannelClassName()
        serialNumber = ph.getDeviceSerialNumber()
        channel = ph.getChannel()
        if(ph.getDeviceClass() == DeviceClass.PHIDCLASS_VINT):
            hubPort = ph.getHubPort()
            print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) +
                "\n\t-> Hub Port: " + str(hubPort) + "\n\t-> Channel:  " + str(channel) + "\n")
        else:
            print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) +
                    "\n\t-> Channel:  " + str(channel) + "\n")
    
        """
        * DataInterval defines the minimum time between VoltageRatioChange events.
        * DataInterval can be set to any value from MinDataInterval to MaxDataInterval.
        """
        print("\n\tSetting DataInterval to " + str(LOG_UPDATE_INTERVAL_MS) + " ms.")
        ph.setDataInterval(LOG_UPDATE_INTERVAL_MS)

        """
        * VoltageRatioChangeTrigger will affect the frequency of VoltageRatioChange events, by limiting them to only occur when
        * the voltage ratio changes by at least the value set.
        """
        print("\tSetting Voltage Ratio ChangeTrigger to 0.0")
        ph.setVoltageRatioChangeTrigger(0.0)
        
        """
        * You can find the appropriate SensorType for your sensor in its User Guide and the VoltageRatioInput API
        * SensorType will apply the appropriate calculations to the voltage ratio reported by the device
        * to convert it to the sensor's units.
        * SensorType can only be set for Sensor Port voltage ratio inputs (VINT Ports and Analog Input Ports)
        """
        if(ph.getChannelSubclass() == ChannelSubclass.PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT):
            print("\tSetting VoltageRatio SensorType")
            ph.setSensorType(VoltageRatioSensorType.SENSOR_TYPE_VOLTAGERATIO)
        
        # Class variable to be initialized. 
        self.startTime = time.time()
        self.datacounter = 0
        
    except PhidgetException as e:
        print("\nError in Attach Event:")
        DisplayError(e)
        traceback.print_exc()
        return

""" Performs log writing when voltageRatioChange event is raised by Phidget """
def onVoltageRatioChangeHandler(self, voltageRatio):
    # Compute timeSinceStart 
    timeNow = time.time()
    timeSinceStart = timeNow - self.startTime
    # Compute force based on calibration
    compensatedvalue = voltageRatio * SENSOR_CAL_SLOPE + SENSOR_CAL_INTERCEPT

    # Occationally print the force value to console
    self.datacounter +=1
    if self.datacounter % 10 == 0:
        print("Force(N): " + str(compensatedvalue))

    # Save value to log file here:
    self.file.write(str(timeNow))
    self.file.write(",")
    self.file.write(str(timeSinceStart))
    self.file.write(",")
    self.file.write(str(voltageRatio))    
    self.file.write(",")
    self.file.write(str(compensatedvalue))
    self.file.write("\n")

    # Min Max Value Logging
    if not hasattr(self, 'minForce'):
        self.minForce = compensatedvalue #Run Once
    elif compensatedvalue < self.minForce:
        self.minForce = compensatedvalue
    if not hasattr(self, 'maxForce'):
        self.maxForce = compensatedvalue #Run Once
    elif compensatedvalue > self.maxForce:
        self.maxForce = compensatedvalue
    if not hasattr(self, 'maxVoltageRatio'):
        self.maxVoltageRatio = voltageRatio #Run Once
    elif compensatedvalue > self.maxVoltageRatio:
        self.maxVoltageRatio = voltageRatio
                

    """
    * Outputs the VoltageRatioInput's most recently reported sensor value.
    * Fired when a VoltageRatioInput channel with onSensorChangeHandler registered meets DataInterval and ChangeTrigger criteria
    *
    * @param self The VoltageRatioInput channel that fired the SensorChange event
    * @param sensorValue The reported sensor value from the VoltageRatioInput channel
    """
    def onSensorChangeHandler(self, sensorValue, sensorUnit):
        print("[Sensor Event] -> Sensor Value: " + str(sensorValue) + sensorUnit.symbol)


# New channel object
ch = VoltageRatioInput()

# Make directory and open file for writting
if not os.path.exists(LOG_FOLDER_NAME):
    os.makedirs(LOG_FOLDER_NAME)
f = open(FILE_LOG_SESSION, "w+") # Overwrite

# Write CSV file header
f.write("Time, Time Since Begin, V/V Raw Data, Force \n")
ch.file = f

# Setup Phidget device parameters
ch.setDeviceSerialNumber(-1)
ch.setHubPort(-1)
ch.setIsHubPortDevice(False)
ch.setChannel(0)   


print("\n--------------------------------------")
# Setup call back functions that is raised when voltage is changed
ch.setOnAttachHandler(onAttachHandler)
ch.setOnVoltageRatioChangeHandler(onVoltageRatioChangeHandler)

# Open Phidget connection
ch.openWaitForAttachment(5000)

# Wait for user to press Enter to stop
print("Press Enter to stop recording")
readin = sys.stdin.readline(1)

ch.close()
print("\n Phidget Challen Closed")

# Logging session finished. Write out statistics.
if hasattr(ch, 'minForce'):
    print("Min Force: " + str(ch.minForce))
if hasattr(ch, 'maxForce'):
    print("Max Force: " + str(ch.maxForce))
if hasattr(ch, 'minForce')&hasattr(ch, 'maxForce'):
    print("Max Difference: " + str(ch.maxForce-ch.minForce))           

# Write out min max data to results file
with open(FILE_MULTIPLE_RESULT, "a") as rf: # Append
    rf.write(str(int(time.time())))
    rf.write(',')
    rf.write(str(param_motor_gear))
    rf.write(',')
    rf.write(str(param_voltage_mv))
    rf.write(',')
    rf.write(str(param_current_ma))
    rf.write(',')
    rf.write(str(ch.maxVoltageRatio))
    rf.write(',')
    rf.write(str(ch.maxForce))
    rf.write('\n')
    
# Write CSV file header
f.write("Time, Time Since Begin, V/V Raw Data, Force \n")
ch.file = f



if (f is not None):
    print ("Data file saved in: data/" + LOG_SESSION_FILENAME + ".csv")
    f.close()

print ("Phidget Script Ended.")

