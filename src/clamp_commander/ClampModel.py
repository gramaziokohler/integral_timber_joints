# Each ClampModel class represent a digital twin of a real clamp
# 

class ClampModel(object):

    def __init__(
        self,
        receiver_address:str ='b',
        StepPerMM:float = 200,
        BattMin:float = 0,
        BattMax:float = 1024
        ):

        assert type(receiver_address) == str
        assert len(receiver_address) == 1
        self._receiver_address = receiver_address

        # High level mointoring configurations for the clamp
        self.StepPerMM = StepPerMM
        self.BattMin = BattMin
        self.BattMax = BattMax

        # Internal model variable store raw values
        self._raw_currentPosition = None
        self._raw_currentTarget = None
        self._raw_currentMotorPowerPercentage = None
        self._raw_statusCode = None
        self._raw_battery = None
        
        # Status Code
        self._ishomed = None
        self._isMotorRunning = None
        self._isDirectionExtend = None


    @property
    def receiver_address(self):
        return self._receiver_address

    @receiver_address.setter
    def receiver_address(self,value:str):
        assert type(value) == str
        assert len(value) == 1
        self._receiver_address = value

    # Read only properities

    @property
    def currentPosition(self):
        if self._raw_currentPosition is None: return None
        return self._raw_currentPosition / self.StepPerMM

    @property
    def currentTarget(self):
        if self._raw_currentTarget is None: return None
        return self._raw_currentTarget / self.StepPerMM

    @property
    def currentMotorPowerPercentage(self):
        return self._raw_currentMotorPowerPercentage

    @property
    def batteryPercentage(self):
        if self._raw_battery is None: return None
        return int ((self._raw_battery - self.BattMin ) / (self.BattMax - self.BattMin) * 100)
    @property
    def statusCode(self):
        return self._raw_statusCode

    @property
    def ishomed(self):
        return self._ishomed

    @property
    def isMotorRunning(self):
        return self._isMotorRunning

    @property
    def isDirectionExtend(self):
        return self._isDirectionExtend

    # Exposed setter function to take a the status String
    # Return true if all sainity checks are passed
    def update_status(self, statusString:str):
        values = statusString.split(',')
        # Check if the number of csv items are equal to 5
        if (len(values) != 5):
            return False

        # Try cast all of the items into numbers
        try:
            for i in range(5):
                values[i] = int(values[i])
        except:
            return False

        # Check if the values are within their min max range
        if (values[0] > 15 or values[0] < 0 ): return False
        if (values[3] > 100 or values[3] < -100): return False
        if (values[4] > 1024 or values[4] < 0): return False

        # All check passed, set internal variables
        self.__set_statusCode(values[0])
        self._raw_currentPosition = values[1]
        self._raw_currentTarget = values[2]
        self._raw_currentMotorPowerPercentage = values[3]
        self._raw_battery = values[4]
        
        # Return true is update is successful
        return True

    # Internal setter functions

    # Check if a bit is set
    @staticmethod
    def __is_set(x, n):
        return x & 1 << n != 0

    # set the status flags according to the status code
    def __set_statusCode(self, statusCodeNumber):
        self._ishomed = self.__is_set(statusCodeNumber, 0)
        self._isMotorRunning = self.__is_set(statusCodeNumber, 1)
        self._isDirectionExtend = self.__is_set(statusCodeNumber, 2)

    def __str__(self):
        if self.currentPosition is None: return "ClampModel Object Address=%s (Not Connected)" % self.receiver_address
        homed_string = "Homed" if self.ishomed else "Not-Homed"
        return "ClampModel Object Address=%s BatteryLevel=%s%% Postion=%4.2fmm %s"  % (self.receiver_address, self.batteryPercentage, self.currentPosition, homed_string)

if __name__ == "__main__":

    # Construct a clamp model
    # ClampModel(Address,StepPerMM, BattMin, BattMax)
    clamp = ClampModel('1', 918, 880, 1024)
    print (clamp)
