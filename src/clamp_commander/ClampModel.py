# Each ClampModel class represent a digital twin of a real clamp
# 
import time

class ClampModel(object):

    def __init__(
        self,
        receiver_address:str ='b',
        StepPerMM:float = 200.0,
        JawOffset:float = 0.0, # (in mmm) / Jaw Position = Offset +  (_raw_currentPosition /  StepPerMM)
        SoftLimitMin_mm:float = -0.1,
        SoftLimitMax_mm:float = 100,
        BattMin:float = 0.0,
        BattMax:float = 1024.0
        ):

        assert type(receiver_address) == str
        assert len(receiver_address) == 1
        self._receiver_address = receiver_address

        # High level mointoring configurations for the clamp (# Settings of the controller)
        self.StepPerMM: float = StepPerMM
        self.JawOffset: float  = JawOffset
        self.BattMin: float  = BattMin
        self.BattMax: float  = BattMax
        self.SoftLimitMin_mm: float  = SoftLimitMin_mm
        self.SoftLimitMax_mm: float  = SoftLimitMax_mm

        # State variable of the clamp firmware, raw values.
        self._raw_currentPosition: int = None
        self._raw_currentTarget: int = None
        self._raw_currentMotorPowerPercentage: int = None
        self._raw_statusCode: int = None
        self._raw_battery: int = None

        # Status Code Derived state.
        self._ishomed: bool = None
        self._isMotorRunning: bool = None
        self._isDirectionExtend: bool = None

        # Status / communication update time
        self._state_timestamp: int = None
        self._last_set_velocity: float = None
        self._last_set_position: float = None

    @property
    def receiver_address(self):
        return self._receiver_address

    @receiver_address.setter
    def receiver_address(self,value:str):
        assert type(value) == str
        assert len(value) == 1
        self._receiver_address = value

    # Read only properities

    """Returns the Motor Position Raw Value in steps  """
    @property
    def currentMotorPosition(self):
        if self._raw_currentPosition is None: return None
        return self._raw_currentPosition

    """Returns the Jaw Position in mm (Offset Included) """
    @property
    def currentJawPosition(self):
        if self._raw_currentPosition is None: return None
        return self.to_jaw_position(self._raw_currentPosition)

    """Returns the Motor Position Target that is set by PID Controller """
    @property
    def currentMotorTarget(self):
        if self._raw_currentTarget is None: return None
        return self._raw_currentTarget / self.StepPerMM

    """Returns the Motor Power Input that is set by PID Controller """
    @property
    def currentMotorPowerPercentage(self):
        return self._raw_currentMotorPowerPercentage

    """Returns the Battery Percentage (offset included) """
    @property
    def batteryPercentage(self) -> int:
        if self._raw_battery is None: return None
        return int ((self._raw_battery - self.BattMin ) / (self.BattMax - self.BattMin) * 100)
    
    @property
    def statusCode(self) -> int:
        return self._raw_statusCode

    @property
    def ishomed(self) -> bool:
        return self._ishomed

    @property
    def isMotorRunning(self):
        return self._isMotorRunning

    @property
    def isDirectionExtend(self):
        return self._isDirectionExtend

    @property
    def state_to_data(self):
        data = {}
        data['state_timestamp'] = self._state_timestamp
        data['raw_currentPosition'] = self._raw_currentPosition
        data['raw_currentTarget'] = self._raw_currentTarget
        data['raw_currentMotorPowerPercentage'] = self._raw_currentMotorPowerPercentage
        data['raw_statusCode'] = self._raw_statusCode
        data['raw_battery'] = self._raw_battery
        data['is_running'] = self.isMotorRunning

        return data

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
        
        # Record the timestamp of this status
        self._state_timestamp = self.current_milli_time()

        # Return true is update is successful
        return True

    # Internal setter functions

    # Check if a bit is set
    @staticmethod
    def __is_set(x, n) -> bool:
        return x & 1 << n != 0

    @staticmethod
    def current_milli_time() -> int:
        return int(round(time.time() * 1000))

    # Method to convert between jaw_position and motor_position
    def to_motor_position(self, jaw_position_mm:float):
        return (jaw_position_mm - self.JawOffset ) * self.StepPerMM

    def to_jaw_position(self, motor_position_step:int):
        return (motor_position_step / self.StepPerMM) + self.JawOffset

    # set the status flags according to the status code
    def __set_statusCode(self, statusCodeNumber):
        self._ishomed = self.__is_set(statusCodeNumber, 0)
        self._isMotorRunning = self.__is_set(statusCodeNumber, 1)
        self._isDirectionExtend = self.__is_set(statusCodeNumber, 2)

    def __str__(self):
        # Readable
        return "Clamp %s" % self.receiver_address
        # if self.currentMotorPosition is None: return "ClampModel Object Address=%s (Not Connected)" % self.receiver_address
        # homed_string = "Homed" if self.ishomed else "Not-Homed"
        # return "ClampModel Object Address=%s BatteryLevel=%s%% Postion=%4.2fmm %s"  % (self.receiver_address, self.batteryPercentage, self.currentJawPosition, homed_string)
    
    def __repr__(self):
        # unambiguous
        return "Clamp %s" % self.receiver_address

if __name__ == "__main__":

    # Construct a clamp model
    # ClampModel(Address,StepPerMM, BattMin, BattMax)
    clamp = ClampModel('1', 918, 880, 1024)
    print (clamp)
