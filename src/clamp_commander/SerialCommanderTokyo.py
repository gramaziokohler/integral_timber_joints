from ClampModel import ClampModel
from SerialCommander import SerialCommander

class SerialCommanderTokyo(SerialCommander):
    
    def __init__(self):
        SerialCommander.__init__(self)
        self.clamp1 = ClampModel('1', 918, 95.0, 94.0, 225.0, 880.0, 1004.0)
        self.clamp2 = ClampModel('2', 918, 95.0, 94.0, 225.0, 880.0, 1004.0)
        self.add_clamp(self.clamp1)
        self.add_clamp(self.clamp2)

