from ClampModel import ClampModel
from SerialCommander import SerialCommander

class SerialCommanderTokyo(SerialCommander):
    
    def start(self):
        SerialCommander.start(self)
        self.clamp1 = ClampModel('1', 918, 95.0, 94.0, 225.0, 880.0, 1004.0)
        self.clamp2 = ClampModel('2', 918, 95.0, 94.0, 225.0, 880.0, 1004.0)
        self.add_clamp(self.clamp1)
        self.add_clamp(self.clamp2)

    def send_all_clamps_to_jaw_position(self, jaw_position_mm:float, velocity_mm_sec:float):
        for addr, clamp in self.clamps.items():
            send_success = self.send_clamp_to_jaw_position(clamp, jaw_position_mm, velocity_mm_sec)
            if not send_success:
                self.stop_clamps()
                return False
        return True