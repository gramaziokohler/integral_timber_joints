from ClampModel import ClampModel
from SerialCommander import SerialCommander

# SerialCommanderTokyo is a SErialCommander that contains the initialization of the clamps
# used in the Tokyo project.

class SerialCommanderTokyo(SerialCommander):
    
    def __init__(self):
        SerialCommander.__init__(self)

        # 918 step/mm is derived from 
        # - 17 steps per rev encoder x 4 phase
        # - 1:54 gearbox
        # - 4mm lead screw

        # Soft Limit Min Max is calibrated to the physical clamp that was constructed for
        # the tokyo project, an extension block was attached to the jaw,
        # causing the jaw opening to be 94mm when homed.

        # Batt Min Max Value 860 to 1004 is calibrated according to the LiPo Charger's percentage reference
        # It is safe to use the battery to 0% as indicated here.
        self.clamp1 = ClampModel('1', 918, 95.0, 94.0, 225.0, 860.0, 1004.0)
        self.clamp2 = ClampModel('2', 918, 95.0, 94.0, 225.0, 860.0, 1004.0)
        self.add_clamp(self.clamp1)
        self.add_clamp(self.clamp2)

