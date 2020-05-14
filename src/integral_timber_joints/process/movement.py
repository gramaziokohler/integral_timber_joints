##################################
# Base Classes for all Movement
# Do not use them directly
##################################

class Movement(object):
    """ Base class of all movements """
    def __init__(self, operator_stop_before = False, operator_stop_after = False):
        self.operator_stop_before = operator_stop_before # type: bool
        self.operator_stop_after = operator_stop_after # type: bool

class RoboticMovement(Movement):
    def __init__(self, target_frame, attached_tool_id = None, attached_beam_id = None):
        Movement.__init__(self)
        self.target_frame = target_frame # type: Frame
        self.attached_tool_id = attached_tool_id # type: Optional[str]
        self.attached_beam_id = attached_beam_id # type: Optional[str]
        self.trajectory = None # type: Optional[compas_fab.robots.JointTrajectory]

######################################
# Movement Classes that can be used
######################################

class OperatorLoadBeamMovement(Movement):
    """ Manual movement to place beam on Pickup Station
    Default: operator_stop_after = True
    """
    def __init__(self, beam_id):
        Movement.__init__(self, operator_stop_after = True)
        self.beam_id = beam_id

    def __str__(self):
        return "Load Beam (%s) to pickup station" % (self.beam_id)

class RoboticFreeMovement(RoboticMovement):
    def __init__(self, target_frame, attached_tool_id = None, attached_beam_id = None):
        RoboticMovement.__init__(self, target_frame, attached_tool_id, attached_beam_id)

    def __str__(self):
        return "Free Move to %s" % (self.target_frame)

class RoboticLinearMovement(RoboticMovement):
    def __init__(self, target_frame, attached_tool_id = None, attached_beam_id = None):
        RoboticMovement.__init__(self, target_frame, attached_tool_id, attached_beam_id)

    def __str__(self):
        return "Linear Move to %s" % (self.target_frame)

class RoboticDigitalOutput(Movement):
    def __init__(self, digital_output):
        # type: (DigitalOutput) -> None
        Movement.__init__(self)
        self.digital_output = digital_output

    def __str__(self):
        if self.digital_output == DigitalOutput.LockTool:
            return "IO LockTool"
        if self.digital_output == DigitalOutput.UnlockTool:
            return "IO UnlockTool"
        if self.digital_output == DigitalOutput.OpenGripper:
            return "IO OpenGripper"
        if self.digital_output == DigitalOutput.CloseGripper:
            return "IO CloseGripper"
        raise NotImplementedError("What type of DigitalOutput is this (%s)" % self.digital_output)

class DigitalOutput(object):
    LockTool = 1
    UnlockTool = 2
    OpenGripper = 3
    CloseGripper = 4
