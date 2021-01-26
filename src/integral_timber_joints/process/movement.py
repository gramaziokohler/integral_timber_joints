##################################
# Base Classes for all Movement
# Do not use them directly
##################################

class Movement(object):
    """ Base class of all movements """
    def __init__(self, operator_stop_before = False, operator_stop_after = False):
        self.operator_stop_before = operator_stop_before # type: bool
        self.operator_stop_after = operator_stop_after # type: bool

    def to_data(self):
        """Simpliest way to get this class serialized.
        """
        return self.data

    @classmethod
    def from_data(cls, data):
        """Construct a Movement from structured data. Subclass must add their properity to
        the data properity.
        """
        movement = cls(False, False)
        movement.data = data
        return movement

    @property
    def data(self):
        data = {
            'operator_stop_before' : self.operator_stop_before,
            'operator_stop_after' : self.operator_stop_after,
        }
        return data

    @data.setter
    def data(self, data):
        self.operator_stop_before = data['operator_stop_before']
        self.operator_stop_after = data['operator_stop_after']


class RoboticMovement(Movement):
    def __init__(self, target_frame, attached_tool_id = None, attached_beam_id = None):
        Movement.__init__(self)
        self.target_frame = target_frame # type: Frame
        self.attached_tool_id = attached_tool_id # type: Optional[str]
        self.attached_beam_id = attached_beam_id # type: Optional[str]
        self.trajectory = None # type: Optional[compas_fab.robots.JointTrajectory]
    
    @property
    def data(self):
        """ Sub class specific data added to the dictionary of the parent class
        """
        data = super(RoboticMovement, self).data
        data['target_frame'] = self.target_frame
        data['attached_tool_id'] = self.attached_tool_id
        data['attached_beam_id'] = self.attached_beam_id
        data['trajectory'] = self.trajectory
        return data
    
    @data.setter
    def data(self, data):
        """ Sub class specific data loaded
        """
        super(RoboticMovement, self).data = data
        self.target_frame = data['target_frame']
        self.attached_tool_id = data['attached_tool_id']    
        self.attached_beam_id = data['attached_beam_id']    
        self.trajectory = data['trajectory']

######################################
# Movement Classes that can be used
######################################

class OperatorLoadBeamMovement(Movement):
    """ Manual movement to place beam on Pickup Station
    Default: operator_stop_after = True
    """
    def __init__(self, beam_id, grasp_face):
        Movement.__init__(self, operator_stop_after = True)
        self.beam_id = beam_id
        self.grasp_face = grasp_face

    def __str__(self):
        return "Load Beam (%s) to pickup station. Side %s face-upwards" % (self.beam_id, self.grasp_face)

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
