from compas.geometry.primitives.frame import Frame
from compas_fab.robots.trajectory import JointTrajectory

from integral_timber_joints.process.state import ObjectState

try:
    from typing import Dict, Iterator, List, Optional, Tuple
except:
    pass

##################################
# Base Classes for all Movement
# Do not use them directly
##################################


class Movement(object):
    """ Base class of all movements """

    def __init__(self, operator_stop_before=False, operator_stop_after=False):
        self.operator_stop_before = operator_stop_before  # type: bool
        self.operator_stop_after = operator_stop_after  # type: bool
        self.end_state = {}  # type: dict[str, ObjectState]

    def to_data(self):
        """Simpliest way to get this class serialized.
        """
        return self.data

    @classmethod
    def from_data(cls, data):
        """Construct a Movement from structured data. Subclass must add their properity to
        the data properity.
        """
        movement = cls()
        movement.data = data
        return movement

    @property
    def data(self):
        data = {
            'operator_stop_before': self.operator_stop_before,
            'operator_stop_after': self.operator_stop_after,
            'end_state': self.end_state,
        }
        return data

    @data.setter
    def data(self, data):
        self.operator_stop_before = data.get('operator_stop_before', False)
        self.operator_stop_after = data.get('operator_stop_after', False)
        self.end_state = data.get('end_state', {})


class RoboticMovement(Movement):
    def __init__(self, target_frame=None, attached_tool_id=None, attached_beam_id=None):
        Movement.__init__(self)
        self.target_frame = target_frame  # type: Frame
        self.attached_tool_id = attached_tool_id  # type: Optional[str]
        self.attached_beam_id = attached_beam_id  # type: Optional[str]
        self.trajectory = None  # type: Optional[JointTrajectory]

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
        super(RoboticMovement, type(self)).data.fset(self, data)
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

    def __init__(self, beam_id=None, grasp_face=None, target_frame=None):
        Movement.__init__(self, operator_stop_after=True)
        self.beam_id = beam_id
        self.grasp_face = grasp_face
        self.target_frame = target_frame  # type: Frame

    def __str__(self):
        return "Load Beam ('%s') for pickup at %s (Side %s face up)." % (self.beam_id, self.target_frame, self.grasp_face)

    @property
    def data(self):
        """ Sub class specific data added to the dictionary of the parent class
        """
        data = super(OperatorLoadBeamMovement, self).data
        data['beam_id'] = self.beam_id
        data['grasp_face'] = self.grasp_face
        data['target_frame'] = self.target_frame
        return data

    @data.setter
    def data(self, data):
        """ Sub class specific data loaded
        """
        super(OperatorLoadBeamMovement, type(self)).data.fset(self, data)
        self.beam_id = data.get('beam_id', None)
        self.grasp_face = data.get('grasp_face', None)
        self.target_frame = data.get('target_frame', None)


class RoboticFreeMovement(RoboticMovement):

    def __str__(self):
        return "Free Move to %s" % (self.target_frame)


class RoboticLinearMovement(RoboticMovement):

    def __str__(self):
        return "Linear Move to %s" % (self.target_frame)


class RoboticDigitalOutput(Movement):
    def __init__(self, digital_output=None, tool_id=None, beam_id=None):
        # type: (DigitalOutput, str, str) -> None
        """ `tool_id` relates to the tool that is being operated.
        `beam_id` should be filled in for Open or Close Gripper movements that
        involved letting go or picking up a beam. This helps the state manage to figure
        out which beam is picked up and attached or no longer attached. 

        For Clamp Closing Gripper to attach to a fixed beam, `beam_id` should be left None. 
        """
        Movement.__init__(self)
        self.digital_output = digital_output
        self.tool_id = tool_id
        self.beam_id = beam_id

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

    @property
    def data(self):
        """ Sub class specific data added to the dictionary of the parent class
        """
        data = super(RoboticDigitalOutput, self).data
        data['digital_output'] = self.digital_output
        data['tool_id'] = self.tool_id
        data['beam_id'] = self.beam_id
        return data

    @data.setter
    def data(self, data):
        """ Sub class specific data loaded
        """
        super(RoboticDigitalOutput, type(self)).data.fset(self, data)
        self.digital_output = data['digital_output']
        self.tool_id = data.get('tool_id', None)
        self.beam_id = data.get('beam_id', None)


class DigitalOutput(object):
    LockTool = 1
    UnlockTool = 2
    OpenGripper = 3
    CloseGripper = 4


class ClampsJawMovement(Movement):
    def __init__(self, jaw_positions=[], clamp_ids=[]):
        Movement.__init__(self)
        self.jaw_positions = jaw_positions  # type: list[float]
        self.clamp_ids = clamp_ids  # type: list[str]

    def __str__(self):
        return "Clamps %s Jaw Move to %s" % (self.clamp_ids, self.jaw_positions)

    @property
    def data(self):
        """ Sub class specific data added to the dictionary of the parent class
        """
        data = super(ClampsJawMovement, self).data
        data['jaw_positions'] = self.jaw_positions
        data['clamp_ids'] = self.clamp_ids
        return data

    @data.setter
    def data(self, data):
        """ Sub class specific data loaded
        """
        super(ClampsJawMovement, type(self)).data.fset(self, data)
        self.jaw_positions = data.get('jaw_positions', [])
        self.clamp_ids = data.get('clamp_ids', [])


class RoboticClampSyncLinearMovement(RoboticMovement, ClampsJawMovement):

    def __init__(self, target_frame=None, attached_tool_id=None, attached_beam_id=None, jaw_positions=[], clamp_ids=[]):
        RoboticMovement.__init__(self, target_frame, attached_tool_id, attached_beam_id)
        ClampsJawMovement.__init__(self, jaw_positions, clamp_ids)

    @property
    def data(self):
        """ Sub class specific data added to the dictionary of the parent class
        """
        data = super(RoboticClampSyncLinearMovement, self).data
        return data

    @data.setter
    def data(self, data):
        """ Sub class specific data loaded
        """
        super(RoboticClampSyncLinearMovement, type(self)).data.fset(self, data)

    def __str__(self):
        return "Robot-Clamp Linear Sync Move to %s. Clamps %s Jaw Move to %s." % (self.target_frame, self.clamp_ids, self.jaw_positions)
