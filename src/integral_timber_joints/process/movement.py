import os
import time

from compas.geometry.primitives.frame import Frame
from compas_fab.robots import Configuration
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
    """ Base class of all movements.
    `Movement.end_state`
    A diction of all the object's ObjectState object representing the scene after the movement is completed.
    No `start_state` exist becuase it is simply the `end_state` of the previous movement.

    `Movement.planning_priority`
    Integer that can only have three possible values
    - -1 means planning not required
    - 0 means normal
    - 1 meaning the movement should be planned first

    `Movement.movement_id`
    - A String on the format of  "A%i_M%i" % (act_n, mov_n)
    - This is a unique identifier for the movement.

    `Movement.tag`
    - Optional human readable text describing the sementic meaning of the movement within the Action.
    - This text should make sense in the absence of the Action description.

    `operator_stop_before` and `operator_stop_after`
    - If set to a non empty String, a operator stop will be triggered before or after the action execution.
    - If set to None, no stop will happen. (default)
    """

    def __init__(self, operator_stop_before=None, operator_stop_after=None, planning_priority=0, tag=None):
        self.operator_stop_before = operator_stop_before  # type: str
        self.operator_stop_after = operator_stop_after  # type: str
        self.end_state = {}  # type: dict[str, ObjectState]
        self.planning_priority = planning_priority  # type: int
        self.movement_id = ""  # type: str
        self.tag = tag or "Generic Movement"  # type: str
        self.path_from_link = None # type: list[Frame]

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
            'planning_priority': self.planning_priority,
            'movement_id': self.movement_id,
            'tag': self.tag,
            'path_from_link': self.path_from_link,
        }
        return data

    @data.setter
    def data(self, data):
        self.operator_stop_before = data.get('operator_stop_before', None)
        self.operator_stop_after = data.get('operator_stop_after', None)
        self.planning_priority = data.get('planning_priority', 0)
        self.end_state = data.get('end_state', {})
        self.movement_id = data.get('movement_id', "")
        self.tag = data.get('tag', "")
        self.path_from_link = data.get('path_from_link', None)

    @property
    def filepath(self):
        # type: () -> str
        """ Returns the location of the json file when saved externally.
        This is useful to save a movement containing computed trajectory and has a large file size
        e.g.: 'movements\A2_M2.json'
        """
        return os.path.join("movements", "%s.json" % self.movement_id)

    @property
    def short_summary(self):
        return '{}(#{}, {})'.format(self.__class__.__name__, self.movement_id, self.tag)


class RoboticMovement(Movement):
    """ Generic class for movements related to Robot Arm movement.

    `RoboticMovement.allowed_collision_matrix`
    - List of Tuple[object_id, object_id] representing allowable collision between beams and tools

    `RoboticMovement.target_configuration`
    - Optional Robotic Configuration (J / E values) for the target,
    When set, the configuration will become a constraint in the end-state for the path planning.
    When None, the pathplanner should decide for the configuration using IK sampling.


    """

    def __init__(self, target_frame=None, attached_tool_id=None, attached_beam_id=None, planning_priority=0, operator_stop_before=None,
                 operator_stop_after=None, speed_type="", target_configuration=None, allowed_collision_matrix=[], tag=None, seed=None):
        # type: (Frame, str, str, int, str, str, str, Configuration, list(tuple(str,str)), str, int) -> RoboticMovement
        Movement.__init__(self, operator_stop_before=operator_stop_before, operator_stop_after=operator_stop_after,
                          planning_priority=planning_priority, tag=tag)
        self.target_frame = target_frame  # type: Frame
        self.attached_tool_id = attached_tool_id  # type: Optional[str]
        self.attached_beam_id = attached_beam_id  # type: Optional[str]
        self.speed_type = speed_type  # type: str # A string linking to a setting
        self.trajectory = None  # type: Optional[JointTrajectory]
        self.path_from_link = None # Optional[dictionary: robot link name[str] -> list(Frame)]
        self.target_configuration = target_configuration  # type: Optional[Configuration]
        self.allowed_collision_matrix = allowed_collision_matrix  # type: list(tuple(str,str))
        self.tag = tag or "Generic Robotic Movement"
        self.seed = seed # or hash(time.time())

    @property
    def data(self):
        """ Sub class specific data added to the dictionary of the parent class
        """
        data = super(RoboticMovement, self).data
        data['target_frame'] = self.target_frame
        data['attached_tool_id'] = self.attached_tool_id
        data['attached_beam_id'] = self.attached_beam_id
        data['trajectory'] = self.trajectory
        data['path_from_link'] = self.path_from_link
        data['speed_type'] = self.speed_type
        data['target_configuration'] = self.target_configuration
        data['allowed_collision_matrix'] = self.allowed_collision_matrix
        data['seed'] = self.seed
        return data

    @data.setter
    def data(self, data):
        """ Sub class specific data loaded
        """
        super(RoboticMovement, type(self)).data.fset(self, data)
        self.target_frame = data['target_frame']
        self.attached_tool_id = data['attached_tool_id']
        self.attached_beam_id = data['attached_beam_id']
        self.trajectory = data.get('trajectory', None)
        self.path_from_link = data.get('path_from_link', None)
        self.speed_type = data.get('speed_type', "")
        self.target_configuration = data.get('target_configuration', None)
        self.allowed_collision_matrix = data.get('allowed_collision_matrix', [])
        self.seed = data.get('seed', None)

    @property
    def short_summary(self):
        return '{}(#{}, {}, traj {})'.format(self.__class__.__name__, self.movement_id, self.tag,
                                                             int(self.trajectory is not None))

######################################
# Movement Classes that can be used
######################################


class OperatorLoadBeamMovement(Movement):
    """ Manual movement to place beam on Pickup Station
    Default: operator_stop_after = True
    """

    def __init__(self, beam_id=None, grasp_face=None, target_frame=None, tag=None):
        # type: (str, int, Frame, str) -> OperatorLoadBeamMovement
        Movement.__init__(self, operator_stop_after="Confirm Beam placed in Pickup", planning_priority=-1, tag=tag)
        self.beam_id = beam_id
        self.grasp_face = grasp_face
        self.target_frame = target_frame  # type: Frame
        self.tag = tag or "Opeartor Load Beam to Pickup Location"

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
    def __init__(self, digital_output=None, tool_id=None, beam_id=None, operator_stop_before=None, operator_stop_after=None, tag=None, planning_priority=-1):
        # type: (DigitalOutput, str, str, str, str, str, int) -> RoboticDigitalOutput
        """ `tool_id` relates to the tool that is being operated.
        `beam_id` should be filled in for Open or Close Gripper movements that
        involved letting go or picking up a beam. This helps the state manage to figure
        out which beam is picked up and attached or no longer attached.

        For Clamp Closing Gripper to attach to a fixed beam, `beam_id` should be left None.
        """
        Movement.__init__(self, operator_stop_before=operator_stop_before,
                          operator_stop_after=operator_stop_after, planning_priority=planning_priority)
        self.digital_output = digital_output
        self.tool_id = tool_id
        self.beam_id = beam_id

        # Default tags for IO actions
        if self.digital_output == DigitalOutput.LockTool:
            self.tag = tag or "ToolChanger Lock Tool"
        elif self.digital_output == DigitalOutput.UnlockTool:
            self.tag = tag or "ToolChanger Unlock Tool"
        elif self.digital_output == DigitalOutput.OpenGripper:
            self.tag = tag or "Open Gripper"
        elif self.digital_output == DigitalOutput.CloseGripper:
            self.tag = tag or "Close Gripper"

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
    def __init__(self, jaw_positions=[], clamp_ids=[], speed_type="", planning_priority=-1, tag=None):
        Movement.__init__(self, planning_priority=planning_priority, tag=tag)
        self.jaw_positions = jaw_positions  # type: list[float]
        self.clamp_ids = clamp_ids  # type: list[str]
        self.speed_type = speed_type  # type: str # A string linking to a setting
        self.tag = tag or "Clamp Jaw Move"

    def __str__(self):
        return "Clamps %s Jaw Move to %s" % (self.clamp_ids, self.jaw_positions)

    @property
    def data(self):
        """ Sub class specific data added to the dictionary of the parent class
        """
        data = super(ClampsJawMovement, self).data
        data['jaw_positions'] = self.jaw_positions
        data['clamp_ids'] = self.clamp_ids
        data['speed_type'] = self.speed_type
        return data

    @data.setter
    def data(self, data):
        """ Sub class specific data loaded
        """
        super(ClampsJawMovement, type(self)).data.fset(self, data)
        self.jaw_positions = data.get('jaw_positions', [])
        self.clamp_ids = data.get('clamp_ids', [])
        self.speed_type = data.get('speed_type', "")


class RoboticClampSyncLinearMovement(RoboticMovement, ClampsJawMovement):

    def __init__(self, target_frame=None, attached_tool_id=None, attached_beam_id=None, jaw_positions=[], clamp_ids=[], planning_priority=1, speed_type="",
                 allowed_collision_matrix=[], tag=None):
        tag = tag or "Robot and Clamp Sync Move"
        RoboticMovement.__init__(self, target_frame, attached_tool_id, attached_beam_id, planning_priority=planning_priority,
                                 speed_type=speed_type, allowed_collision_matrix=allowed_collision_matrix, tag=tag)
        ClampsJawMovement.__init__(self, jaw_positions, clamp_ids,
                                   planning_priority=planning_priority, speed_type=speed_type, tag=tag)

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
