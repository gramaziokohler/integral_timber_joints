import os
import time

from compas.geometry import Frame, Transformation, multiply_matrices
from compas_fab.robots import Configuration
from compas_fab.robots.trajectory import JointTrajectory

try:
    from typing import Dict, List, Optional, Tuple

    from integral_timber_joints.process import RobotClampAssemblyProcess
except:
    pass

from integral_timber_joints.process.state import ObjectState

try:
    from typing import Any, Dict, Iterator, List, Optional, Tuple
except:
    pass

##################################
# Base Classes for all Movement
# Do not use them directly
##################################


class Movement(object):
    """ Base class of all movements.
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

    `state_diff`
    - They record the changed state of objects in the scene.
    - It is implemented as a dictionary where key is Tuple (object_id, ['f', 'a', 'c']),
        value is Any[Frame, bool, Configuration]
    """

    def __init__(self, operator_stop_before=None, operator_stop_after=None, planning_priority=0, tag=None):
        self.operator_stop_before = operator_stop_before  # type: str
        self.operator_stop_after = operator_stop_after  # type: str
        self.state_diff = {}  # type: dict[Tuple[str, str], Any[Frame, Configuration, bool]]
        self.planning_priority = planning_priority  # type: int
        self.movement_id = ""  # type: str
        self.tag = tag or "Generic Movement"  # type: str

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
        flattened_state_diff = {}
        for key, value in self.state_diff.items():
            flattened_state_diff[str(key)] = value
        data = {
            'operator_stop_before': self.operator_stop_before,
            'operator_stop_after': self.operator_stop_after,
            'flattened_state_diff': flattened_state_diff,
            'planning_priority': self.planning_priority,
            'movement_id': self.movement_id,
            'tag': self.tag,
        }
        return data

    @data.setter
    def data(self, data):
        import ast
        self.operator_stop_before = data.get('operator_stop_before', None)
        self.operator_stop_after = data.get('operator_stop_after', None)
        self.planning_priority = data.get('planning_priority', 0)
        flattened_state_diff = data.get('flattened_state_diff', {})
        self.state_diff = {}
        for key, value in flattened_state_diff.items():
            self.state_diff[ast.literal_eval(key)] = value
        self.movement_id = data.get('movement_id', "")
        self.tag = data.get('tag', "")

    def get_filepath(self, subdir='movements'):
        # type: (str) -> str
        """ Returns the location of the json file when saved externally.
        This is useful to save a movement containing computed trajectory and has a large file size
        e.g.: 'movements\A2_M2.json'
        """
        return os.path.join(subdir, "%s.json" % self.movement_id)

    @property
    def short_summary(self):
        return '{}(#{}, {})'.format(self.__class__.__name__, self.movement_id, self.tag)

    def create_state_diff(self, process, clear=True):
        # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
        """The create_state_diff() functions are are implemented within each of the Movement child class.

        Create entry(s) to `state_diff` regarding object(s) that have changed
        (`a` = attachment status,`f` = frame or `c` = configuration) during this movement.

        `self.state_diff` is implemented as a dictionary where key is Tuple (object_id, ['f', 'a', 'c']),
        value is Any[Frame, bool, Configuration]

        `clear` can be set to `False` when multiple `create_state_diff()` are
        being call by multi-inhereted child class.
        """
        raise NotImplementedError("Movement.create_state_diff() is not implemented by child class: %s" % (self.__class__.__name__))


class RoboticMovement(Movement):
    """ Generic class for movements related to Robot Arm movement.

    `RoboticMovement.allowed_collision_matrix`
    - List of Tuple[object_id, object_id] representing allowable collision between beams and tools

    `RoboticMovement.target_configuration`
    - Optional Robotic Configuration (J / E values) for the target,
    When set, the configuration will become a constraint in the end-state for the path planning.
    When None, the pathplanner should decide for the configuration using IK sampling.


    """

    def __init__(
        self,
        target_frame=None,  # type: Frame # Target of the Robotic Movement
        attached_objects=[],  # type: List[str]
        t_flange_from_attached_objects=[],  # type: List[Transformation]
        planning_priority=0,  # type: int
        operator_stop_before=None,  # type: str
        operator_stop_after=None,  # type: str
        speed_type="",  # type: str # A string linking to a setting
        target_configuration=None,  # type: Optional[Configuration]
        allowed_collision_matrix=[],  # type: list(tuple(str,str))
        tag=None,  # type: str
        seed=None  # type: int
    ):
        # type: (...) -> RoboticMovement
        """
        `speed_type` - a string linking to a speed setting in execution controller
        """

        Movement.__init__(self, operator_stop_before=operator_stop_before, operator_stop_after=operator_stop_after,
                          planning_priority=planning_priority, tag=tag)
        self.target_frame = target_frame  # type: Frame
        self.attached_objects = attached_objects  # type: List[str]
        self.t_flange_from_attached_objects = t_flange_from_attached_objects  # type: List[Transformation]
        self.speed_type = speed_type  # type: str # A string linking to a setting
        self.trajectory = None  # type: Optional[JointTrajectory]
        self.target_configuration = target_configuration  # type: Optional[Configuration]
        self.allowed_collision_matrix = allowed_collision_matrix  # type: list(tuple(str,str))
        self.tag = tag or "Generic Robotic Movement"
        self.seed = seed  # or hash(time.time())

    @property
    def data(self):
        """ Sub class specific data added to the dictionary of the parent class
        """
        data = super(RoboticMovement, self).data
        data['target_frame'] = self.target_frame
        data['attached_objects'] = self.attached_objects
        data['t_flange_from_attached_objects'] = self.t_flange_from_attached_objects
        data['trajectory'] = self.trajectory
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
        self.attached_objects = data.get('attached_objects', [])
        self.t_flange_from_attached_objects = data.get('t_flange_from_attached_objects', [])
        self.trajectory = data.get('trajectory', None)
        self.speed_type = data.get('speed_type', "")
        self.target_configuration = data.get('target_configuration', None)
        self.allowed_collision_matrix = data.get('allowed_collision_matrix', [])
        self.seed = data.get('seed', None)

    @property
    def short_summary(self):
        return '{}(#{}, {}, traj {})'.format(self.__class__.__name__, self.movement_id, self.tag,
                                             int(self.trajectory is not None))

    def create_state_diff(self, process, clear=True):
        # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
        if clear:
            self.state_diff = {}
        # Change robot flange frame
        self.state_diff[('robot', 'f')] = self.target_frame

        # If target_configuration is available, pass it to robot kinematic_config.
        if self.target_configuration is not None:
            self.state_diff[('robot', 'c')] = self.target_configuration

        # Change ToolChanger location, it is always attached to robot
        self.state_diff[('tool_changer', 'f')] = self.target_frame

        # Change attached tool location
        for attached_object_id, t_flange_from_attached_object in zip(self.attached_objects, self.t_flange_from_attached_objects):
            t_world_from_flange = Transformation.from_frame(self.target_frame)
            object_frame = Frame.from_transformation(t_world_from_flange * t_flange_from_attached_object)
            self.state_diff[(attached_object_id, 'f')] = object_frame


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
        self.tag = tag or "Opeartor Load Beam (%s) to Pickup Location" % self.beam_id

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

    def create_state_diff(self, process, clear=True):
        # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
        if clear:
            self.state_diff = {}
        self.state_diff[(self.beam_id, 'f')] = self.target_frame


class OperatorAttachToolMovement(Movement):
    """ Manual movement to attach a tool to a beam
    The gripper will wait for operator to lock gripper.
    Configuration of the tool will be changed by calling close_gripper().

    Default: operator_stop_after = True
    """

    def __init__(self, beam_id=None, joint_id=None, tool_type=None, tool_id=None, target_frame=None, beam_already_attached_to_robot=True, tag=None):
        # type: (str, tuple[str, str], str, str, Frame, bool, str) -> OperatorAttachToolMovement
        Movement.__init__(self, operator_stop_after="Confirm Tool placed.", planning_priority=-1, tag=tag)
        self.beam_id = beam_id
        self.joint_id = joint_id
        self.tool_type = tool_type
        self.tool_id = tool_id
        self.target_frame = target_frame
        self.beam_already_attached_to_robot = beam_already_attached_to_robot
        self.tag = tag or "Opeartor Attach Tool to Beam"

    def __str__(self):
        return "Load Beam ('%s') for pickup at %s." % (self.beam_id, self.target_frame)

    @property
    def data(self):
        """ Sub class specific data added to the dictionary of the parent class
        """
        data = super(OperatorAttachToolMovement, self).data
        data['beam_id'] = self.beam_id
        data['joint_id'] = self.joint_id
        data['tool_type'] = self.tool_type
        data['tool_id'] = self.tool_id
        data['target_frame'] = self.target_frame
        data['beam_already_attached_to_robot'] = self.beam_already_attached_to_robot
        data['tag'] = self.tag
        return data

    @data.setter
    def data(self, data):
        """ Sub class specific data loaded
        """
        super(OperatorAttachToolMovement, type(self)).data.fset(self, data)
        self.beam_id = data.get('beam_id', None)
        self.joint_id = data.get('joint_id', None)
        self.tool_type = data.get('tool_type', None)
        self.tool_id = data.get('tool_id', None)
        self.target_frame = data.get('target_frame', None)
        self.beam_already_attached_to_robot = data.get('beam_already_attached_to_robot', None)
        self.tag = data.get('tag', None)

    def create_state_diff(self, process, clear=True):
        # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
        if clear:
            self.state_diff = {}
        self.state_diff[(self.tool_id, 'f')] = self.target_frame
        tool = process.tool(self.tool_id)
        tool.close_gripper()
        self.state_diff[(self.tool_id, 'c')] = tool._get_kinematic_state()
        # Change attachment status of the tool only if the beam is already attached to robot
        if self.beam_already_attached_to_robot:
            self.state_diff[(self.tool_id, 'a')] = True


class RoboticFreeMovement(RoboticMovement):

    def __str__(self):
        return "Free Move to %s" % (self.target_frame)


class RoboticLinearMovement(RoboticMovement):

    def __str__(self):
        return "Linear Move to %s" % (self.target_frame)


class RoboticDigitalOutput(Movement):

    def __init__(self, digital_output=None, tool_id=None, attached_objects=[], operator_stop_before=None, operator_stop_after=None, tag=None, planning_priority=-1):
        # type: (DigitalOutput, str, list[str], str, str, str, int) -> RoboticDigitalOutput
        """ `tool_id` relates to the tool that is being operated.
        `attached_objects` should be filled in for Open or Close Gripper movements that
        involved letting go or picking up a beam or other objects.
        This helps the state manage to figure
        out what objects is picked up and attached or no longer attached.

        State Diff
        ----------

        `DigitalOutput.LockTool` and `CloseGripper` will change the attached status
        of the `attached_objects` to True.

        `DigitalOutput.UnlockTool` and `OpenGripper` will change the attached status
        of the `attached_objects` to False.

        `DigitalOutput.LockTool` and `UnlockTool` will change the attached status of the tool

        `DigitalOutput.CloseGripper` and `OpenGripper` will change the kinematic configuration of the tool
        """
        Movement.__init__(self, operator_stop_before=operator_stop_before,
                          operator_stop_after=operator_stop_after, planning_priority=planning_priority)
        self.digital_output = digital_output
        self.tool_id = tool_id
        self.attached_objects = attached_objects

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
        data['attached_objects'] = self.attached_objects
        return data

    @data.setter
    def data(self, data):
        """ Sub class specific data loaded
        """
        super(RoboticDigitalOutput, type(self)).data.fset(self, data)
        self.digital_output = data['digital_output']
        self.tool_id = data.get('tool_id', None)
        self.attached_objects = data.get('attached_objects', [])

    def create_state_diff(self, process, clear=True):
        # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
        if clear:
            self.state_diff = {}
        tool_id = self.tool_id

        # Changing Tool States
        if self.digital_output == DigitalOutput.LockTool:
            self.state_diff[(tool_id, 'a')] = True

        elif self.digital_output == DigitalOutput.UnlockTool:
            self.state_diff[(tool_id, 'a')] = False

        # OpenGripper and CloseGripper type affects the tool and the beam
        elif self.digital_output == DigitalOutput.OpenGripper:
            gripper = process.tool(self.tool_id)
            gripper.open_gripper()
            self.state_diff[(tool_id, 'c')] = gripper._get_kinematic_state()

        elif self.digital_output == DigitalOutput.CloseGripper:
            gripper = process.tool(self.tool_id)
            gripper.close_gripper()
            self.state_diff[(tool_id, 'c')] = gripper._get_kinematic_state()

        # Changing States for attached objects
        if self.digital_output in [DigitalOutput.LockTool, DigitalOutput.CloseGripper]:
            for object_id in self.attached_objects:
                self.state_diff[(object_id, 'a')] = True
        elif self.digital_output in [DigitalOutput.UnlockTool, DigitalOutput.OpenGripper]:
            for object_id in self.attached_objects:
                self.state_diff[(object_id, 'a')] = False


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

    def create_state_diff(self, process, clear=True):
        # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
        if clear:
            self.state_diff = {}
        for i, clamp_id in enumerate(self.clamp_ids):
            clamp = process.tool(clamp_id)
            clamp.clamp_jaw_position = self.jaw_positions[i]
            self.state_diff[(clamp_id, 'c')] = clamp._get_kinematic_state()


class RoboticClampSyncLinearMovement(RoboticMovement, ClampsJawMovement):

    def __init__(self, target_frame=None, attached_objects=[], t_flange_from_attached_objects=[], jaw_positions=[], clamp_ids=[], planning_priority=1, speed_type="",
                 allowed_collision_matrix=[], tag=None):
        tag = tag or "Robot and Clamp Sync Move"
        RoboticMovement.__init__(self, target_frame, attached_objects, t_flange_from_attached_objects, planning_priority=planning_priority,
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

    def create_state_diff(self, process, clear=True):
        # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
        RoboticMovement.create_state_diff(self, process, clear)
        ClampsJawMovement.create_state_diff(self, process, clear=False)


class RobotScrewdriverSyncLinearMovement(RoboticMovement):
    def __init__(
            self,
            target_frame=None,  # type: Frame
            attached_objects=[],  # type: List[str]
            t_flange_from_attached_objects=[],  # type: List[Transformation]
            screw_positions=[],  # type: List[float]
            screwdriver_ids=[],  # type: List[str]
            planning_priority=1,  # type: int
            speed_type="",  # type: str
            allowed_collision_matrix=[],  # type: List[Tuple[str,str]]
            tag=None  # type: str
    ):
        """Syncronized linear movement between screwdrivers and robot.

        The `attached_objects` and `t_flange_from_attached_objects` lists should contain the
        screwdriver_ids in the `screwdriver_ids`.

        `screw_positions` are relative distances (mm) from the start of the movement.
        Positive means tightening, negative means retracting.

        State_diff
        ----------
        Robot Frame is updated
        All the screwdriver's frame are updated
        """

        RoboticMovement.__init__(
            self,
            target_frame=target_frame,
            attached_objects=attached_objects,
            t_flange_from_attached_objects=t_flange_from_attached_objects,
            planning_priority=planning_priority,
            speed_type=speed_type,
            allowed_collision_matrix=allowed_collision_matrix,
            tag=tag or "Robot and Clamp Sync Move")
        self.screw_positions = screw_positions
        self.screwdriver_ids = screwdriver_ids

    @property
    def data(self):
        """ Sub class specific data added to the dictionary of the parent class
        """
        data = super(RobotScrewdriverSyncLinearMovement, self).data
        data['screw_positions'] = self.screw_positions
        data['screwdriver_ids'] = self.screwdriver_ids
        return data

    @data.setter
    def data(self, data):
        """ Sub class specific data loaded
        """
        super(RobotScrewdriverSyncLinearMovement, type(self)).data.fset(self, data)
        self.screw_positions = data.get('screw_positions', [])
        self.screwdriver_ids = data.get('screwdriver_ids', [])

    def __str__(self):
        return "Robot-Screwdriver Linear Sync Move to %s. Screwdrivers %s Move by %s mm." % (self.target_frame, self.screwdriver_ids, self.screw_positions)

    def create_state_diff(self, process, clear=True):
        # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
        RoboticMovement.create_state_diff(self, process, clear)


class AcquireDockingOffset(Movement):
    def __init__(
        self,
        target_frame=None,  # type: Frame
        tool_id=None,  # type: str
        tag=None  # type: str
    ):
        Movement.__init__(
            self,
            tag=tag or "Acquire Docking Offset"
        )
        self.target_frame = target_frame  # type: Frame
        self.tool_id = tool_id  # type: str

    @property
    def data(self):
        """ Sub class specific data added to the dictionary of the parent class
        """
        data = super(AcquireDockingOffset, self).data
        data['target_frame'] = self.target_frame
        data['tool_id'] = self.tool_id
        return data

    @data.setter
    def data(self, data):
        """ Sub class specific data loaded
        """
        super(AcquireDockingOffset, type(self)).data.fset(self, data)
        self.target_frame = data.get('target_frame', None)
        self.tool_id = data.get('tool_id', None)

    def __str__(self):
        return "Aquire docking offset for Tool ('%s')" % self.tool_id

    def create_state_diff(self, process, clear=True):
        # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
        if clear:
            self.state_diff = {}


class CancelRobotOffset(Movement):
    def __init__(
        self,
        tool_id=None,  # type: str
        tag=None  # type: str
    ):
        Movement.__init__(
            self,
            tag=tag or "Cancel Docking Offset"
        )
        self.tool_id = tool_id  # type: str

    @property
    def data(self):
        """ Sub class specific data added to the dictionary of the parent class
        """
        data = super(CancelRobotOffset, self).data
        data['tool_id'] = self.tool_id
        return data

    @data.setter
    def data(self, data):
        """ Sub class specific data loaded
        """
        super(CancelRobotOffset, type(self)).data.fset(self, data)
        self.tool_id = data.get('tool_id', None)

    def __str__(self):
        return "Cancel docking offset for Tool ('%s')" % self.tool_id

    def create_state_diff(self, process, clear=True):
        # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
        if clear:
            self.state_diff = {}
