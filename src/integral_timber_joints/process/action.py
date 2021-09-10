from itertools import chain

from compas.geometry import Transformation, Translation, add_vectors

from integral_timber_joints.process.movement import *
from integral_timber_joints.tools import Clamp, Gripper, Screwdriver

try:
    from typing import Dict, List, Optional, Tuple, cast

    from integral_timber_joints.process import RobotClampAssemblyProcess
except:
    pass

##############################
# Base Classes for all actions
# Do not use them directly
##############################


class Action(object):
    """ Base class for all actions.
    It contains a list of movements
    """

    def __init__(self, seq_n=0, act_n=0):
        # type: (int, int) -> Action
        self.movements = []  # type: list[Movement]
        self.seq_n = seq_n  # type: int # Zero-based index corelating which Action belongs to which Beam according to the Beam sequence in process.assembly.sequence
        self.act_n = act_n  # type: int # Zero-based index corrisponding to the sequential list of Actions. This can be a unique id for an Action

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """The create_movements() functions are are located within each of the Action child class.

        They copy tool_ids / target frames from the Action classes,
        occationally performing transformation along the tool chain.
        All the information needed to execute a Movement will need to be copied as Movement
        object have to be consummed individually.
        """
        raise NotImplementedError("Action.create_movements() is not implemented by child class")

    def to_data(self):
        """Simpliest way to get this class serialized.
        """
        return self.data

    def assign_movement_ids(self):
        """Called to asign the movement.movement_id in all child movements."""
        for mov_n, movement in enumerate(self.movements):
            movement.movement_id = "A%i_M%i" % (self.act_n, mov_n)

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
            'movements': self.movements,
            'seq_n': self.seq_n,
            'act_n': self.act_n,
        }
        return data

    @data.setter
    def data(self, data):
        self.movements = data['movements']
        self.seq_n = data['seq_n']
        self.act_n = data['act_n']


class OperatorAction(Action):
    """ Base class for actions performed manually.
    Representing actions that may affect the scene, but not necessary produce robotic code.
    However, pause code for operator-confirmation maybe standard result.
    """
    pass


class RobotAction(Action):
    """ Base class for actions performed automatically.
    Representing actions that affect the scene.
    """
    pass


class RobotIOAction(Action):
    """ Base class for actions performed automatically.
    Representing actions that affect the scene.
    """
    pass

#########################################################
# Base Classes for attaching and detaching objects
# Do not use them directly
#########################################################


class AttachToolAction(RobotIOAction):
    """ Base class for actions that involve attaching a tool (gripper or clamp) to robot.
    For example: PickClampAction (can be from storage or from joint), PickGripperAction (from storage)
    """

    def __init__(self, tool_type=None, tool_id=None):
        # type: (str, str) -> None
        self.tool_type = tool_type
        self.tool_id = tool_id

    @property
    def data(self):
        data = super(AttachToolAction, self).data
        data['tool_type'] = self.tool_type
        data['tool_id'] = self.tool_id
        return data

    @data.setter
    def data(self, data):
        super(AttachToolAction, type(self)).data.fset(self, data)
        self.tool_type = data['tool_type']
        self.tool_id = data['tool_id']

    @property
    def _tool_string(self):
        if self.tool_id:
            return "%s ('%s')" % (self.tool_type, self.tool_id)
        else:
            return "%s (?)" % (self.tool_type)


class DetachToolAction(RobotIOAction):
    """ Base class for actions that involve detaching a tool (gripper or clamp) from robot.
    For example: PlaceClampAction (can be to storage or to joint), PlaceGripperAction (to storage)
    """

    def __init__(self, tool_type=None, tool_id=None):
        # type: (str, str) -> None
        self.tool_type = tool_type
        self.tool_id = tool_id

    @property
    def data(self):
        data = super(DetachToolAction, self).data
        data['tool_type'] = self.tool_type
        data['tool_id'] = self.tool_id
        return data

    @data.setter
    def data(self, data):
        super(DetachToolAction, type(self)).data.fset(self, data)
        self.tool_type = data['tool_type']
        self.tool_id = data['tool_id']

    @property
    def _tool_string(self):
        if self.tool_id:
            return "%s ('%s')" % (self.tool_type, self.tool_id)
        else:
            return "%s (?)" % (self.tool_type)


class AttachBeamAction(RobotIOAction):
    """ Base class for actions that involve attaching a workpiece (e.g. beam) to a tool (of the robot).
    For example: PickBeamAction (from storage)
    For path planning purpose, both workpiece and tool is attached to robot.
    """

    def __init__(self, beam_id=None, gripper_id=None):
        # type: (str, str) -> None
        self.beam_id = beam_id
        self.gripper_id = gripper_id

    @property
    def data(self):
        data = super(AttachBeamAction, self).data
        data['beam_id'] = self.beam_id
        data['gripper_id'] = self.gripper_id
        return data

    @data.setter
    def data(self, data):
        super(AttachBeamAction, type(self)).data.fset(self, data)
        self.beam_id = data['beam_id']
        self.gripper_id = data['gripper_id']


class DetachBeamAction(RobotIOAction):
    """ Base class for actions that involve detaching a workpiece (e.g. beam) from a tool (of the robot).
    For example: PlaceBeamWithClampAction, PlaceBeamWithoutClampAction
    """

    def __init__(self, beam_id=None, gripper_id=None):
        # type: (str, str) -> None
        self.beam_id = beam_id
        self.gripper_id = gripper_id

    @property
    def data(self):
        data = super(DetachBeamAction, self).data
        data['beam_id'] = self.beam_id
        data['gripper_id'] = self.gripper_id
        return data

    @data.setter
    def data(self, data):
        super(DetachBeamAction, type(self)).data.fset(self, data)
        self.beam_id = data['beam_id']
        self.gripper_id = data['gripper_id']

##############################
# Manually Performed Actions
##############################


class LoadBeamAction(OperatorAction):
    def __init__(self, seq_n=0, act_n=0, beam_id=None):
        # type: (int, int, str) -> None
        super(LoadBeamAction, self).__init__()
        # OperatorAction.__init__(self)
        self.seq_n = seq_n
        self.act_n = act_n
        self.beam_id = beam_id

    @property
    def data(self):
        data = super(LoadBeamAction, self).data
        data['beam_id'] = self.beam_id
        return data

    @data.setter
    def data(self, data):
        super(LoadBeamAction, type(self)).data.fset(self, data)
        self.beam_id = data['beam_id']

    def __str__(self):
        return "Operator load Beam ('%s') for pickup" % (self.beam_id)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        self.movements = []
        grasp_face = process.assembly.get_beam_attribute(self.beam_id, 'gripper_grasp_face')
        beam_pickup_frame_wcf = process.assembly.get_beam_attribute(self.beam_id, 'assembly_wcf_pickup')

        self.movements.append(OperatorLoadBeamMovement(
            beam_id=self.beam_id,
            grasp_face=grasp_face,
            target_frame=beam_pickup_frame_wcf
        ))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


class OperatorAttachScrewdriverAction(OperatorAction):
    def __init__(self, seq_n=0, act_n=0, beam_id=None, joint_id=None, tool_type=None, tool_id=None, beam_position=None):
        # type: (int, int, str, tuple[str, str], str, str, str) -> None
        super(OperatorAttachScrewdriverAction, self).__init__()
        # OperatorAction.__init__(self)
        self.seq_n = seq_n
        self.act_n = act_n
        self.beam_id = beam_id
        self.joint_id = joint_id
        self.tool_type = tool_type
        self.tool_id = tool_id
        self.beam_position = beam_position

    @property
    def data(self):
        data = super(OperatorAttachScrewdriverAction, self).data
        data['beam_id'] = self.beam_id
        data['joint_id'] = self.joint_id
        data['tool_type'] = self.tool_type
        data['tool_id'] = self.tool_id
        data['beam_position'] = self.beam_position
        return data

    @property
    def _tool_string(self):
        if self.tool_id:
            return "%s ('%s')" % (self.tool_type, self.tool_id)
        else:
            return "%s (?)" % (self.tool_type)

    @property
    def _joint_string(self):
        return "Joint (%s-%s) of Beam (%s)" % (self.joint_id[0], self.joint_id[1], self.beam_id)

    @data.setter
    def data(self, data):
        super(OperatorAttachScrewdriverAction, type(self)).data.fset(self, data)
        self.beam_id = data['beam_id']
        self.joint_id = data['joint_id']
        self.tool_type = data['tool_type']
        self.tool_id = data['tool_id']
        self.beam_position = data['beam_position']

    def __str__(self):
        return "Operator attach %s to %s." % (self._tool_string, self._joint_string)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        self.movements = []
        beam_id = self.beam_id
        joint_id = self.joint_id
        beam = process.assembly.beam(self.beam_id)

        # Obtaining the screwdriver set at assembled and attached state.

        screwdriver = process.get_tool_of_joint(joint_id, 'screwdriver_pickup_attached')
        screwdriver_frame_at_position = screwdriver.current_frame
        # Moving it to whereever the beam location is (self.beam_position)
        # t_BeamFinal_BeamAtPosition = process.assembly.get_beam_transformaion_to(beam_id, self.beam_position)
        # assert t_BeamFinal_BeamAtPosition is not None
        # screwdriver_frame_at_position = screwdriver.current_frame.transformed(t_BeamFinal_BeamAtPosition)

        self.movements.append(OperatorAttachToolMovement(
            beam_id=self.beam_id,
            joint_id=self.joint_id,
            tool_type=self.tool_type,
            tool_id=self.tool_id,
            target_frame=screwdriver_frame_at_position,
            tag="Opeartor Attach %s to %s." % (self._tool_string, self._joint_string),
        ))
        self.assign_movement_ids()

##############################
# Tools From Storage Actions
##############################


class PickToolFromStorageAction(RobotAction, AttachToolAction):
    def __init__(self, seq_n=0, act_n=0, tool_type=None, tool_id=None):
        # type: (int, int, str, str) -> None
        RobotAction.__init__(self)
        AttachToolAction.__init__(self, tool_type, tool_id)
        self.seq_n = seq_n
        self.act_n = act_n

    def __str__(self):
        return "Pick %s from Storage" % (self._tool_string)


class PickGripperFromStorageAction(PickToolFromStorageAction):

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Clamp (and other tools) from Storage
        """
        self.movements = []
        tool = process.tool(self.tool_id)  # type: Gripper
        toolchanger = process.robot_toolchanger
        tool_storage_frame_wcf = tool.tool_storage_frame
        tool_storage_frame_t0cf = toolchanger.set_current_frame_from_tcp(tool_storage_frame_wcf)

        tool_pick_up_frame_wcf = tool.tool_pick_up_frame_in_wcf(tool_storage_frame_wcf)
        tool_pick_up_frame_t0cf = toolchanger.set_current_frame_from_tcp(tool_pick_up_frame_wcf)

        self.movements.append(RoboticFreeMovement(
            target_frame=tool_pick_up_frame_t0cf,
            speed_type='speed.transit.rapid',
            tag="Free Move reach Storage Approach Frame of %s, to get tool." % self._tool_string
        ))  # Tool Storage Approach
        self.movements.append(RoboticLinearMovement(
            target_frame=tool_storage_frame_t0cf,
            speed_type='speed.toolchange.approach.notool',
            target_configuration=tool.tool_storage_configuration,
            tag="Linear Advance to Storage Frame of %s, to get tool." % self._tool_string,
            allowed_collision_matrix=[('tool_changer', self.tool_id)]
        ))  # Tool Storage Final
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.LockTool,
            tool_id=self.tool_id,
            tag="Toolchanger Lock %s" % self._tool_string))
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.OpenGripper,
            tool_id=self.tool_id,
            tag="%s Open Gripper to release itself from storage pad." % self._tool_string
        ))

        tool_env_acm = [(self.tool_id, env_id) for env_id in process.environment_models.keys()]

        tool_pick_up_retract_frame_wcf = tool.tool_pick_up_retract_frame_in_wcf(tool_storage_frame_wcf)
        tool_pick_up_retract_frame_t0cf = toolchanger.set_current_frame_from_tcp(tool_pick_up_retract_frame_wcf)

        self.movements.append(RoboticLinearMovement(
            target_frame=tool_pick_up_retract_frame_t0cf,
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.toolchange.retract.withtool',
            tag="Linear Retract after getting %s from storage." % self._tool_string,
            allowed_collision_matrix=tool_env_acm
        ))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


class PickScrewdriverFromStorageAction(PickToolFromStorageAction):

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for placing Clamp (and other tools) to Storage.
        Modified from PickToolFromStorageAction.create_movements()
        """
        self.movements = []
        tool = process.tool(self.tool_id)  # type: Screwdriver
        toolchanger = process.robot_toolchanger
        tool_storage_frame_wcf = tool.tool_storage_frame

        # * Free Move for Toolchanger to reach Approach Frame of tool storage
        t_world_from_toolbase = Transformation.from_frame(tool_storage_frame_wcf)
        t_gripperbase_from_retractedgripperbase = Translation.from_vector(toolchanger.approach_vector.scaled(-1))
        t_world_retractedgripperbase = t_world_from_toolbase * t_gripperbase_from_retractedgripperbase
        tool_storage_retracted_t0cf = Frame.from_transformation(t_world_retractedgripperbase * toolchanger.t_tcf_from_t0cf)
        self.movements.append(RoboticFreeMovement(
            target_frame=tool_storage_retracted_t0cf,
            speed_type='speed.transit.rapid',
            tag="Free Move for Toolchanger to approach Storage Frame of %s, to pick tool from storage." % self._tool_string
        ))

        # * Linear move into Tool Storage
        t_world_from_toolbase = Transformation.from_frame(tool_storage_frame_wcf)
        tool_storage_frame_t0cf = Frame.from_transformation(t_world_from_toolbase * toolchanger.t_tcf_from_t0cf)
        self.movements.append(RoboticLinearMovement(
            target_frame=tool_storage_frame_t0cf,
            speed_type='speed.toolchange.approach.notool',
            target_configuration=tool.tool_storage_configuration,
            tag="Toolchanger Advance to dock with %s." % self._tool_string,
            allowed_collision_matrix=[('tool_changer', self.tool_id)],
        ))

        # * Lock Toolchanger
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.LockTool,
            tool_id=self.tool_id,
            tag="Toolchanger Lock %s" % self._tool_string))

        # * Linear Retract 1
        t_world_from_toolbase = Transformation.from_frame(tool_storage_frame_wcf)
        t_gripperbase_from_approachgripperbase = Translation.from_vector(tool.storageapproach2_vector.scaled(-1))
        t_world_approachgripperbase = t_world_from_toolbase * t_gripperbase_from_approachgripperbase
        tool_storage_approach2_t0cf = Frame.from_transformation(t_world_approachgripperbase * toolchanger.t_tcf_from_t0cf)

        tool_env_acm = [(self.tool_id, env_id) for env_id in process.environment_models.keys()]
        self.movements.append(RoboticLinearMovement(
            target_frame=tool_storage_approach2_t0cf,
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.toolchange.retract.withtool',
            tag="Linear Retract 1 from Storage Frame of %s, after picking up tool." % self._tool_string,
            allowed_collision_matrix=tool_env_acm,
        ))

        # * Linear Retract 2
        t_world_from_toolbase = Transformation.from_frame(tool_storage_frame_wcf)
        t_gripperbase_from_approachgripperbase = Translation.from_vector(add_vectors(tool.storageapproach1_vector.scaled(-1), tool.storageapproach2_vector.scaled(-1)))
        t_world_approachgripperbase = t_world_from_toolbase * t_gripperbase_from_approachgripperbase
        tool_storage_approach2_t0cf = Frame.from_transformation(t_world_approachgripperbase * toolchanger.t_tcf_from_t0cf)

        tool_env_acm = [(self.tool_id, env_id) for env_id in process.environment_models.keys()]
        self.movements.append(RoboticLinearMovement(
            target_frame=tool_storage_approach2_t0cf,
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.toolchange.retract.withtool',
            tag="Linear Retract 1 from Storage Frame of %s, after picking up tool." % self._tool_string,
            allowed_collision_matrix=tool_env_acm,
        ))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


class PickClampFromStorageAction(PickToolFromStorageAction):

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Clamp (and other tools) from Storage
        """
        self.movements = []
        tool = process.clamp(self.tool_id)  # type: Clamp
        toolchanger = process.robot_toolchanger

        tool_storage_frame_wcf = tool.tool_storage_frame
        tool_storage_frame_t0cf = toolchanger.set_current_frame_from_tcp(tool_storage_frame_wcf)

        tool_pick_up_frame_wcf = tool.tool_pick_up_frame_in_wcf(tool_storage_frame_wcf)
        tool_pick_up_frame_t0cf = toolchanger.set_current_frame_from_tcp(tool_pick_up_frame_wcf)

        # Tool Changer Approach Storage
        self.movements.append(RoboticFreeMovement(
            target_frame=tool_pick_up_frame_t0cf.copy(),
            speed_type='speed.transit.rapid',
            tag="Free Move reach Storage Approach Frame of %s, to get clamp." % self._tool_string
        ))  # Tool Storage Approach
        self.movements.append(RoboticLinearMovement(
            target_frame=tool_storage_frame_t0cf.copy(),
            speed_type='speed.toolchange.approach.notool',
            target_configuration=tool.tool_storage_configuration,
            tag="Linear Advance to Storage Frame of %s, to get tool." % self._tool_string,
            allowed_collision_matrix=[('tool_changer', self.tool_id)]
        ))  # Tool Storage Final
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.LockTool,
            tool_id=self.tool_id,
            tag="Toolchanger Lock %s" % self._tool_string
        ))
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.OpenGripper,
            tool_id=self.tool_id,
            tag="%s Open Gripper to release itself from storage pad." % self._tool_string
        ))
        # Retraction movement
        acm = [(self.tool_id, env_id) for env_id in process.environment_models.keys()]
        tool_storage_retract_frame1_t0cf = toolchanger.set_current_frame_from_tcp(tool.tool_storage_retract_frame1)
        tool_storage_retract_frame2_t0cf = toolchanger.set_current_frame_from_tcp(tool.tool_storage_retract_frame2)
        tool_env_acm = [(self.tool_id, env_id) for env_id in process.environment_models.keys()]

        self.movements.append(RoboticLinearMovement(
            target_frame=tool_storage_retract_frame1_t0cf,
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.toolchange.retract.withtool',
            tag="Linear Retract 1 of 2 after getting %s from storage." % self._tool_string,
            allowed_collision_matrix=tool_env_acm
        ))
        self.movements.append(RoboticLinearMovement(
            target_frame=tool_storage_retract_frame2_t0cf,
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.toolchange.retract.withtool',
            tag="Linear Retract 2 of 2 after getting %s from storage." % self._tool_string,
            allowed_collision_matrix=tool_env_acm
        ))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()

##############################
# Tools To Storage Actions
##############################


class PlaceToolToStorageAction(RobotAction, DetachToolAction):
    def __init__(self, seq_n=0, act_n=0, tool_type=None, tool_id=None):
        # type: (int, int, str, str) -> None
        RobotAction.__init__(self)
        DetachToolAction.__init__(self, tool_type, tool_id)
        self.seq_n = seq_n
        self.act_n = act_n

    def __str__(self):
        if self.tool_id:
            object_str = "%s ('%s')" % (self.tool_type, self.tool_id)
        else:
            object_str = "%s (?)" % (self.tool_type)
        return "Place %s to Storage" % (object_str)


class PlaceGripperToStorageAction(PlaceToolToStorageAction):

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for placing Clamp (and other tools) to Storage.
        Modified from PickToolFromStorageAction.create_movements()
        """
        self.movements = []
        tool = process.tool(self.tool_id)  # type: Clamp
        toolchanger = process.robot_toolchanger
        tool_storage_frame_wcf = tool.tool_storage_frame
        tool_storage_frame_t0cf = toolchanger.set_current_frame_from_tcp(tool_storage_frame_wcf)

        tool_pick_up_retract_frame_wcf = tool.tool_pick_up_retract_frame_in_wcf(tool_storage_frame_wcf)
        tool_pick_up_retract_frame_t0cf = toolchanger.set_current_frame_from_tcp(tool_pick_up_retract_frame_wcf)

        self.movements.append(RoboticFreeMovement(
            target_frame=tool_pick_up_retract_frame_t0cf,
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.transit.rapid',
            tag="Free Move to reach Storage Approach Frame of %s, to place tool in storage." % self._tool_string
        ))  # Tool Storage Approach
        tool_env_acm = [(self.tool_id, env_id) for env_id in process.environment_models.keys()]
        self.movements.append(RoboticLinearMovement(
            target_frame=tool_storage_frame_t0cf,
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.toolchange.approach.withtool',
            target_configuration=tool.tool_storage_configuration,
            tag="Linear Advance to Storage Frame of %s, to place tool in storage." % self._tool_string,
            allowed_collision_matrix=tool_env_acm,
        ))  # Tool Storage Final
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.CloseGripper,
            tool_id=self.tool_id,
            tag="%s Close Gripper to lock onto storage pad." % self._tool_string
        ))
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.UnlockTool,
            tool_id=self.tool_id,
            tag="Toolchanger Unlock %s" % self._tool_string
        ))

        tool_pick_up_frame_wcf = tool.tool_pick_up_frame_in_wcf(tool_storage_frame_wcf)
        tool_pick_up_frame_t0cf = toolchanger.set_current_frame_from_tcp(tool_pick_up_frame_wcf)

        self.movements.append(RoboticLinearMovement(
            target_frame=tool_pick_up_frame_t0cf,
            speed_type='speed.toolchange.retract.notool',
            tag="Linear Retract from storage after placing %s in storage" % self._tool_string,
            allowed_collision_matrix=[('tool_changer', self.tool_id)],
        ))  # Tool Storage Retract

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


class PlaceClampToStorageAction(PlaceToolToStorageAction):

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for placing Clamp to Storage.
        """
        self.movements = []
        tool = process.clamp(self.tool_id)  # type: Clamp
        toolchanger = process.robot_toolchanger

        # Tool approaching storage
        tool_storage_approach_frame1_t0cf = toolchanger.set_current_frame_from_tcp(tool.tool_storage_approach_frame1)
        tool_storage_approach_frame2_t0cf = toolchanger.set_current_frame_from_tcp(tool.tool_storage_approach_frame2)
        self.movements.append(RoboticFreeMovement(
            target_frame=tool_storage_approach_frame1_t0cf,
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.transit.rapid',
            tag="Free Move reach Storage Approach Frame of %s, to place clamp in storage." % self._tool_string
        ))  # Tool Storage Approach

        tool_env_acm = [(self.tool_id, env_id) for env_id in process.environment_models.keys()]
        self.movements.append(RoboticLinearMovement(
            target_frame=tool_storage_approach_frame2_t0cf,
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.toolchange.approach.withtool',
            tag="Linear Approach 1 of 2 to place %s in storage." % self._tool_string,
            allowed_collision_matrix=tool_env_acm
        ))  # Tool Storage Approach

        # Tool go to final
        tool_storage_frame_wcf = tool.tool_storage_frame
        tool_storage_frame_t0cf = toolchanger.set_current_frame_from_tcp(tool_storage_frame_wcf)
        self.movements.append(RoboticLinearMovement(
            target_frame=tool_storage_frame_t0cf,
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.toolchange.approach.withtool',
            target_configuration=tool.tool_storage_configuration,
            tag="Linear Approach 2 of 2 to place %s in storage." % self._tool_string,
            allowed_collision_matrix=tool_env_acm
        ))  # Tool Storage Final
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.CloseGripper,
            tool_id=self.tool_id,
            tag="Close Gripper to lock %s onto storage pad." % self._tool_string
        ))
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.UnlockTool,
            tool_id=self.tool_id,
            tag="Toolchanger Unlock %s" % self._tool_string
        ))

        # Toolchanger retract
        tool_pick_up_frame_wcf = tool.tool_pick_up_frame_in_wcf(tool_storage_frame_wcf)
        tool_pick_up_frame_t0cf = toolchanger.set_current_frame_from_tcp(tool_pick_up_frame_wcf)
        self.movements.append(RoboticLinearMovement(
            target_frame=tool_pick_up_frame_t0cf.copy(),
            speed_type='speed.toolchange.retract.notool',
            tag="Linear Retract from storage after placing %s in storage" % self._tool_string,
            allowed_collision_matrix=[('tool_changer', self.tool_id)]
        ))  # Tool Storage Retract

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


class PlaceScrewdriverToStorageAction(PlaceToolToStorageAction):

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for placing Clamp (and other tools) to Storage.
        Modified from PickToolFromStorageAction.create_movements()
        """
        self.movements = []
        tool = process.tool(self.tool_id)  # type: Screwdriver
        toolchanger = process.robot_toolchanger
        tool_storage_frame_wcf = tool.tool_storage_frame

        t_world_from_toolbase = Transformation.from_frame(tool_storage_frame_wcf)

        # * Free Move to reach Storage Approach Frame
        t_gripperbase_from_approachgripperbase = Translation.from_vector(add_vectors(tool.storageapproach1_vector.scaled(-1), tool.storageapproach2_vector.scaled(-1)))
        t_world_approachgripperbase = t_world_from_toolbase * t_gripperbase_from_approachgripperbase
        tool_storage_approach1_t0cf = Frame.from_transformation(t_world_approachgripperbase * toolchanger.t_tcf_from_t0cf)

        self.movements.append(RoboticFreeMovement(
            target_frame=tool_storage_approach1_t0cf,
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.transit.rapid',
            tag="Free Move to reach Storage Approach Frame of %s, to place tool in storage." % self._tool_string
        ))

        # * Tool Storage Pre Final
        t_gripperbase_from_approachgripperbase = Translation.from_vector(tool.storageapproach2_vector.scaled(-1))
        t_world_approachgripperbase = t_world_from_toolbase * t_gripperbase_from_approachgripperbase
        tool_storage_approach2_t0cf = Frame.from_transformation(t_world_approachgripperbase * toolchanger.t_tcf_from_t0cf)

        tool_env_acm = [(self.tool_id, env_id) for env_id in process.environment_models.keys()]
        self.movements.append(RoboticLinearMovement(
            target_frame=tool_storage_approach2_t0cf,
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.toolchange.approach.withtool',
            tag="Linear Advance 1 to approach Storage Frame of %s, to place tool in storage." % self._tool_string,
            allowed_collision_matrix=tool_env_acm,
        ))

        # * Tool Storage Final
        tool_storage_frame_t0cf = Frame.from_transformation(t_world_from_toolbase * toolchanger.t_tcf_from_t0cf)
        self.movements.append(RoboticLinearMovement(
            target_frame=tool_storage_frame_t0cf.copy(),
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.toolchange.approach.withtool',
            target_configuration=tool.tool_storage_configuration,
            tag="Linear Advance 2 to Storage Frame of %s, to place tool in storage." % self._tool_string,
            allowed_collision_matrix=tool_env_acm,
        ))

        # * Unlock tool
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.UnlockTool,
            tool_id=self.tool_id,
            tag="Toolchanger Unlock %s" % self._tool_string
        ))

        # * Toolchanger Retract after letting go of tool
        t_world_from_toolbase = Transformation.from_frame(tool_storage_frame_wcf)
        t_gripperbase_from_retractedgripperbase = Translation.from_vector(toolchanger.approach_vector.scaled(-1))
        t_world_retractedgripperbase = t_world_from_toolbase * t_gripperbase_from_retractedgripperbase
        tool_storage_retracted_t0cf = Frame.from_transformation(t_world_retractedgripperbase * toolchanger.t_tcf_from_t0cf)
        self.movements.append(RoboticLinearMovement(
            target_frame=tool_storage_retracted_t0cf,
            speed_type='speed.toolchange.retract.notool',
            tag="Linear Retract after placing %s in storage" % self._tool_string,
            allowed_collision_matrix=[('tool_changer', self.tool_id)],
        ))  # Tool Storage Retract

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()

############################################
# Actions for leaving Clamps on Structure
############################################


class PickClampFromStructureAction(RobotAction, AttachToolAction):
    def __init__(self, seq_n=0, act_n=0, joint_id=None, tool_type=None, tool_id=None):
        # type: (int, int, tuple[str, str], str, str) -> None
        RobotAction.__init__(self)
        AttachToolAction.__init__(self, tool_type, tool_id)
        self.seq_n = seq_n
        self.act_n = act_n
        self.joint_id = joint_id

    def __str__(self):
        if self.tool_id:
            object_str = "%s ('%s')" % (self.tool_type, self.tool_id)
        else:
            object_str = "%s (?)" % (self.tool_type)
        location_str = "Joint ('%s-%s')" % (self.joint_id[0], self.joint_id[1])
        return "Pick %s from %s" % (object_str, location_str)

    @property
    def data(self):
        data = super(PickClampFromStructureAction, self).data
        data['joint_id'] = self.joint_id
        return data

    @data.setter
    def data(self, data):
        super(PickClampFromStructureAction, type(self)).data.fset(self, data)
        self.joint_id = data.get('joint_id', None)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Clamp (and other tools) from Storage
        """
        self.movements = []
        tool = process.clamp(self.tool_id)  # type: Clamp
        toolchanger = process.robot_toolchanger

        clamp_wcf_detachapproach = process.get_tool_t0cf_at(self.joint_id, 'clamp_wcf_detachapproach')
        clamp_wcf_final = process.get_tool_t0cf_at(self.joint_id, 'clamp_wcf_final')
        clamp_wcf_detachretract1 = process.get_tool_t0cf_at(self.joint_id, 'clamp_wcf_detachretract1')
        clamp_wcf_detachretract2 = process.get_tool_t0cf_at(self.joint_id, 'clamp_wcf_detachretract2')

        # Approach the clamp at the structure
        self.movements.append(RoboticFreeMovement(
            target_frame=clamp_wcf_detachapproach,
            speed_type='speed.transit.rapid',
            tag="Free Move to reach %s to detach it from structure." % self._tool_string
        ))

        # Toolchanger engaging - confirmation of alignment necessary.
        self.movements.append(RoboticLinearMovement(
            target_frame=clamp_wcf_final,
            speed_type='speed.toolchange.approach.clamp_on_structure',
            tag="Linear Advance to mate toolchanger of %s to detach it from structure." % self._tool_string,
            operator_stop_before="Confirm ToolChanger alignment",
            allowed_collision_matrix=[('tool_changer', self.tool_id)]
        ))
        # Additional ACM between clamp and the attached two beam (at the joint)
        acm = [(self.joint_id[0], self.tool_id), (self.joint_id[1], self.tool_id)]

        # Lock tool and Open Clamp Jaw
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.LockTool,
            tool_id=self.tool_id,
            operator_stop_after="Confirm ToolChanger Locked",
            tag="Toolchanger Lock %s" % self._tool_string
        ))
        self.movements.append(ClampsJawMovement(
            jaw_positions=[process.clamp_appraoch_position],
            clamp_ids=[self.tool_id],
            speed_type='speed.clamp.rapid',
            tag="%s Open Clamp Jaws to be released." % self._tool_string
        ))
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.OpenGripper,
            tool_id=self.tool_id,
            tag="%s Open Gripper to be released from structure." % self._tool_string
        ))

        # Retract
        self.movements.append(RoboticLinearMovement(
            target_frame=clamp_wcf_detachretract1,
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.toolchange.retract.clamp_on_structure',
            tag="Linear Retract 1 of 2 to storage after picking up %s from structure." % self._tool_string,
            allowed_collision_matrix=acm
        ))  # Tool Retract Frame at structure
        self.movements.append(RoboticLinearMovement(
            target_frame=clamp_wcf_detachretract2,
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.toolchange.retract.clamp_on_structure',
            tag="Linear Retract 2 of 2 to storage after picking up %s from structure." % self._tool_string,
            allowed_collision_matrix=acm
        ))  # Tool Retract Frame at structure

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


class PlaceClampToStructureAction(RobotAction, DetachToolAction):
    def __init__(self, seq_n=0, act_n=0, joint_id=None, tool_type=None, tool_id=None):
        # type: (int, int, tuple[str, str], str, str) -> None
        RobotAction.__init__(self)
        DetachToolAction.__init__(self, tool_type, tool_id)
        self.seq_n = seq_n
        self.act_n = act_n
        self.joint_id = joint_id

    def __str__(self):
        if self.tool_id:
            object_str = "%s ('%s')" % (self.tool_type, self.tool_id)
        else:
            object_str = "%s (?)" % (self.tool_type)
        location_str = "Joint ('%s-%s')" % (self.joint_id[0], self.joint_id[1])
        return "Place %s to %s" % (object_str, location_str)

    @property
    def data(self):
        data = super(PlaceClampToStructureAction, self).data
        data['joint_id'] = self.joint_id
        return data

    @data.setter
    def data(self, data):
        super(PlaceClampToStructureAction, type(self)).data.fset(self, data)
        self.joint_id = data.get('joint_id', None)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Clamp (and other tools) from Storage
        """
        self.movements = []
        tool = process.tool(self.tool_id)  # type: Clamp
        toolchanger = process.robot_toolchanger

        clamp_wcf_attachapproach1 = process.get_tool_t0cf_at(self.joint_id, 'clamp_wcf_attachapproach1')
        clamp_wcf_attachapproach2 = process.get_tool_t0cf_at(self.joint_id, 'clamp_wcf_attachapproach2')
        clamp_wcf_final = process.get_tool_t0cf_at(self.joint_id, 'clamp_wcf_final')
        clamp_wcf_attachretract = process.get_tool_t0cf_at(self.joint_id, 'clamp_wcf_attachretract')

        # Approach the clamp at the structure
        self.movements.append(RoboticFreeMovement(
            target_frame=clamp_wcf_attachapproach1,
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.transit.rapid',
            tag="Free Move to bring %s to structure." % self._tool_string
        ))  # Tool Approach Frame where tool is at structure

        # Additional ACM between clamp and the attached two beam (at the joint)
        acm = [(self.joint_id[0], self.tool_id), (self.joint_id[1], self.tool_id)]

        # Two Approach Moves
        self.movements.append(RoboticLinearMovement(
            target_frame=clamp_wcf_attachapproach2,
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.toolchange.approach.clamp_on_structure',
            tag="Linear Approach 1 of 2 to attach %s to structure." % self._tool_string,
            allowed_collision_matrix=acm
        ))  # Tool Approach Frame where tool is at structure
        self.movements.append(RoboticLinearMovement(
            target_frame=clamp_wcf_final,
            attached_objects=[self.tool_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.toolchange.approach.clamp_on_structure',
            tag="Linear Approach 2 of 2 to attach %s to structure." % self._tool_string,
            allowed_collision_matrix=acm
        ))  # Tool Final Frame at structure
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.CloseGripper,
            tool_id=self.tool_id,
            operator_stop_before="Confirm Gripper Pins Alignment",
            operator_stop_after="Confirm Gripper Closed Properly",
            tag="%s Close Gripper and attach to structure." % self._tool_string
        ))
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.UnlockTool,
            tool_id=self.tool_id,
            tag="Toolchanger Unlock %s." % self._tool_string
        ))
        self.movements.append(RoboticLinearMovement(
            target_frame=clamp_wcf_attachretract,
            speed_type='speed.toolchange.retract.notool',
            tag="Linear Retract after attaching %s on structure" % self._tool_string,
            allowed_collision_matrix=[('tool_changer', self.tool_id)],
            operator_stop_after="Confirm Clamp stay sttached"
        ))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


######################################
# Actions for PickingBeam
######################################

class PickBeamWithGripperAction(RobotAction, AttachBeamAction):
    def __init__(self, seq_n=0, act_n=0, beam_id=None, gripper_id=None, additional_attached_objects=[]):
        # type: (int, int, str, str, list[str]) -> None
        """

        This Action can be used for picking up beams with or without flying tools.
        `additional_attached_objects` can only be screwdriver_id

        Both `beam_id` and `additional_attached_objects` will be attached to the robot.
        """
        RobotAction.__init__(self)
        AttachBeamAction.__init__(self, beam_id, gripper_id)
        self.seq_n = seq_n
        self.act_n = act_n
        self.additional_attached_objects = additional_attached_objects

    def __str__(self):
        object_str = "Beam ('%s')" % (self.beam_id)
        location_str = "PickupPoint"
        return "Pick %s from %s using Gripper(%s)" % (object_str, location_str, self.gripper_id)

    @property
    def data(self):
        data = super(PickBeamWithGripperAction, self).data
        data['additional_attached_objects'] = self.additional_attached_objects
        return data

    @data.setter
    def data(self, data):
        super(PickBeamWithGripperAction, type(self)).data.fset(self, data)
        self.additional_attached_objects = data.get('additional_attached_objects', [])

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Beam (with a tool) from Storage
        """
        self.movements = []
        gripper = process.tool(self.gripper_id)  # type: Clamp
        toolchanger = process.robot_toolchanger

        assembly_wcf_pickupapproach = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_pickupapproach')
        assembly_wcf_pickup = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_pickup')
        assembly_wcf_pickupretract = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_pickupretract')
        assert assembly_wcf_pickupapproach is not None and assembly_wcf_pickup is not None and assembly_wcf_pickupretract is not None
        t_gripper_tcf_from_beam = process.assembly.get_t_gripper_tcf_from_beam(self.beam_id)

        # Gripper approach
        self.movements.append(RoboticFreeMovement(
            target_frame=assembly_wcf_pickupapproach,
            attached_objects=[self.gripper_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.transit.rapid',
            tag="Free Move to reach Pickup Approach Frame of Beam ('%s')" % (self.beam_id)
        ))  # Tool Approach Frame where tool is at structure
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.OpenGripper,
            tool_id=self.gripper_id,
            tag="Gripper ('%s') Open Gripper before gripping Beam ('%s')" % (self.gripper_id, self.beam_id)
        ))
        self.movements.append(RoboticLinearMovement(
            target_frame=assembly_wcf_pickup,
            attached_objects=[self.gripper_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.gripper.approach',
            tag="Linear Advance to Storage Frame of Beam ('%s')" % (self.beam_id),
            target_configuration=process.pickup_station.beam_pickup_configuration
        ))  # Tool Final Frame at structure

        # Close Gripper and liftoff
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.CloseGripper,
            tool_id=self.gripper_id,
            attached_objects=[self.beam_id] + self.additional_attached_objects,
            operator_stop_before="Confirm Gripper Ready to close ",
            operator_stop_after="Confirm Grip OK",
            tag="Gripper ('%s') Close Gripper to grip Beam ('%s')" % (self.gripper_id, self.beam_id)))

        # Compute Attached Screwdrives transformations

        gripper = process.get_gripper_of_beam(self.beam_id)
        attached_screwdrivers_id = []
        t_flange_from_attached_screwdrivers = []
        for joint_id in process.assembly.get_joint_ids_with_tools_for_beam(self.beam_id):
            screwdriver = process.get_tool_of_joint(joint_id)
            if screwdriver.name in self.additional_attached_objects:
                attached_screwdrivers_id.append(screwdriver.name)

                f_world_screwdriver_base = screwdriver.current_frame
                f_world_gripper_base = process.get_gripper_of_beam(self.beam_id, 'assembly_wcf_final').current_frame
                t_gripper_base_from_world = Transformation.from_frame(f_world_gripper_base).inverse()
                t_flange_from_attached_screwdrivers.append(toolchanger.t_t0cf_from_tcf * t_gripper_base_from_world * Transformation.from_frame(f_world_screwdriver_base))

        acm = [(self.beam_id, env_id) for env_id in process.environment_models.keys()]
        self.movements.append(RoboticLinearMovement(
            target_frame=assembly_wcf_pickupretract,
            attached_objects=[self.gripper_id, self.beam_id] + attached_screwdrivers_id,
            t_flange_from_attached_objects=[
                toolchanger.t_t0cf_from_tcf,
                toolchanger.t_t0cf_from_tcf * gripper.t_t0cf_from_tcf * t_gripper_tcf_from_beam
            ] + t_flange_from_attached_screwdrivers,
            speed_type='speed.transfer.caution',
            tag="Linear Retract after picking up Beam ('%s')" % (self.beam_id),
            allowed_collision_matrix=acm
        ))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


class PickBeamWithScrewdriverAction(PickBeamWithGripperAction):
    """
    Move beam linear upwards after pickup. Cancel Docking offset.

    - largely copied from `PickBeamWithGripperAction`
    - the difference is the movements it creates is based on the retraction of the screwdriver
    - there is no approach Movement because that is handled by `DockWithScrewdriverAction`

    """

    def __str__(self):
        object_str = "Beam ('%s')" % (self.beam_id)
        location_str = "PickupPoint"
        return "Pick %s from %s using Screwdriver(%s)" % (object_str, location_str, self.gripper_id)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Beam (with a tool) from Storage
        """
        self.movements = []
        tool = process.tool(self.gripper_id)  # type: Clamp
        toolchanger = process.robot_toolchanger

        assembly_wcf_pickupretract = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_pickupretract')
        assert assembly_wcf_pickupretract is not None
        t_gripper_tcf_from_beam = process.assembly.get_t_gripper_tcf_from_beam(self.beam_id)

        # Compute Attached Screwdrives transformations (not including self.gripper_id)
        flying_screwdrivers_id = []
        t_flange_from_attached_screwdrivers = []
        for joint_id in process.assembly.get_joint_ids_with_tools_for_beam(self.beam_id):
            screwdriver = process.get_tool_of_joint(joint_id)
            if screwdriver.name in self.additional_attached_objects and screwdriver.name != self.gripper_id:
                flying_screwdrivers_id.append(screwdriver.name)

                f_world_screwdriver_base = screwdriver.current_frame
                f_world_gripper_base = process.get_gripper_of_beam(self.beam_id, 'assembly_wcf_final').current_frame
                t_gripper_base_from_world = Transformation.from_frame(f_world_gripper_base).inverse()
                t_flange_from_attached_screwdrivers.append(toolchanger.t_t0cf_from_tcf * t_gripper_base_from_world * Transformation.from_frame(f_world_screwdriver_base))

        self.movements.append(RoboticLinearMovement(
            target_frame=assembly_wcf_pickupretract,
            attached_objects=[self.gripper_id, self.beam_id] + flying_screwdrivers_id,
            t_flange_from_attached_objects=[
                toolchanger.t_t0cf_from_tcf,
                toolchanger.t_t0cf_from_tcf * tool.t_t0cf_from_tcf * t_gripper_tcf_from_beam
            ] + t_flange_from_attached_screwdrivers,
            speed_type='speed.transfer.caution',
            tag="Linear Retract after picking up Beam ('%s')" % (self.beam_id)
        ))

        self.movements.append(CancelRobotOffset(
            tool_id=self.gripper_id,
        ))
        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


class GenericGripperApproachBeamPickupAction(RobotAction):
    def __init__(self, seq_n=0, act_n=0, beam_id=None, gripper_id=None):
        # type: (int, int, str, Tuple[str, str],) -> None
        """
        This Action can be used for moving a Gripper/Screwdriver to beam pickup
        """
        RobotAction.__init__(self, seq_n=seq_n, act_n=act_n)
        self.beam_id = beam_id
        self.gripper_id = gripper_id

    def __str__(self):
        object_str = "Beam ('%s')" % (self.beam_id)
        return "Gripper (%s) approaching pickup of %s" % (self.gripper_id, object_str)

    @property
    def data(self):
        data = super(GenericGripperApproachBeamPickupAction, self).data
        data['beam_id'] = self.beam_id
        data['gripper_id'] = self.gripper_id
        return data

    @data.setter
    def data(self, data):
        super(GenericGripperApproachBeamPickupAction, type(self)).data.fset(self, data)
        self.beam_id = data.get('beam_id', None)
        self.gripper_id = data.get('gripper_id', None)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Beam (with a tool) from Storage
        """
        self.movements = []
        gripper = process.tool(self.gripper_id)  # type: Gripper
        toolchanger = process.robot_toolchanger

        # * Free Move for Gripper to reach Approach Frame of beam pickup
        f_beam_pickup_in_wcf = process.assembly.get_beam_attribute(self.beam_id, 'assembly_wcf_pickup')
        t_world_from_beam = Transformation.from_frame(f_beam_pickup_in_wcf)
        t_toolbase_from_robot_flange = toolchanger.t_tcf_from_t0cf
        t_toolbase_from_approaching_toolbase = gripper.t_approach.inverse()
        t_tooltip_from_toolbase = gripper.t_tcf_from_t0cf
        t_beam_from_tooltip = Transformation.from_frame(process.assembly.get_beam_attribute(self.beam_id, 'gripper_tcp_in_ocf'))
        t_world_from_robot_at_pickup_approach = t_world_from_beam * t_beam_from_tooltip * t_tooltip_from_toolbase * t_toolbase_from_approaching_toolbase * t_toolbase_from_robot_flange
        self.movements.append(RoboticFreeMovement(
            target_frame=Frame.from_transformation(t_world_from_robot_at_pickup_approach),
            attached_objects=[self.gripper_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.transit.rapid',
            tag="Free Move to reach Pickup Approach Frame of Beam ('%s')" % (self.beam_id)
        ))  # Tool Approach Frame where tool is at structure

        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.OpenGripper,
            tool_id=self.gripper_id,
            tag="Gripper ('%s') Open Gripper before gripping Beam ('%s')" % (self.gripper_id, self.beam_id)
        ))

        # * Linear Move for Gripper to reach Approach Frame of beam pickup
        t_world_from_robot_at_pickup = t_world_from_beam * t_beam_from_tooltip * t_tooltip_from_toolbase * t_toolbase_from_robot_flange
        tool_env_acm = [(self.gripper_id, env_id) for env_id in process.environment_models.keys()]
        self.movements.append(RoboticLinearMovement(
            target_frame=Frame.from_transformation(t_world_from_robot_at_pickup),
            attached_objects=[self.gripper_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.gripper.approach',
            tag="Linear Advance to Storage Frame of Beam ('%s')" % (self.beam_id),
            target_configuration=process.pickup_station.beam_pickup_configuration,
            allowed_collision_matrix=tool_env_acm,
        ))  # Tool Final Frame at structure

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


class GenericFreeMoveBeamWithGripperAction(RobotAction):
    def __init__(self, seq_n=0, act_n=0, beam_id=None, gripper_id=None, beam_position=None):
        # type: (int, int, str, Tuple[str, str], str) -> None
        """
        This Action can be used for moving a Gripper/Screwdriver to beam pickup
        """
        RobotAction.__init__(self, seq_n=seq_n, act_n=act_n)
        self.beam_id = beam_id
        self.gripper_id = gripper_id
        self.beam_position = beam_position

    def __str__(self):
        object_str = "Beam ('%s')" % (self.beam_id)
        return "Reorient %s using Gripper (%s)" % (object_str, self.gripper_id)

    @property
    def data(self):
        data = super(GenericGripperApproachBeamPickupAction, self).data
        data['beam_id'] = self.beam_id
        data['gripper_id'] = self.gripper_id
        data['beam_position'] = self.beam_position
        return data

    @data.setter
    def data(self, data):
        super(GenericGripperApproachBeamPickupAction, type(self)).data.fset(self, data)
        self.beam_id = data.get('beam_id', None)
        self.gripper_id = data.get('gripper_id', None)
        self.beam_position = data.get('beam_position', None)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Beam (with a tool) from Storage
        """
        self.movements = []
        gripper = process.tool(self.gripper_id)  # type: Gripper
        toolchanger = process.robot_toolchanger

        # * Free Move for Gripper to reach Approach Frame of beam pickup
        f_beam_pickup_in_wcf = process.assembly.get_beam_attribute(self.beam_id, self.beam_position)
        if f_beam_pickup_in_wcf is None:
            print("Beam Attribute %s not available for Beam (%s)" % (self.beam_position, self.beam_id))
        t_world_from_beam = Transformation.from_frame(f_beam_pickup_in_wcf)
        t_toolbase_from_robot_flange = toolchanger.t_tcf_from_t0cf
        t_tooltip_from_toolbase = gripper.t_tcf_from_t0cf
        t_beam_from_tooltip = Transformation.from_frame(process.assembly.get_beam_attribute(self.beam_id, 'gripper_tcp_in_ocf'))
        t_world_from_robot_at_pickup_approach = t_world_from_beam * t_beam_from_tooltip * t_tooltip_from_toolbase * t_toolbase_from_robot_flange
        self.movements.append(RoboticFreeMovement(
            target_frame=Frame.from_transformation(t_world_from_robot_at_pickup_approach),
            attached_objects=[self.gripper_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.transit.rapid',
            tag="Free Move to reorient Beam ('%s')" % (self.beam_id)
        ))  # Tool Approach Frame where tool is at structure

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()

class CloseGripperOnBeamAction(AttachBeamAction):
    def __init__(self, seq_n=0, act_n=0, beam_id=None, gripper_id=None, additional_attached_objects=[]):
        # type: (int, int, str, str, list[str]) -> None
        """
        This Action can be used for picking up beams with or without flying tools.
        Only a single close Gripper Movement is created.
        `additional_attached_objects` can only be screwdriver_id

        Both `beam_id` and `additional_attached_objects` will be attached to the robot.
        """
        RobotIOAction.__init__(self)
        AttachBeamAction.__init__(self, beam_id, gripper_id)
        self.seq_n = seq_n
        self.act_n = act_n
        self.additional_attached_objects = additional_attached_objects

    def __str__(self):
        object_str = "Beam ('%s')" % (self.beam_id)
        return "Gripper (%s) close gripper on %s" % (self.gripper_id, object_str)

    @property
    def data(self):
        data = super(PickBeamWithGripperAction, self).data
        data['additional_attached_objects'] = self.additional_attached_objects
        return data

    @data.setter
    def data(self, data):
        super(PickBeamWithGripperAction, type(self)).data.fset(self, data)
        self.additional_attached_objects = data.get('additional_attached_objects', [])

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Beam (with a tool) from Storage
        """
        self.movements = []

        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.CloseGripper,
            tool_id=self.gripper_id,
            attached_objects=[self.beam_id] + self.additional_attached_objects,
            operator_stop_before="Confirm Gripper Ready to close ",
            operator_stop_after="Confirm Grip OK",
            tag="Gripper ('%s') Close Gripper to grip Beam ('%s')" % (self.gripper_id, self.beam_id)))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


######################################
# Actions for Placing Beam
######################################

class BeamPlacementWithoutClampsAction(RobotAction, DetachBeamAction):
    def __init__(self, seq_n=0, act_n=0, beam_id=None, gripper_id=None):
        # type: (int, int, tuple[str, str], str) -> None
        RobotAction.__init__(self)
        DetachBeamAction.__init__(self, beam_id, gripper_id)
        self.seq_n = seq_n
        self.act_n = act_n

    def __str__(self):
        object_str = "Beam ('%s')" % (self.beam_id)
        location_str = "final location without clamps"
        return "Place %s to %s" % (object_str, location_str)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Beam (with a tool) from Storage
        """
        self.movements = []
        tool = process.tool(self.gripper_id)  # type: Clamp
        toolchanger = process.robot_toolchanger

        assembly_wcf_inclamp = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_inclamp')
        assembly_wcf_final = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_final')
        assembly_wcf_finalretract = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_finalretract')
        assert assembly_wcf_inclamp is not None and assembly_wcf_final is not None and assembly_wcf_finalretract is not None
        t_gripper_tcf_from_beam = process.assembly.get_t_gripper_tcf_from_beam(self.beam_id)

        self.movements.append(RoboticFreeMovement(
            target_frame=assembly_wcf_inclamp,
            attached_objects=[self.gripper_id, self.beam_id],
            t_flange_from_attached_objects=[
                toolchanger.t_t0cf_from_tcf,
                toolchanger.t_t0cf_from_tcf * tool.t_t0cf_from_tcf * t_gripper_tcf_from_beam,
            ],
            speed_type='speed.transfer.rapid',
            tag="Free Move to bring Beam ('%s') to final location" % self.beam_id
        ))

        # Additional ACM between the attached beam, clamps and the neighbouring beams
        neighbour_beam_ids = process.assembly.get_already_built_neighbors(self.beam_id)
        acm = [(self.beam_id, nbr_id) for nbr_id in neighbour_beam_ids] + \
              [(self.beam_id, env_id) for env_id in process.environment_models.keys()]

        # Assembly linear move to final location
        self.movements.append(RoboticLinearMovement(
            target_frame=assembly_wcf_final,
            attached_objects=[self.gripper_id, self.beam_id],
            t_flange_from_attached_objects=[
                toolchanger.t_t0cf_from_tcf,
                toolchanger.t_t0cf_from_tcf * tool.t_t0cf_from_tcf * t_gripper_tcf_from_beam,
            ],
            planning_priority=1,
            speed_type='speed.assembly.noclamp',
            tag="Linear Advance to Final Frame of Beam ('%s')" % (self.beam_id),
            allowed_collision_matrix=acm
        ))
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.OpenGripper,
            tool_id=self.gripper_id,
            attached_objects=[self.beam_id],
            operator_stop_before="Confirm Beam Temporary Support In Place",
            operator_stop_after="Confirm Gripper Cleared Beam",
            tag="Open Gripper ('%s') and let go of Beam ('%s')" % (self.gripper_id, self.beam_id)
        ))
        self.movements.append(RoboticLinearMovement(
            target_frame=assembly_wcf_finalretract,
            attached_objects=[self.gripper_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.gripper.retract',
            tag="Linear retract after placing Beam ('%s')" % self.beam_id,
        ))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


class BeamPlacementWithClampsAction(RobotAction, DetachBeamAction):
    def __init__(self, seq_n=0, act_n=0, beam_id=None, joint_ids=[], gripper_id=None, clamp_ids=None):
        # type: (int, int, str, list[tuple[str, str]], str, str) -> None
        RobotAction.__init__(self)
        DetachBeamAction.__init__(self, beam_id, gripper_id)
        self.seq_n = seq_n
        self.act_n = act_n
        self.joint_ids = joint_ids  # type: list[tuple[str, str]]
        if clamp_ids is None:
            self.clamp_ids = [None for _ in joint_ids]  # type: list[str]
        else:
            self.clamp_ids = clamp_ids

    @property
    def data(self):
        data = super(BeamPlacementWithClampsAction, self).data
        data['joint_ids'] = self.joint_ids
        data['clamp_ids'] = self.clamp_ids
        return data

    @data.setter
    def data(self, data):
        super(BeamPlacementWithClampsAction, type(self)).data.fset(self, data)
        self.joint_ids = data.get('joint_ids', [])
        self.clamp_ids = data.get('clamp_ids', [])

    def __str__(self):
        object_str = "Beam ('%s')" % (self.beam_id)
        joint_str = ["%s-%s" % (joint_id[0], joint_id[1]) for joint_id in self.joint_ids]
        location_str = "final location with clamps at %s" % joint_str
        clamp_id_str = [("?" if clamp_id is None else clamp_id) for clamp_id in self.clamp_ids]
        return "Place %s to Joint %s using Clamp %s" % (object_str, location_str, clamp_id_str)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Beam (with a tool) from Storage
        """
        self.movements = []
        tool = process.tool(self.gripper_id)  # type: Clamp
        toolchanger = process.robot_toolchanger

        assembly_wcf_inclampapproach = process.get_gripper_t0cp_for_beam_at(
            self.beam_id, 'assembly_wcf_inclampapproach')
        assembly_wcf_inclamp = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_inclamp')
        assembly_wcf_final = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_final')
        assembly_wcf_finalretract = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_finalretract')
        assert assembly_wcf_inclampapproach is not None and assembly_wcf_inclamp is not None and assembly_wcf_final is not None and assembly_wcf_finalretract is not None
        t_gripper_tcf_from_beam = process.assembly.get_t_gripper_tcf_from_beam(self.beam_id)

        # Transfer beam to the clamp(s)
        self.movements.append(RoboticFreeMovement(
            target_frame=assembly_wcf_inclampapproach,
            attached_objects=[self.gripper_id, self.beam_id],
            t_flange_from_attached_objects=[
                toolchanger.t_t0cf_from_tcf,
                toolchanger.t_t0cf_from_tcf * tool.t_t0cf_from_tcf * t_gripper_tcf_from_beam,
            ],
            speed_type='speed.transfer.rapid',
            tag="Free Move to bring Beam ('%s') to approach clamps on structure." % self.beam_id
        ))

        acm = [(self.beam_id, clamp_id) for clamp_id in self.clamp_ids]
        # Linear movement into clamp jaws
        self.movements.append(RoboticLinearMovement(
            target_frame=assembly_wcf_inclamp.copy(),
            attached_objects=[self.gripper_id, self.beam_id],
            t_flange_from_attached_objects=[
                toolchanger.t_t0cf_from_tcf,
                toolchanger.t_t0cf_from_tcf * tool.t_t0cf_from_tcf * t_gripper_tcf_from_beam,
            ],
            speed_type='speed.assembly.inclamp',
            tag="Linear Advance to bring Beam ('%s') into clamp jaws" % self.beam_id,
            allowed_collision_matrix=acm))
        self.movements.append(ClampsJawMovement(
            [process.clamp_inclamp_position] * len(self.clamp_ids),
            self.clamp_ids,
            speed_type='speed.clamp.rapid',
            tag="Clamps (%s) close slightly to touch Beam ('%s')" % (self.clamp_ids, self.beam_id)
        ))  # Extend the clamp arm

        # Additional ACM between the attached beam, clamps and the neighbouring beams
        neighbour_beam_ids = process.assembly.get_already_built_neighbors(self.beam_id)
        acm = [(self.beam_id, nbr_id) for nbr_id in chain(self.clamp_ids, neighbour_beam_ids)] + \
              [(self.beam_id, env_id) for env_id in process.environment_models.keys()] + \
              [(self.gripper_id, clamp_id) for clamp_id in self.clamp_ids]  # this is a hack for the planner to work because it cannot actuate the clamp jaw while planning.

        # Robot Clamp Sync Move to final location
        target_frame = assembly_wcf_final.copy()
        jaw_positions, clamp_ids = [process.clamp_final_position] * len(self.clamp_ids), self.clamp_ids
        self.movements.append(RoboticClampSyncLinearMovement(
            target_frame=target_frame,
            attached_objects=[self.gripper_id, self.beam_id],
            t_flange_from_attached_objects=[
                toolchanger.t_t0cf_from_tcf,
                toolchanger.t_t0cf_from_tcf * tool.t_t0cf_from_tcf * t_gripper_tcf_from_beam,
            ],
            jaw_positions=jaw_positions,
            clamp_ids=clamp_ids,
            planning_priority=1,
            speed_type='speed.assembly.clamping',
            tag="Robot and Clamps (%s) syncronously move to clamp Beam ('%s')" % (self.clamp_ids, self.beam_id),
            allowed_collision_matrix=acm
        ))

        # Open gripper and retract
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.OpenGripper,
            tool_id=self.gripper_id,
            attached_objects=[self.beam_id],
            operator_stop_before="Confirm Beam Is Stable",
            operator_stop_after="Confirm Gripper Cleared Beam",
            tag="Open Gripper ('%s') and let go of Beam ('%s')" % (self.gripper_id, self.beam_id)
        ))
        self.movements.append(RoboticLinearMovement(
            target_frame=assembly_wcf_finalretract,
            attached_objects=[self.gripper_id],
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf],
            speed_type='speed.gripper.retract',
            tag="Linear retract after placing Beam ('%s')" % self.beam_id,
        ))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


class AssembleBeamWithScrewdriversAction(RobotAction):
    """Action to Place Beam with flying Screwdrivers.
    Screw it with Screwdriver, not including retract
    (The gripper used can be Scrwederiver or Gripper.)
    """

    def __init__(self, seq_n=0, act_n=0, beam_id=None, joint_ids=[], gripper_id=None, screwdriver_ids=[]):
        """If a screrwdriver is used as the gripper.
        fill in the screwdriver's id into `gripper_id` and also include it in the list of `screwdriver_id`

        list of `joint_ids` need to match with `screwdriver_ids`.
        """
        RobotAction.__init__(self, seq_n, act_n)
        self.beam_id = beam_id  # type: str
        self.joint_ids = joint_ids  # type: list[tuple[str, str]]
        self.gripper_id = gripper_id  # type: list[str]

        # Maintaining same list length between screwdriver_ids and joint_ids
        if screwdriver_ids is []:
            self.screwdriver_ids = [None for _ in joint_ids]  # type: list[str]
        else:
            self.screwdriver_ids = screwdriver_ids  # type: list[str]

    @property
    def data(self):
        data = super(AssembleBeamWithScrewdriversAction, self).data
        data['beam_id'] = self.beam_id
        data['joint_ids'] = self.joint_ids
        data['gripper_id'] = self.gripper_id
        data['screwdriver_ids'] = self.screwdriver_ids
        return data

    @property
    def screwdriver_ids_without_gripper(self):
        return list(set(self.screwdriver_ids) - set([self.gripper_id]))

    @data.setter
    def data(self, data):
        super(AssembleBeamWithScrewdriversAction, type(self)).data.fset(self, data)
        self.joint_ids = data.get('joint_ids', [])
        self.clamp_ids = data.get('clamp_ids', [])

    def __str__(self):
        object_str = "Beam ('%s')" % (self.beam_id)
        joint_str = ["%s-%s" % (joint_id[0], joint_id[1]) for joint_id in self.joint_ids]
        location_str = "final location with clamps at %s" % joint_str
        clamp_id_str = [("?" if clamp_id is None else clamp_id) for clamp_id in self.screwdriver_ids]
        return "Place %s to Joint %s using Clamp %s" % (object_str, location_str, clamp_id_str)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Beam (with a tool) from Storage
        """
        self.movements = []
        gripper = process.tool(self.gripper_id)  # type: Clamp
        toolchanger = process.robot_toolchanger

        # Robot Flange Target frames for the Movements
        assembly_wcf_assembleapproach = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_assembleapproach')
        assembly_wcf_assemblebegin = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_assemblebegin')
        assembly_wcf_final = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_final')

        # * Compute Beam Grasp
        t_gripper_tcf_from_beam = process.assembly.get_t_gripper_tcf_from_beam(self.beam_id)
        # * Compute Gripper(s) Grasp
        t_flange_from_beam = toolchanger.t_t0cf_from_tcf * gripper.t_t0cf_from_tcf * t_gripper_tcf_from_beam

        # * Compute the grasp of attached screwdrivers (Not including the gripper)
        t_flange_from_attached_screwdrivers = []
        for screwdriver_id in self.screwdriver_ids_without_gripper:
            joint_id = self.joint_ids[self.screwdriver_ids.index(screwdriver_id)]

            f_world_gripper_base = process.get_gripper_of_beam(self.beam_id, 'assembly_wcf_final').current_frame
            t_gripper_base_from_world = Transformation.from_frame(f_world_gripper_base).inverse()
            f_world_screwdriver_base = process.assembly.get_joint_attribute(joint_id, 'screwdriver_assembled_attached')
            t_flange_from_attached_screwdrivers.append(toolchanger.t_t0cf_from_tcf * t_gripper_base_from_world * Transformation.from_frame(f_world_screwdriver_base))

        # Derive linear move amount
        sync_linear_move_amount = assembly_wcf_assemblebegin.point.distance_to_point(assembly_wcf_final.point)

        # Rapid free move to transfer beam to the `assembly_wcf_assembleapproach`
        self.movements.append(RoboticFreeMovement(
            target_frame=assembly_wcf_assembleapproach,
            attached_objects=[self.gripper_id, self.beam_id] + self.screwdriver_ids_without_gripper,
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf, t_flange_from_beam] + t_flange_from_attached_screwdrivers,
            speed_type='speed.transfer.rapid',
            tag="Free Move to bring Beam ('%s') to assemble_approach position on structure." % self.beam_id
        ))

        # Slow linear movement to the `assembly_wcf_assemblebegin`
        neighbour_beam_ids = [joint_id[0] for joint_id in self.joint_ids]
        acm = [(nbr_id, tool_id) for tool_id, nbr_id in zip(self.screwdriver_ids, neighbour_beam_ids)]  # Between beam and all screwdrivers

        self.movements.append(RoboticLinearMovement(
            target_frame=assembly_wcf_assemblebegin,
            attached_objects=[self.gripper_id, self.beam_id] + self.screwdriver_ids_without_gripper,
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf, t_flange_from_beam] + t_flange_from_attached_screwdrivers,
            speed_type='speed.assembly.inclamp',
            tag="Linear Advance to bring Screwdriver tips to touch the predrilled hole.",
            allowed_collision_matrix=acm))

        # Additional ACM when the screwdriver touches the target beams
        neighbour_beam_ids = [joint_id[0] for joint_id in self.joint_ids]

        acm = [(self.beam_id, tool_id) for tool_id in self.screwdriver_ids]  # Between beam and all screwdrivers
        acm.extend([(nbr_id, tool_id) for tool_id, nbr_id in zip(self.screwdriver_ids, neighbour_beam_ids)])  # Between screwdrivers and their target beam
        acm.extend(self.joint_ids)  # Between beam and neighbouring beam
        acm.extend([(self.beam_id, env_id) for env_id in process.environment_models.keys()])

        # Robot Clamp Sync Move to final location
        self.movements.append(RobotScrewdriverSyncLinearMovement(
            target_frame=assembly_wcf_final,
            attached_objects=[self.gripper_id, self.beam_id] + self.screwdriver_ids_without_gripper,
            t_flange_from_attached_objects=[toolchanger.t_t0cf_from_tcf, t_flange_from_beam] + t_flange_from_attached_screwdrivers,
            screw_positions=[sync_linear_move_amount] * len(self.screwdriver_ids),
            screwdriver_ids=self.screwdriver_ids,
            planning_priority=1,
            speed_type='speed.assembly.screwing',
            tag="Robot and Screwdrivers (%s) syncronously move to screw Beam ('%s')" % (self.screwdriver_ids, self.beam_id),
            allowed_collision_matrix=acm
        ))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


class RetractGripperFromBeamAction(RobotAction, DetachBeamAction):
    """Open the gripper and Retract the gripper from the beam."""

    def __init__(self, seq_n=0, act_n=0, beam_id=None, gripper_id=None, additional_attached_objects=[]):
        # type: (int, int, str, str, list[str]) -> None
        """
        Both `beam_id` and `additional_attached_objects` will be detached from the robot.
        """
        RobotAction.__init__(self, seq_n=seq_n, act_n=act_n)
        DetachBeamAction.__init__(self, beam_id=beam_id, gripper_id=gripper_id)
        self.additional_attached_objects = additional_attached_objects

    def __str__(self):
        tool_str = "Gripper ('%s')" % (self.gripper_id)
        object_str = "Beam ('%s')" % (self.beam_id)
        return "Retract %s from %s" % (tool_str, object_str)

    @property
    def data(self):
        data = super(RetractGripperFromBeamAction, self).data
        data['additional_attached_objects'] = self.additional_attached_objects
        return data

    @data.setter
    def data(self, data):
        super(RetractGripperFromBeamAction, type(self)).data.fset(self, data)
        self.additional_attached_objects = data.get('additional_attached_objects', [])

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Beam (with a tool) from Storage
        """
        self.movements = []

        assembly_wcf_finalretract = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_finalretract')
        assert assembly_wcf_finalretract is not None

        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.OpenGripper,
            tool_id=self.gripper_id,
            attached_objects=[self.beam_id] + self.additional_attached_objects,
            operator_stop_before="Confirm Beam Temporary Support In Place",
            operator_stop_after="Confirm Gripper Cleared Beam",
            tag="Open Gripper ('%s') and let go of Beam ('%s')" % (self.gripper_id, self.beam_id)
        ))
        self.movements.append(RoboticLinearMovement(
            target_frame=assembly_wcf_finalretract,
            attached_objects=[self.gripper_id],
            t_flange_from_attached_objects=[process.robot_toolchanger.t_t0cf_from_tcf],
            speed_type='speed.gripper.retract',
            tag="Linear retract after placing Beam ('%s')" % self.beam_id,
        ))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


class RetractScrewdriverFromBeamAction(RobotAction, DetachBeamAction,):
    """Open the Screwdriver gripper, retract main screw syncronously with robot.
    Pull out slightly further afterwards. Cancel docking offset."""

    def __init__(self, seq_n=0, act_n=0, beam_id=None, joint_id=None, tool_id=None, additional_attached_objects=[]):
        # type: (int, int, str, Tuple[str,str],  str, list[str]) -> None
        """
        Both `beam_id` and `additional_attached_objects` will be detached from the robot.
        """
        RobotAction.__init__(self, seq_n=seq_n, act_n=act_n)
        DetachBeamAction.__init__(self, beam_id=beam_id, gripper_id=tool_id)
        self.joint_id = joint_id
        self.additional_attached_objects = additional_attached_objects

    def __str__(self):
        tool_str = "Screwdriver ('%s')" % (self.gripper_id)
        object_str = "Beam ('%s')" % (self.beam_id)
        return "Detach and Retract %s from %s" % (tool_str, object_str)

    @property
    def data(self):
        data = super(RetractScrewdriverFromBeamAction, self).data
        data['joint_id'] = self.joint_id
        data['additional_attached_objects'] = self.additional_attached_objects
        return data

    @data.setter
    def data(self, data):
        super(RetractScrewdriverFromBeamAction, type(self)).data.fset(self, data)
        self.joint_id = data.get('joint_id', None)
        self.additional_attached_objects = data.get('additional_attached_objects', [])

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Beam (with a tool) from Storage
        """
        self.movements = []

        screwdriver_assembled_attached = process.get_tool_t0cf_at(self.joint_id, 'screwdriver_assembled_attached')
        screwdriver_assembled_retracted = process.get_tool_t0cf_at(self.joint_id, 'screwdriver_assembled_retracted')
        screwdriver_assembled_retractedfurther = process.get_tool_t0cf_at(self.joint_id, 'screwdriver_assembled_retractedfurther')

        assert screwdriver_assembled_retracted is not None and screwdriver_assembled_retractedfurther is not None

        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.OpenGripper,
            tool_id=self.gripper_id,
            attached_objects=[self.beam_id] + self.additional_attached_objects,
            operator_stop_before="Confirm Beam Temporary Support In Place",
            operator_stop_after="Confirm Gripper Cleared Beam",
            tag="Detach Screwdriver ('%s') from Beam ('%s')" % (self.gripper_id, self.beam_id)
        ))

        sync_linear_move_amount = screwdriver_assembled_attached.point.distance_to_point(screwdriver_assembled_retracted.point)

        # Robot Clamp Sync Move to retract back to begin_assemble location
        self.movements.append(RobotScrewdriverSyncLinearMovement(
            target_frame=screwdriver_assembled_retracted,
            attached_objects=[self.gripper_id],
            t_flange_from_attached_objects=[process.robot_toolchanger.t_t0cf_from_tcf],
            screw_positions=[-1.0 * sync_linear_move_amount],
            screwdriver_ids=[self.gripper_id],
            planning_priority=1,
            speed_type='speed.assembly.screwing',
            tag="Robot and Screwdriver (%s) syncronously move to retract from Beam ('%s')" % (self.gripper_id, self.beam_id),
            allowed_collision_matrix=[(self.gripper_id, self.joint_id[0]), (self.gripper_id, self.joint_id[1])]
        ))

        self.movements.append(RoboticLinearMovement(
            target_frame=screwdriver_assembled_retractedfurther,
            attached_objects=[self.gripper_id],
            t_flange_from_attached_objects=[process.robot_toolchanger.t_t0cf_from_tcf],
            speed_type='speed.gripper.retract',
            tag="Linear retract after placing Beam ('%s')" % self.beam_id,
        ))

        self.movements.append(CancelRobotOffset(
            tool_id=self.gripper_id,
        ))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


##################
# Visual Docking
##################


class DockWithScrewdriverAction(RobotAction, AttachToolAction):
    """
    This docking action includes:
    - TC approaching the target tool
    - visually getting offset
    - small linear movement to align
    - linear movement to dock TC
    - Finally lock tool to TC.
    """

    def __init__(self, seq_n=0, act_n=0, joint_id=None, tool_position=None, tool_type=None, tool_id=None, additional_attached_objects=[]):
        # type: (int, int, str, tuple[str, str], str, str, list[str]) -> None
        """
        This docking action includes:
        - TC approaching the target tool (tool_id) (location retrived from `tool_position` from specific `joint_id`)
        - visually getting offset
        - small linear movement to align
        - linear movement to dock TC
        - Finally lock tool to TC.


        The DigitalOutput.LockTool Movement will only attach the tool_id to robot.
        If the `beam` or other objects had to be attached, it should be included in the `additional_attached_objects`.
        """
        RobotAction.__init__(self)
        AttachToolAction.__init__(self, tool_type, tool_id)
        self.seq_n = seq_n
        self.act_n = act_n
        self.joint_id = joint_id
        self.tool_position = tool_position
        self.additional_attached_objects = additional_attached_objects

    @property
    def _beam_id(self):
        # The screwdriver is always attached to the unfinished side of structure.
        return self.joint_id[1]

    @property
    def _beam_string(self):
        return "Beam ('%s')" % (self._beam_id)

    def __str__(self):
        return "Dock visually with Screwdriver %s on %s in %s" % (self._tool_string, self._beam_string, self.tool_position)

    @property
    def data(self):
        data = super(DockWithScrewdriverAction, self).data
        data['joint_id'] = self.joint_id
        data['tool_position'] = self.tool_position
        data['additional_attached_objects'] = self.additional_attached_objects
        return data

    @data.setter
    def data(self, data):
        super(DockWithScrewdriverAction, type(self)).data.fset(self, data)
        self.joint_id = data.get('joint_id', None)
        self.tool_position = data.get('tool_position', None)
        self.additional_attached_objects = data.get('additional_attached_objects', [])

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for docking with Screwdriver that is already attached on a beam.
        """
        self.movements = []
        tool = process.screwdriver(self.tool_id)  # type: Screwdriver
        toolchanger = process.robot_toolchanger
        beam_id = self.joint_id[1]  # type: str

        # * Transformations
        # Tool positions (`self.tool_position`) should be precalculated before this point, such as: `screwdriver_pickup_attached` or `screwdriver_assembled_attached`
        f_screwdriver_base = process.assembly.get_joint_attribute(self.joint_id, self.tool_position)
        if f_screwdriver_base is None:
            print("Cannot retrive %s from joint (%s)attribute" % (self.tool_position, self.joint_id))
            raise ValueError("Cannot retrive %s from joint (%s)attribute" % (self.tool_position, self.joint_id))
        t_world_toolbase = Transformation.from_frame(f_screwdriver_base)
        t_toolbase_from_robot_flange = toolchanger.t_tcf_from_t0cf
        t_toolbase_from_approaching_toolbase = toolchanger.t_approach.inverse()

        # * Main transformation formular
        # t_world_from_beam = Transformation.from_frame(process.assembly.get_beam_attribute(self._beam_id, 'assembly_wcf_pickup'))
        # t_world_from_robot_at_docked = t_world_from_beam * t_beam_from_tooltip * t_tooltip_from_toolbase * t_toolbase_from_robot_flange
        # t_world_from_robot_at_approach = t_world_from_beam * t_beam_from_tooltip * t_tooltip_from_toolbase * t_toolbase_from_approaching_toolbase * t_toolbase_from_robot_flange

        t_world_from_robot_at_docked = t_world_toolbase * t_toolbase_from_robot_flange
        t_world_from_robot_at_approach = t_world_toolbase * t_toolbase_from_approaching_toolbase * t_toolbase_from_robot_flange

        # Approach the clamp at the structure
        self.movements.append(RoboticFreeMovement(
            target_frame=Frame.from_transformation(t_world_from_robot_at_approach),
            speed_type='speed.transit.rapid',
            tag="Free Move to reach %s on %s " % (self._tool_string, self._beam_string)
        ))

        # Use vision to aqurire docking offset and apply it to robot controller.
        self.movements.append(AcquireDockingOffset(
            target_frame=Frame.from_transformation(t_world_from_robot_at_approach),
            tag="Visually acquire offset to Toolchanger of %s and move to alignment." % (self._tool_string)
        ))

        # Toolchanger engaging - confirmation of alignment necessary.
        self.movements.append(RoboticLinearMovement(
            target_frame=Frame.from_transformation(t_world_from_robot_at_docked),
            speed_type='speed.toolchange.approach.clamp_on_structure',
            tag="Linear Advance to dock toolchanger of %s." % (self._tool_string),
            operator_stop_before="Confirm Docking alignment",
            allowed_collision_matrix=[('tool_changer', self.tool_id)]
        ))

        # Lock tool
        self.movements.append(RoboticDigitalOutput(
            digital_output=DigitalOutput.LockTool,
            tool_id=self.tool_id,
            attached_objects=self.additional_attached_objects,
            operator_stop_after="Confirm ToolChanger Locked",
            tag="Toolchanger Lock %s" % self._tool_string
        ))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()

##################
# Backward support
##################


class BackwardCompatibilityAction():

    def __init__(self):
        # type: () -> None
        print("WARNING: Deprecated Action Class Used BeamPickupAction. Recompute Actions to fix.")

    @property
    def data(self):
        data = {}

    @data.setter
    def data(self, data):
        pass

    @classmethod
    def from_data(cls, data):
        return cls()

    def to_data(self):
        return {}

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        self.movements = []


BeamPickupAction = BackwardCompatibilityAction
