try:
    from typing import Dict, List, Optional, Tuple, cast

    from integral_timber_joints.process import RobotClampAssemblyProcess
except:
    pass

from itertools import chain
from integral_timber_joints.process.movement import *
from integral_timber_joints.tools import Clamp, Gripper
from compas.geometry.transformations.translation import Translation


##############################
# Base Classes for all actions
# Do not use them directly
##############################


class Action(object):
    """ Base class for all actions.
    It contains a list of movements
    """

    def __init__(self):
        self.movements = []  # type: list[Movement]
        self.seq_n = 0  # type: int # Zero-based index corelating which Action belongs to which Beam according to the Beam sequence in process.assembly.sequence
        self.act_n = 0  # type: int # Zero-based index corrisponding to the sequential list of Actions. This can be a unique id for an Action

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
# Actions for Tool Change
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

        self.movements.append(OperatorLoadBeamMovement(self.beam_id, grasp_face, beam_pickup_frame_wcf))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


class PickToolFromStorageAction(RobotAction, AttachToolAction):
    def __init__(self, seq_n=0, act_n=0, tool_type=None, tool_id=None):
        # type: (int, int, str, str) -> None
        RobotAction.__init__(self)
        AttachToolAction.__init__(self, tool_type, tool_id)
        self.seq_n = seq_n
        self.act_n = act_n

    def __str__(self):
        return "Pick %s from Storage" % (self._tool_string)


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

        self.movements.append(RoboticFreeMovement(tool_pick_up_frame_t0cf.copy(), speed_type='speed.transit.rapid',
                                                  tag="Free Move reach Storage Approach Frame of %s, to get tool." % self._tool_string))  # Tool Storage Approach
        self.movements.append(RoboticLinearMovement(tool_storage_frame_t0cf.copy(),
                                                    speed_type='speed.toolchange.approach.notool',
                                                    target_configuration=tool.tool_storage_configuration,
                                                    tag="Linear Advance to Storage Frame of %s, to get tool." % self._tool_string,
                                                    allowed_collision_matrix=[('tool_changer', self.tool_id)]))  # Tool Storage Final
        self.movements.append(RoboticDigitalOutput(DigitalOutput.LockTool, self.tool_id,
                                                   tag="Toolchanger Lock %s" % self._tool_string))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.OpenGripper, self.tool_id,
                                                   tag="%s Open Gripper to release itself from storage pad." % self._tool_string))

        tool_env_acm = [(self.tool_id, env_id) for env_id in process.environment_models.keys()]

        tool_pick_up_retract_frame_wcf = tool.tool_pick_up_retract_frame_in_wcf(tool_storage_frame_wcf)
        tool_pick_up_retract_frame_t0cf = toolchanger.set_current_frame_from_tcp(tool_pick_up_retract_frame_wcf)

        self.movements.append(RoboticLinearMovement(tool_pick_up_retract_frame_t0cf.copy(), attached_tool_id=self.tool_id,
                                                    speed_type='speed.toolchange.retract.withtool',
                                                    tag="Linear Retract after getting %s from storage." % self._tool_string,
                                                    allowed_collision_matrix=tool_env_acm,
                                                    ))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


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

        self.movements.append(RoboticFreeMovement(tool_pick_up_retract_frame_t0cf.copy(), attached_tool_id=self.tool_id, speed_type='speed.transit.rapid',
                                                  tag="Free Move to reach Storage Approach Frame of %s, to place tool in storage." % self._tool_string))  # Tool Storage Approach
        tool_env_acm = [(self.tool_id, env_id) for env_id in process.environment_models.keys()]
        self.movements.append(RoboticLinearMovement(tool_storage_frame_t0cf.copy(), attached_tool_id=self.tool_id,
                                                    speed_type='speed.toolchange.approach.withtool',
                                                    target_configuration=tool.tool_storage_configuration,
                                                    tag="Linear Advance to Storage Frame of %s, to place tool in storage." % self._tool_string,
                                                    allowed_collision_matrix=tool_env_acm,
                                                    ))  # Tool Storage Final
        self.movements.append(RoboticDigitalOutput(DigitalOutput.CloseGripper, self.tool_id,
                                                   tag="%s Close Gripper to lock onto storage pad." % self._tool_string))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.UnlockTool, self.tool_id,
                                                   tag="Toolchanger Unlock %s" % self._tool_string))

        tool_pick_up_frame_wcf = tool.tool_pick_up_frame_in_wcf(tool_storage_frame_wcf)
        tool_pick_up_frame_t0cf = toolchanger.set_current_frame_from_tcp(tool_pick_up_frame_wcf)
        self.movements.append(RoboticLinearMovement(tool_pick_up_frame_t0cf.copy(), speed_type='speed.toolchange.retract.notool',
                                                    tag="Linear Retract from storage after placing %s in storage" % self._tool_string,
                                                    allowed_collision_matrix=[('tool_changer', self.tool_id)],
                                                    ))  # Tool Storage Retract

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
        self.movements.append(RoboticFreeMovement(tool_pick_up_frame_t0cf.copy(), speed_type='speed.transit.rapid',
                                                  tag="Free Move reach Storage Approach Frame of %s, to get clamp." % self._tool_string))  # Tool Storage Approach
        self.movements.append(RoboticLinearMovement(tool_storage_frame_t0cf.copy(), speed_type='speed.toolchange.approach.notool',
                                                    target_configuration=tool.tool_storage_configuration,
                                                    tag="Linear Advance to Storage Frame of %s, to get tool." % self._tool_string,
                                                    allowed_collision_matrix=[('tool_changer', self.tool_id)]
                                                    ))  # Tool Storage Final
        self.movements.append(RoboticDigitalOutput(DigitalOutput.LockTool, self.tool_id,
                                                   tag="Toolchanger Lock %s" % self._tool_string))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.OpenGripper, self.tool_id,
                                                   tag="%s Open Gripper to release itself from storage pad." % self._tool_string))
        # Retraction movement
        acm = [(self.tool_id, env_id) for env_id in process.environment_models.keys()]
        tool_storage_retract_frame1_t0cf = toolchanger.set_current_frame_from_tcp(tool.tool_storage_retract_frame1)
        tool_storage_retract_frame2_t0cf = toolchanger.set_current_frame_from_tcp(tool.tool_storage_retract_frame2)
        tool_env_acm = [(self.tool_id, env_id) for env_id in process.environment_models.keys()]

        self.movements.append(RoboticLinearMovement(tool_storage_retract_frame1_t0cf.copy(), attached_tool_id=self.tool_id,
                                                    speed_type='speed.toolchange.retract.withtool',
                                                    tag="Linear Retract 1 of 2 after getting %s from storage." % self._tool_string,
                                                    allowed_collision_matrix=tool_env_acm
                                                    ))
        self.movements.append(RoboticLinearMovement(tool_storage_retract_frame2_t0cf.copy(), attached_tool_id=self.tool_id,
                                                    speed_type='speed.toolchange.retract.withtool',
                                                    tag="Linear Retract 2 of 2 after getting %s from storage." % self._tool_string,
                                                    allowed_collision_matrix=tool_env_acm
                                                    ))

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
        self.movements.append(RoboticFreeMovement(tool_storage_approach_frame1_t0cf.copy(),
                                                  attached_tool_id=self.tool_id, speed_type='speed.transit.rapid',
                                                  tag="Free Move reach Storage Approach Frame of %s, to place clamp in storage." % self._tool_string))  # Tool Storage Approach

        tool_env_acm = [(self.tool_id, env_id) for env_id in process.environment_models.keys()]
        self.movements.append(RoboticLinearMovement(tool_storage_approach_frame2_t0cf.copy(), attached_tool_id=self.tool_id,
                                                    speed_type='speed.toolchange.approach.withtool',
                                                    tag="Linear Approach 1 of 2 to place %s in storage." % self._tool_string,
                                                    allowed_collision_matrix=tool_env_acm
                                                    ))  # Tool Storage Approach

        # Tool go to final
        tool_storage_frame_wcf = tool.tool_storage_frame
        tool_storage_frame_t0cf = toolchanger.set_current_frame_from_tcp(tool_storage_frame_wcf)
        self.movements.append(RoboticLinearMovement(tool_storage_frame_t0cf.copy(), attached_tool_id=self.tool_id,
                                                    speed_type='speed.toolchange.approach.withtool',
                                                    target_configuration=tool.tool_storage_configuration,
                                                    tag="Linear Approach 2 of 2 to place %s in storage." % self._tool_string,
                                                    allowed_collision_matrix=tool_env_acm
                                                    ))  # Tool Storage Final
        self.movements.append(RoboticDigitalOutput(DigitalOutput.CloseGripper, self.tool_id,
                                                   tag="Close Gripper to lock %s onto storage pad." % self._tool_string))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.UnlockTool, self.tool_id,
                                                   tag="Toolchanger Unlock %s" % self._tool_string))

        # Toolchanger retract
        tool_pick_up_frame_wcf = tool.tool_pick_up_frame_in_wcf(tool_storage_frame_wcf)
        tool_pick_up_frame_t0cf = toolchanger.set_current_frame_from_tcp(tool_pick_up_frame_wcf)
        self.movements.append(RoboticLinearMovement(tool_pick_up_frame_t0cf.copy(), speed_type='speed.toolchange.retract.notool',
                                                    tag="Linear Retract from storage after placing %s in storage" % self._tool_string,
                                                    allowed_collision_matrix=[('tool_changer', self.tool_id)]
                                                    ))  # Tool Storage Retract

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

        self.movements.append(OperatorAttachToolMovement(self.beam_id, self.joint_id, self.tool_type, self.tool_id,
                                                         target_frame=screwdriver_frame_at_position,
                                                         tag="Opeartor Attach %s to %s." % (self._tool_string, self._joint_string),
                                                         ))
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

        clamp_wcf_detachapproach = process.get_tool_t0cf_at(self.joint_id, 'clamp_wcf_detachapproach')
        clamp_wcf_final = process.get_tool_t0cf_at(self.joint_id, 'clamp_wcf_final')
        clamp_wcf_detachretract1 = process.get_tool_t0cf_at(self.joint_id, 'clamp_wcf_detachretract1')
        clamp_wcf_detachretract2 = process.get_tool_t0cf_at(self.joint_id, 'clamp_wcf_detachretract2')

        # Approach the clamp at the structure
        self.movements.append(RoboticFreeMovement(clamp_wcf_detachapproach, speed_type='speed.transit.rapid',
                                                  tag="Free Move to reach %s to detach it from structure." % self._tool_string))

        # Toolchanger engaging - confirmation of alignment necessary.
        self.movements.append(RoboticLinearMovement(clamp_wcf_final.copy(), speed_type='speed.toolchange.approach.clamp_on_structure',
                                                    tag="Linear Advance to mate toolchanger of %s to detach it from structure." % self._tool_string,
                                                    operator_stop_before="Confirm ToolChanger alignment",
                                                    allowed_collision_matrix=[('tool_changer', self.tool_id)]
                                                    ))
        # Additional ACM between clamp and the attached two beam (at the joint)
        acm = [(self.joint_id[0], self.tool_id), (self.joint_id[1], self.tool_id)]

        # Lock tool and Open Clamp Jaw
        self.movements.append(RoboticDigitalOutput(DigitalOutput.LockTool, self.tool_id,
                                                   operator_stop_after="Confirm ToolChanger Locked",
                                                   tag="Toolchanger Lock %s" % self._tool_string,))
        self.movements.append(ClampsJawMovement([process.clamp_appraoch_position], [self.tool_id],
                                                speed_type='speed.clamp.rapid',
                                                tag="%s Open Clamp Jaws to be released." % self._tool_string))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.OpenGripper, self.tool_id,
                                                   tag="%s Open Gripper to be released from structure." % self._tool_string))

        # Retract
        self.movements.append(RoboticLinearMovement(clamp_wcf_detachretract1.copy(), attached_tool_id=self.tool_id,
                                                    speed_type='speed.toolchange.retract.clamp_on_structure',
                                                    tag="Linear Retract 1 of 2 to storage after picking up %s from structure." % self._tool_string,
                                                    allowed_collision_matrix=acm))  # Tool Retract Frame at structure
        self.movements.append(RoboticLinearMovement(clamp_wcf_detachretract2.copy(), attached_tool_id=self.tool_id,
                                                    speed_type='speed.toolchange.retract.clamp_on_structure',
                                                    tag="Linear Retract 2 of 2 to storage after picking up %s from structure." % self._tool_string,
                                                    allowed_collision_matrix=acm))  # Tool Retract Frame at structure

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

        clamp_wcf_attachapproach1 = process.get_tool_t0cf_at(self.joint_id, 'clamp_wcf_attachapproach1')
        clamp_wcf_attachapproach2 = process.get_tool_t0cf_at(self.joint_id, 'clamp_wcf_attachapproach2')
        clamp_wcf_final = process.get_tool_t0cf_at(self.joint_id, 'clamp_wcf_final')
        clamp_wcf_attachretract = process.get_tool_t0cf_at(self.joint_id, 'clamp_wcf_attachretract')

        # Approach the clamp at the structure
        self.movements.append(RoboticFreeMovement(clamp_wcf_attachapproach1.copy(), attached_tool_id=self.tool_id,
                                                  speed_type='speed.transit.rapid',
                                                  tag="Free Move to bring %s to structure." % self._tool_string))  # Tool Approach Frame where tool is at structure

        # Additional ACM between clamp and the attached two beam (at the joint)
        acm = [(self.joint_id[0], self.tool_id), (self.joint_id[1], self.tool_id)]

        # Two Approach Moves
        self.movements.append(RoboticLinearMovement(clamp_wcf_attachapproach2.copy(), attached_tool_id=self.tool_id,
                                                    speed_type='speed.toolchange.approach.clamp_on_structure',
                                                    tag="Linear Approach 1 of 2 to attach %s to structure." % self._tool_string,
                                                    allowed_collision_matrix=acm))  # Tool Approach Frame where tool is at structure
        self.movements.append(RoboticLinearMovement(clamp_wcf_final.copy(), attached_tool_id=self.tool_id,
                                                    speed_type='speed.toolchange.approach.clamp_on_structure',
                                                    tag="Linear Approach 2 of 2 to attach %s to structure." % self._tool_string,
                                                    allowed_collision_matrix=acm))  # Tool Final Frame at structure
        self.movements.append(RoboticDigitalOutput(DigitalOutput.CloseGripper, self.tool_id,
                                                   operator_stop_before="Confirm Gripper Pins Alignment",
                                                   operator_stop_after="Confirm Gripper Closed Properly",
                                                   tag="%s Close Gripper and attach to structure." % self._tool_string))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.UnlockTool, self.tool_id,
                                                   tag="Toolchanger Unlock %s." % self._tool_string))
        self.movements.append(RoboticLinearMovement(clamp_wcf_attachretract.copy(), speed_type='speed.toolchange.retract.notool',
                                                    tag="Linear Retract after attaching %s on structure" % self._tool_string,
                                                    allowed_collision_matrix=[('tool_changer', self.tool_id)],
                                                    operator_stop_after="Confirm Clamp stay sttached",))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


######################################
# Actions for Picking and Placing Beam
######################################

class PickBeamWithGripperAction(RobotAction, AttachBeamAction):
    def __init__(self, seq_n=0, act_n=0, beam_id=None, gripper_id=None, additional_attached_objects=[]):
        # type: (int, int, str, str, list[str]) -> None
        """
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
        tool = process.tool(self.gripper_id)  # type: Clamp

        assembly_wcf_pickupapproach = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_pickupapproach')
        assembly_wcf_pickup = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_pickup')
        assembly_wcf_pickupretract = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_pickupretract')
        assert assembly_wcf_pickupapproach is not None and assembly_wcf_pickup is not None and assembly_wcf_pickupretract is not None

        self.movements.append(RoboticFreeMovement(assembly_wcf_pickupapproach.copy(), attached_tool_id=self.gripper_id,
                                                  speed_type='speed.transit.rapid',
                                                  tag="Free Move to reach Pickup Approach Frame of Beam ('%s')" % (self.beam_id)))  # Tool Approach Frame where tool is at structure
        self.movements.append(RoboticDigitalOutput(DigitalOutput.OpenGripper, self.gripper_id,
                                                   tag="Gripper ('%s') Open Gripper before gripping Beam ('%s')" % (self.gripper_id, self.beam_id)))
        self.movements.append(RoboticLinearMovement(assembly_wcf_pickup.copy(), attached_tool_id=self.gripper_id,
                                                    speed_type='speed.gripper.approach',
                                                    tag="Linear Advance to Storage Frame of Beam ('%s')" % (self.beam_id),
                                                    target_configuration=process.pickup_station.beam_pickup_configuration))  # Tool Final Frame at structure

        # Close Gripper and
        self.movements.append(RoboticDigitalOutput(DigitalOutput.CloseGripper, self.gripper_id,
                                                   attached_objects=[self.beam_id] + self.additional_attached_objects,
                                                   operator_stop_before="Confirm Gripper Ready to close ",
                                                   operator_stop_after="Confirm Grip OK",
                                                   tag="Gripper ('%s') Close Gripper to grip Beam ('%s')" % (self.gripper_id, self.beam_id)))
        self.movements.append(RoboticLinearMovement(assembly_wcf_pickupretract.copy(), attached_tool_id=self.gripper_id,
                                                    attached_beam_id=self.beam_id, speed_type='speed.transfer.caution',
                                                    tag="Linear Retract after picking up Beam ('%s')" % (self.beam_id)))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


class PickBeamWithScrewdriverAction(PickBeamWithGripperAction):
    """
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

        assembly_wcf_pickupapproach = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_pickupapproach')
        assembly_wcf_pickup = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_pickup')
        assembly_wcf_pickupretract = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_pickupretract')
        assert assembly_wcf_pickupapproach is not None and assembly_wcf_pickup is not None and assembly_wcf_pickupretract is not None

        self.movements.append(RoboticLinearMovement(assembly_wcf_pickupretract.copy(), attached_tool_id=self.gripper_id,
                                                    attached_beam_id=self.beam_id, speed_type='speed.transfer.caution',
                                                    tag="Linear Retract after picking up Beam ('%s')" % (self.beam_id)))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()


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

        assembly_wcf_inclamp = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_inclamp')
        assembly_wcf_final = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_final')
        assembly_wcf_finalretract = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_finalretract')
        assert assembly_wcf_inclamp is not None and assembly_wcf_final is not None and assembly_wcf_finalretract is not None

        self.movements.append(RoboticFreeMovement(assembly_wcf_inclamp.copy(), attached_tool_id=self.gripper_id, attached_beam_id=self.beam_id, speed_type='speed.transfer.rapid',
                                                  tag="Free Move to bring Beam ('%s') to final location" % self.beam_id))

        # Additional ACM between the attached beam, clamps and the neighbouring beams
        neighbour_beam_ids = process.assembly.get_already_built_neighbors(self.beam_id)
        acm = [(self.beam_id, nbr_id) for nbr_id in neighbour_beam_ids] + \
              [(self.beam_id, env_id) for env_id in process.environment_models.keys()]

        # Assembly linear move to final location
        self.movements.append(RoboticLinearMovement(assembly_wcf_final.copy(), attached_tool_id=self.gripper_id,
                                                    attached_beam_id=self.beam_id, planning_priority=1, speed_type='speed.assembly.noclamp',
                                                    tag="Linear Advance to Final Frame of Beam ('%s')" % (self.beam_id),
                                                    allowed_collision_matrix=acm))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.OpenGripper, self.gripper_id,
                                                   attached_objects=[self.beam_id],
                                                   operator_stop_before="Confirm Beam Temporary Support In Place", operator_stop_after="Confirm Gripper Cleared Beam",
                                                   tag="Open Gripper ('%s') and let go of Beam ('%s')" % (self.gripper_id, self.beam_id)))
        self.movements.append(RoboticLinearMovement(assembly_wcf_finalretract.copy(), attached_tool_id=self.gripper_id, speed_type='speed.gripper.retract',
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

        assembly_wcf_inclampapproach = process.get_gripper_t0cp_for_beam_at(
            self.beam_id, 'assembly_wcf_inclampapproach')
        assembly_wcf_inclamp = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_inclamp')
        assembly_wcf_final = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_final')
        assembly_wcf_finalretract = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_finalretract')
        assert assembly_wcf_inclampapproach is not None and assembly_wcf_inclamp is not None and assembly_wcf_final is not None and assembly_wcf_finalretract is not None

        # Transfer beam to the clamp(s)
        self.movements.append(RoboticFreeMovement(assembly_wcf_inclampapproach.copy(), attached_tool_id=self.gripper_id,
                                                  attached_beam_id=self.beam_id, speed_type='speed.transfer.rapid',
                                                  tag="Free Move to bring Beam ('%s') to approach clamps on structure." % self.beam_id))

        acm = [(self.beam_id, clamp_id) for clamp_id in self.clamp_ids]
        # Linear movement into clamp jaws
        self.movements.append(RoboticLinearMovement(assembly_wcf_inclamp.copy(), attached_tool_id=self.gripper_id,
                                                    attached_beam_id=self.beam_id, speed_type='speed.assembly.inclamp',
                                                    tag="Linear Advance to bring Beam ('%s') into clamp jaws" % self.beam_id,
                                                    allowed_collision_matrix=acm))
        self.movements.append(ClampsJawMovement([process.clamp_inclamp_position] * len(self.clamp_ids), self.clamp_ids, speed_type='speed.clamp.rapid',
                                                tag="Clamps (%s) close slightly to touch Beam ('%s')" % (self.clamp_ids, self.beam_id)))  # Extend the clamp arm

        # Additional ACM between the attached beam, clamps and the neighbouring beams
        neighbour_beam_ids = process.assembly.get_already_built_neighbors(self.beam_id)
        acm = [(self.beam_id, nbr_id) for nbr_id in chain(self.clamp_ids, neighbour_beam_ids)] + \
              [(self.beam_id, env_id) for env_id in process.environment_models.keys()] + \
              [(self.gripper_id, clamp_id) for clamp_id in self.clamp_ids]  # this is a hack for the planner to work because it cannot actuate the clamp jaw while planning.

        # Robot Clamp Sync Move to final location
        target_frame, attached_tool_id, attached_beam_id = assembly_wcf_final.copy(), self.gripper_id, self.beam_id
        jaw_positions, clamp_ids = [process.clamp_final_position] * len(self.clamp_ids), self.clamp_ids
        self.movements.append(RoboticClampSyncLinearMovement(target_frame, attached_tool_id, attached_beam_id,
                                                             jaw_positions, clamp_ids, planning_priority=1, speed_type='speed.assembly.clamping',
                                                             tag="Robot and Clamps (%s) syncronously move to clamp Beam ('%s')" % (
                                                                 self.clamp_ids, self.beam_id),
                                                             allowed_collision_matrix=acm))

        # Open gripper and retract
        self.movements.append(RoboticDigitalOutput(DigitalOutput.OpenGripper, self.gripper_id,
                                                   attached_objects=[self.beam_id],
                                                   operator_stop_before="Confirm Beam Is Stable", operator_stop_after="Confirm Gripper Cleared Beam",
                                                   tag="Open Gripper ('%s') and let go of Beam ('%s')" % (self.gripper_id, self.beam_id)))
        self.movements.append(RoboticLinearMovement(assembly_wcf_finalretract.copy(), attached_tool_id=self.gripper_id, speed_type='speed.gripper.retract',
                                                    tag="Linear retract after placing Beam ('%s')" % self.beam_id,
                                                    ))

        # Assign Unique Movement IDs to all movements
        self.assign_movement_ids()
