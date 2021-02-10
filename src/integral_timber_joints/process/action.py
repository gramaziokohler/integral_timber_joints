try:
    from typing import Dict, List, Optional, Tuple

    from integral_timber_joints.process import RobotClampAssemblyProcess
except:
    pass

from integral_timber_joints.process.movement import *
from integral_timber_joints.tools import Clamp, Gripper

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
        self.seq_n = 0  # type: int
        self.act_n = 0  # type: int

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        raise NotImplementedError("Action.create_movements() is not implemented by child class")

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


class AttachBeamAction(RobotIOAction):
    """ Base class for actions that involve attaching a workpiece (e.g. beam) to a tool (of the robot).
    For example: PickBeamAction (from storage)
    For path planning purpose, both workpiece and tool is attached to robot.
    """

    def __init__(self, beam_id=None):
        # type: (str) -> None
        self.beam_id = beam_id
        self.gripper_id = None

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

    def __init__(self, beam_id=None):
        # type: (str) -> None
        self.beam_id = beam_id
        self.gripper_id = None

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
        # type: (str, int, int) -> None
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


class PickToolFromStorageAction(RobotAction, AttachToolAction):
    def __init__(self, seq_n=0, act_n=0, tool_type=None):
        # type: (str, int , int, str) -> None
        RobotAction.__init__(self)
        AttachToolAction.__init__(self, tool_type)
        self.seq_n = seq_n
        self.act_n = act_n

    def __str__(self):
        if self.tool_id:
            object_str = "%s ('%s')" % (self.tool_type, self.tool_id)
        else:
            object_str = "%s (?)" % (self.tool_type)
        return "Pick %s from Storage" % (object_str)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Clamp (and other tools) from Storage
        """
        self.movements = []
        tool = process.tool(self.tool_id)  # type: Clamp
        toolchanger = process.robot_toolchanger
        tool_storage_frame_wcf = tool.tool_storage_frame
        tool_storage_frame_t0cf = toolchanger.set_current_frame_from_tcp(tool_storage_frame_wcf)

        tool_pick_up_frame_wcf = tool.tool_pick_up_frame_in_wcf(tool_storage_frame_wcf)
        tool_pick_up_frame_t0cf = toolchanger.set_current_frame_from_tcp(tool_pick_up_frame_wcf)

        self.movements.append(RoboticFreeMovement(tool_pick_up_frame_t0cf.copy()))  # Tool Storage Approach
        self.movements.append(RoboticLinearMovement(tool_storage_frame_t0cf.copy()))  # Tool Storage Final
        self.movements.append(RoboticDigitalOutput(DigitalOutput.LockTool, self.tool_id))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.OpenGripper, self.tool_id))  # Open Gripper to release it from the tool storage rack
        self.movements.append(RoboticLinearMovement(tool_pick_up_frame_t0cf.copy(), attached_tool_id=self.tool_id))  # Tool Storage Retract


class PlaceToolToStorageAction(RobotAction, DetachToolAction):
    def __init__(self, seq_n=0, act_n=0, tool_type=None):
        # type: (str, int , int, str) -> None
        RobotAction.__init__(self)
        DetachToolAction.__init__(self, tool_type)
        self.seq_n = seq_n
        self.act_n = act_n

    def __str__(self):
        if self.tool_id:
            object_str = "%s ('%s')" % (self.tool_type, self.tool_id)
        else:
            object_str = "%s (?)" % (self.tool_type)
        return "Place %s to Storage" % (object_str)

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

        tool_pick_up_frame_wcf = tool.tool_pick_up_frame_in_wcf(tool_storage_frame_wcf)
        tool_pick_up_frame_t0cf = toolchanger.set_current_frame_from_tcp(tool_pick_up_frame_wcf)

        self.movements.append(RoboticFreeMovement(tool_pick_up_frame_t0cf.copy(), attached_tool_id=self.tool_id))  # Tool Storage Approach
        self.movements.append(RoboticLinearMovement(tool_storage_frame_t0cf.copy(), attached_tool_id=self.tool_id))  # Tool Storage Final
        self.movements.append(RoboticDigitalOutput(DigitalOutput.CloseGripper, self.tool_id))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.UnlockTool, self.tool_id))
        self.movements.append(RoboticLinearMovement(tool_pick_up_frame_t0cf.copy()))  # Tool Storage Retract


class PickGripperFromStorageAction(PickToolFromStorageAction):
    pass


class PlaceGripperToStorageAction(PlaceToolToStorageAction):
    pass


class PickClampFromStorageAction(PickToolFromStorageAction):
    pass


class PlaceClampToStorageAction(PlaceToolToStorageAction):
    pass

############################################
# Actions for leaving Clamps on Structure
############################################


class PickClampFromStructureAction(RobotAction, AttachToolAction):
    def __init__(self, seq_n=0, act_n=0, joint_id=None, tool_type=None):
        # type: (int, int, tuple[str, str], str) -> None
        RobotAction.__init__(self)
        AttachToolAction.__init__(self, tool_type)
        self.seq_n = seq_n
        self.act_n = act_n
        self.joint_id = joint_id

    def __str__(self):
        if self.tool_id:
            object_str = "%s ('%s')" % (self.tool_type, self.tool_id)
        else:
            object_str = "%s (?)" % (self.tool_type)
        location_str = "Joint ('%s-%s')" % (self.joint_id)
        return "Pick %s from %s" % (object_str, location_str)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Clamp (and other tools) from Storage
        """
        self.movements = []
        tool = process.tool(self.tool_id)  # type: Clamp

        clamp_wcf_detachapproach = process.get_clamp_t0cf_at(self.joint_id, 'clamp_wcf_detachapproach')
        clamp_wcf_final = process.get_clamp_t0cf_at(self.joint_id, 'clamp_wcf_final')
        clamp_wcf_detachretract1 = process.get_clamp_t0cf_at(self.joint_id, 'clamp_wcf_detachretract1')
        clamp_wcf_detachretract2 = process.get_clamp_t0cf_at(self.joint_id, 'clamp_wcf_detachretract2')

        self.movements.append(RoboticFreeMovement(clamp_wcf_detachapproach))
        self.movements.append(RoboticLinearMovement(clamp_wcf_final.copy()))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.LockTool, self.tool_id))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.OpenGripper, self.tool_id))
        self.movements.append(RoboticLinearMovement(clamp_wcf_detachretract1.copy(), attached_tool_id=self.tool_id))  # Tool Retract Frame at structure
        self.movements.append(RoboticLinearMovement(clamp_wcf_detachretract2.copy(), attached_tool_id=self.tool_id))  # Tool Retract Frame at structure


class PlaceClampToStructureAction(RobotAction, DetachToolAction):
    def __init__(self, seq_n=0, act_n=0, joint_id=None, tool_type=None):
        # type: (int, int, tuple[str, str], str) -> None
        RobotAction.__init__(self)
        DetachToolAction.__init__(self, tool_type)
        self.seq_n = seq_n
        self.act_n = act_n
        self.joint_id = joint_id

    def __str__(self):
        if self.tool_id:
            object_str = "%s ('%s')" % (self.tool_type, self.tool_id)
        else:
            object_str = "%s (?)" % (self.tool_type)
        location_str = "Joint ('%s-%s')" % (self.joint_id)
        return "Place %s to %s" % (object_str, location_str)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Clamp (and other tools) from Storage
        """
        self.movements = []
        tool = process.tool(self.tool_id)  # type: Clamp

        clamp_wcf_attachapproach1 = process.get_clamp_t0cf_at(self.joint_id, 'clamp_wcf_attachapproach1')
        clamp_wcf_attachapproach2 = process.get_clamp_t0cf_at(self.joint_id, 'clamp_wcf_attachapproach2')
        clamp_wcf_final = process.get_clamp_t0cf_at(self.joint_id, 'clamp_wcf_final')
        clamp_wcf_attachretract = process.get_clamp_t0cf_at(self.joint_id, 'clamp_wcf_attachretract')

        self.movements.append(ClampsJawMovement([process.clamp_appraoch_position], [self.tool_id]))  # Extend the clamp arm
        self.movements.append(RoboticFreeMovement(clamp_wcf_attachapproach1.copy(), attached_tool_id=self.tool_id))  # Tool Approach Frame where tool is at structure
        self.movements.append(RoboticLinearMovement(clamp_wcf_attachapproach2.copy(), attached_tool_id=self.tool_id))  # Tool Approach Frame where tool is at structure
        self.movements.append(RoboticLinearMovement(clamp_wcf_final.copy(), attached_tool_id=self.tool_id))  # Tool Final Frame at structure
        self.movements.append(RoboticDigitalOutput(DigitalOutput.CloseGripper, self.tool_id))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.UnlockTool, self.tool_id))
        self.movements.append(RoboticLinearMovement(clamp_wcf_attachretract.copy()))


class BeamPickupAction(RobotAction, AttachBeamAction):
    def __init__(self, seq_n=0, act_n=0, beam_id=None):
        # type: (str, int , int, str) -> None
        RobotAction.__init__(self)
        AttachBeamAction.__init__(self, beam_id)
        self.seq_n = seq_n
        self.act_n = act_n

    def __str__(self):
        object_str = "Beam ('%s')" % (self.beam_id)
        location_str = "Storage"
        return "Pick %s from %s" % (object_str, location_str)

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

        self.movements.append(RoboticFreeMovement(assembly_wcf_pickupapproach.copy(), attached_tool_id=self.gripper_id))  # Tool Approach Frame where tool is at structure
        self.movements.append(RoboticDigitalOutput(DigitalOutput.OpenGripper, self.gripper_id))
        self.movements.append(RoboticLinearMovement(assembly_wcf_pickup.copy(), attached_tool_id=self.gripper_id))  # Tool Final Frame at structure
        self.movements.append(RoboticDigitalOutput(DigitalOutput.CloseGripper, self.gripper_id, self.beam_id))
        self.movements.append(RoboticLinearMovement(assembly_wcf_pickupretract.copy(), attached_tool_id=self.gripper_id,
                                                    attached_beam_id=self.beam_id))  # Tool Retract (current implementation is = clamp_wcf_attachapproach)


class BeamPlacementWithoutClampsAction(RobotAction, DetachBeamAction):
    def __init__(self, seq_n=0, act_n=0, beam_id=None):
        # type: (int, int, tuple[str, str]) -> None
        RobotAction.__init__(self)
        DetachBeamAction.__init__(self, beam_id)
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

        self.movements.append(RoboticFreeMovement(assembly_wcf_inclamp.copy(), attached_tool_id=self.gripper_id, attached_beam_id=self.beam_id))
        self.movements.append(RoboticLinearMovement(assembly_wcf_final.copy(), attached_tool_id=self.gripper_id, attached_beam_id=self.beam_id))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.OpenGripper, self.gripper_id, self.beam_id))
        self.movements.append(RoboticLinearMovement(assembly_wcf_finalretract.copy(), attached_tool_id=self.gripper_id))


class BeamPlacementWithClampsAction(RobotAction, DetachBeamAction):
    def __init__(self, seq_n=0, act_n=0, beam_id=None, joint_ids=[]):
        # type: (int, int, str, list[tuple[str, str]]) -> None
        RobotAction.__init__(self)
        DetachBeamAction.__init__(self, beam_id)
        self.seq_n = seq_n
        self.act_n = act_n
        self.joint_ids = joint_ids  # type: list[tuple[str, str]]
        self.clamp_ids = [None for _ in joint_ids]  # type: list[str]

    def __str__(self):
        object_str = "Beam ('%s')" % (self.beam_id)
        joint_str = ["%s-%s" % (joint_id) for joint_id in self.joint_ids]
        location_str = "final location with clamps at %s" % joint_str
        clamp_id_str = [("?" if clamp_id is None else clamp_id) for clamp_id in self.clamp_ids]
        return "Place %s to Joint %s using Clamp %s" % (object_str, location_str, clamp_id_str)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Beam (with a tool) from Storage
        """
        self.movements = []
        tool = process.tool(self.gripper_id)  # type: Clamp

        assembly_wcf_inclampapproach = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_inclampapproach')
        assembly_wcf_inclamp = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_inclamp')
        assembly_wcf_final = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_final')
        assembly_wcf_finalretract = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_finalretract')
        assert assembly_wcf_inclampapproach is not None and assembly_wcf_inclamp is not None and assembly_wcf_final is not None and assembly_wcf_finalretract is not None

        self.movements.append(RoboticFreeMovement(assembly_wcf_inclampapproach.copy(), attached_tool_id=self.gripper_id, attached_beam_id=self.beam_id))
        self.movements.append(RoboticLinearMovement(assembly_wcf_inclamp.copy(), attached_tool_id=self.gripper_id, attached_beam_id=self.beam_id))
        self.movements.append(ClampsJawMovement([process.clamp_inclamp_position] * len(self.clamp_ids), self.clamp_ids))  # Extend the clamp arm

        # A Robot Clamp Sync Move
        target_frame, attached_tool_id, attached_beam_id = assembly_wcf_final.copy(), self.gripper_id, self.beam_id
        jaw_positions, clamp_ids = [process.clamp_final_position] * len(self.clamp_ids), self.clamp_ids
        self.movements.append(RoboticClampSyncLinearMovement(target_frame, attached_tool_id, attached_beam_id, jaw_positions, clamp_ids))

        self.movements.append(RoboticDigitalOutput(DigitalOutput.OpenGripper, self.gripper_id, self.beam_id))
        self.movements.append(RoboticLinearMovement(assembly_wcf_finalretract.copy(), attached_tool_id=self.gripper_id))
