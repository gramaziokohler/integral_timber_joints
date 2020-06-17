from integral_timber_joints.process.movement import *

##############################
# Base Classes for all actions
# Do not use them directly
##############################

class Action(object):
    """ Base class for all actions.
    """
    def __init__(self):
        self.movements = [] # type: Movement
        self.seq_n = 0 # type: int
        self.act_n = 0 # type: int

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        raise NotImplementedError("Action.create_movements() is not implemented by child class")

class OperatorAction(Action):
    """ Base class for actions performed manually.
    Representing actions that may affect the scene, but not necessary produce robotic code.
    However, pause code for operator-confirmation maybe standard result.
    """
    def __init__(self):
        Action.__init__(self)

class RobotAction(Action):
    """ Base class for actions performed automatically.
    Representing actions that affect the scene.
    """
    def __init__(self):
        Action.__init__(self)

#########################################################
# Base Classes for attaching and detaching objects
# Do not use them directly
#########################################################

class AttachToolAction(object):
    """ Base class for actions that involve attaching a tool (gripper or clamp) to robot.
    For example: PickClampAction (can be from storage or from joint), PickGripperAction (from storage)
    """
    def __init__(self, tool_type, tool_id = None):
        # type: (str, str) -> None
        self.tool_type = tool_type
        self.tool_id = tool_id

class DetachToolAction(object):
    """ Base class for actions that involve detaching a tool (gripper or clamp) from robot.
    For example: PlaceClampAction (can be to storage or to joint), PlaceGripperAction (to storage)
    """
    def __init__(self, tool_type, tool_id = None):
        # type: (str, str) -> None
        self.tool_type = tool_type
        self.tool_id = tool_id

class AttachBeamAction(object):
    """ Base class for actions that involve attaching a workpiece (e.g. beam) to a tool (of the robot).
    For example: PickBeamAction (from storage)
    For path planning purpose, both workpiece and tool is attached to robot.
    """
    def __init__(self, beam_id):
        # type: (str) -> None
        self.beam_id = beam_id
        self.gripper_id = None

class DetachBeamAction(object):
    """ Base class for actions that involve detaching a workpiece (e.g. beam) from a tool (of the robot).
    For example: PlaceBeamWithClampAction, PlaceBeamWithoutClampAction
    """
    def __init__(self, beam_id):
        # type: (str) -> None
        self.beam_id = beam_id
        self.gripper_id = None

##############################
# Actions Classes
##############################

class LoadBeamAction(OperatorAction):
    def __init__(self, seq_n, act_n, beam_id):
        # type: (str) -> None
        OperatorAction.__init__(self)
        self.seq_n = seq_n
        self.act_n = act_n
        self.beam_id = beam_id

    def __str__(self):
        return "Operator load Beam (%s) to pickup station" % (self.beam_id)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        self.movements = []
        grasp_face = process.assembly.get_beam_attribute(self.beam_id, 'gripper_grasp_face')
        self.movements.append(OperatorLoadBeamMovement(self.beam_id, grasp_face))

class PickClampFromStorageAction(RobotAction, AttachToolAction):
    def __init__(self, seq_n, act_n, tool_type = None):
        # type: (str) -> None
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
        tool = process.tool(self.tool_id) # type: Clamp
        tool_storage_frame_wcf = tool.tool_storage_frame
        tool_pick_up_frame_wcf = tool.tool_pick_up_frame_in_wcf(tool_storage_frame_wcf)

        self.movements.append(RoboticFreeMovement(tool_pick_up_frame_wcf.copy())) # Tool Storage Approach
        self.movements.append(RoboticLinearMovement(tool_storage_frame_wcf.copy())) # Tool Storage Final
        self.movements.append(RoboticDigitalOutput(DigitalOutput.LockTool))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.OpenGripper))
        self.movements.append(RoboticLinearMovement(tool_pick_up_frame_wcf.copy(), attached_tool_id=self.tool_id)) # Tool Storage Retract

class PlaceClampToStorageAction(RobotAction, DetachToolAction):
    def __init__(self, seq_n, act_n, tool_type = None):
        # type: (str) -> None
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
        Modified from PickClampFromStorageAction.create_movements()
        """
        self.movements = []
        tool = process.tool(self.tool_id) # type: Clamp
        tool_storage_frame_wcf = tool.tool_storage_frame
        tool_pick_up_frame_wcf = tool.tool_pick_up_frame_in_wcf(tool_storage_frame_wcf)

        self.movements.append(RoboticFreeMovement(tool_pick_up_frame_wcf.copy(), attached_tool_id=self.tool_id)) # Tool Storage Approach
        self.movements.append(RoboticLinearMovement(tool_storage_frame_wcf.copy(), attached_tool_id=self.tool_id)) # Tool Storage Final
        self.movements.append(RoboticDigitalOutput(DigitalOutput.CloseGripper))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.UnlockTool))
        self.movements.append(RoboticLinearMovement(tool_pick_up_frame_wcf.copy())) # Tool Storage Retract

class PickClampFromStructureAction(RobotAction, AttachToolAction):
    def __init__(self, seq_n, act_n, joint_id , tool_type = None):
        # type: (Tuple[str, str], str) -> None
        RobotAction.__init__(self)
        AttachToolAction.__init__(self, tool_type)
        self.seq_n = seq_n
        self.act_n = act_n
        self.joint_id = joint_id

    def __str__(self):
        if self.tool_id:
            object_str = "%s (%s)" % (self.tool_type, self.tool_id)
        else:
            object_str = "%s (?)" % (self.tool_type)
        location_str = "Joint (%s-%s)" % (self.joint_id)
        return "Pick %s from %s" % (object_str, location_str)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Clamp (and other tools) from Storage
        """
        self.movements = []
        tool = process.tool(self.tool_id) # type: Clamp

        clamp_wcf_detachapproach = process.assembly.get_joint_attribute(self.joint_id, 'clamp_wcf_detachapproach')
        clamp_wcf_final = process.assembly.get_joint_attribute(self.joint_id, 'clamp_wcf_final')
        clamp_wcf_detachretract1 = process.assembly.get_joint_attribute(self.joint_id, 'clamp_wcf_detachretract1')
        clamp_wcf_detachretract2 = process.assembly.get_joint_attribute(self.joint_id, 'clamp_wcf_detachretract2')

        self.movements.append(RoboticFreeMovement(clamp_wcf_detachapproach))
        self.movements.append(RoboticLinearMovement(clamp_wcf_final.copy()))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.LockTool))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.OpenGripper))
        self.movements.append(RoboticLinearMovement(clamp_wcf_detachretract1.copy(), attached_tool_id=self.tool_id)) # Tool Retract Frame at structure
        self.movements.append(RoboticLinearMovement(clamp_wcf_detachretract2.copy(), attached_tool_id=self.tool_id)) # Tool Retract Frame at structure

class PlaceClampToStructureAction(RobotAction, DetachToolAction):
    def __init__(self, seq_n, act_n, joint_id , tool_type = None):
        # type: (Tuple[str, str], str) -> None
        RobotAction.__init__(self)
        DetachToolAction.__init__(self, tool_type)
        self.seq_n = seq_n
        self.act_n = act_n
        self.joint_id = joint_id

    def __str__(self):
        if self.tool_id:
            object_str = "%s (%s)" % (self.tool_type, self.tool_id)
        else:
            object_str = "%s (?)" % (self.tool_type)
        location_str = "Joint (%s-%s)" % (self.joint_id)
        return "Place %s to %s" % (object_str, location_str)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Clamp (and other tools) from Storage
        """
        self.movements = []
        tool = process.tool(self.tool_id) # type: Clamp

        clamp_wcf_attachapproach = process.assembly.get_joint_attribute(self.joint_id, 'clamp_wcf_attachapproach')
        clamp_wcf_final = process.assembly.get_joint_attribute(self.joint_id, 'clamp_wcf_final')
        clamp_wcf_attachretract = process.assembly.get_joint_attribute(self.joint_id, 'clamp_wcf_attachretract')

        self.movements.append(RoboticFreeMovement(clamp_wcf_attachapproach.copy(), attached_tool_id=self.tool_id)) # Tool Approach Frame where tool is at structure
        self.movements.append(RoboticLinearMovement(clamp_wcf_final.copy(), attached_tool_id=self.tool_id)) # Tool Final Frame at structure
        self.movements.append(RoboticDigitalOutput(DigitalOutput.CloseGripper))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.UnlockTool))
        self.movements.append(RoboticLinearMovement(clamp_wcf_attachretract.copy()))

class PickGripperFromStorageAction(PickClampFromStorageAction):
    def __init__(self, seq_n, act_n, tool_type):
        # type: (str) -> None
        PickClampFromStorageAction.__init__(self, seq_n, act_n, tool_type)

class PlaceGripperToStorageAction(PlaceClampToStorageAction):
    def __init__(self, seq_n, act_n, tool_type):
        # type: (str) -> None
        PlaceClampToStorageAction.__init__(self, seq_n, act_n, tool_type)

class PickBeamFromStorageAction(RobotAction, AttachBeamAction):
    def __init__(self, seq_n, act_n, beam_id):
        # type: (str, str) -> None
        RobotAction.__init__(self)
        AttachBeamAction.__init__(self, beam_id)
        self.seq_n = seq_n
        self.act_n = act_n

    def __str__(self):
        object_str = "Beam (%s)" % (self.beam_id)
        location_str = "Storage"
        return "Pick %s from %s" % (object_str, location_str)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Beam (with a tool) from Storage
        """
        self.movements = []
        tool = process.tool(self.gripper_id) # type: Clamp

        assembly_wcf_storageapproach = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_storageapproach')
        assembly_wcf_storage = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_storage')
        assembly_wcf_storageretract = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_storageretract')
        assert assembly_wcf_storageapproach is not None and assembly_wcf_storage is not None and assembly_wcf_storageretract is not None

        self.movements.append(RoboticFreeMovement(assembly_wcf_storageapproach.copy(), attached_tool_id=self.gripper_id)) # Tool Approach Frame where tool is at structure
        self.movements.append(RoboticDigitalOutput(DigitalOutput.OpenGripper))
        self.movements.append(RoboticLinearMovement(assembly_wcf_storage.copy(), attached_tool_id=self.gripper_id)) # Tool Final Frame at structure
        self.movements.append(RoboticDigitalOutput(DigitalOutput.CloseGripper))
        self.movements.append(RoboticLinearMovement(assembly_wcf_storageretract.copy(), attached_tool_id=self.gripper_id, attached_beam_id = self.beam_id)) # Tool Retract (current implementation is = clamp_wcf_attachapproach)

class PlaceBeamWithoutClampsAction(RobotAction, DetachBeamAction):
    def __init__(self, seq_n, act_n, beam_id):
        # type: (Tuple[str, str]) -> None
        RobotAction.__init__(self)
        DetachBeamAction.__init__(self, beam_id)
        self.seq_n = seq_n
        self.act_n = act_n

    def __str__(self):
        object_str = "Beam (%s)" % (self.beam_id)
        location_str = "final location without clamps"
        return "Place %s to %s" % (object_str, location_str)

    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Beam (with a tool) from Storage
        """
        self.movements = []
        tool = process.tool(self.gripper_id) # type: Clamp

        assembly_wcf_inclamp = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_inclamp')
        assembly_wcf_final = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_final')
        assembly_wcf_finalretract = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_finalretract')
        assert assembly_wcf_inclamp is not None and assembly_wcf_final is not None and assembly_wcf_finalretract is not None

        self.movements.append(RoboticFreeMovement(assembly_wcf_inclamp.copy(), attached_tool_id=self.gripper_id, attached_beam_id = self.beam_id))
        self.movements.append(RoboticLinearMovement(assembly_wcf_final.copy(), attached_tool_id=self.gripper_id, attached_beam_id = self.beam_id))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.OpenGripper))
        self.movements.append(RoboticLinearMovement(assembly_wcf_finalretract.copy(), attached_tool_id=self.gripper_id))


class PlaceBeamWithClampsAction(RobotAction, DetachBeamAction):
    def __init__(self, seq_n, act_n, beam_id , joint_ids):
        # type: (str, List[Tuple[str, str]]) -> None
        RobotAction.__init__(self)
        DetachBeamAction.__init__(self, beam_id)
        self.seq_n = seq_n
        self.act_n = act_n
        self.joint_ids = joint_ids # type: List[Tuple[str, str]]
        self.clamp_ids = [None for _ in joint_ids] # type: List[str]

    def __str__(self):
        object_str = "Beam (%s)" % (self.beam_id)
        joint_id_str = ["%s-%s"%(joint_id) for joint_id in self.joint_ids]
        location_str = "final location with clamps at joint %s" % joint_id_str
        clamp_id_str = [("?" if clamp_id is None else clamp_id) for clamp_id in self.clamp_ids]
        return "Place %s to %s using clamp %s" % (object_str, location_str, clamp_id_str)


    def create_movements(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        """ Movement for picking Beam (with a tool) from Storage
        """
        self.movements = []
        tool = process.tool(self.gripper_id) # type: Clamp

        assembly_wcf_inclampapproach = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_inclampapproach')
        assembly_wcf_inclamp = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_inclamp')
        assembly_wcf_final = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_final')
        assembly_wcf_finalretract = process.get_gripper_t0cp_for_beam_at(self.beam_id, 'assembly_wcf_finalretract')
        assert assembly_wcf_inclampapproach is not None and assembly_wcf_inclamp is not None and assembly_wcf_final is not None and assembly_wcf_finalretract is not None

        self.movements.append(RoboticFreeMovement(assembly_wcf_inclampapproach.copy(), attached_tool_id=self.gripper_id, attached_beam_id = self.beam_id))
        self.movements.append(RoboticLinearMovement(assembly_wcf_inclamp.copy(), attached_tool_id=self.gripper_id, attached_beam_id = self.beam_id))
        # TODO, This should be a Robot Clamp Sync move
        self.movements.append(RoboticLinearMovement(assembly_wcf_final.copy(), attached_tool_id=self.gripper_id, attached_beam_id = self.beam_id))
        self.movements.append(RoboticDigitalOutput(DigitalOutput.OpenGripper))
        self.movements.append(RoboticLinearMovement(assembly_wcf_finalretract.copy(), attached_tool_id=self.gripper_id))

