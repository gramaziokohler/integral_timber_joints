try:
    from typing import Dict, List, Optional, Tuple

    from integral_timber_joints.process import RFLPathPlanner, RobotClampAssemblyProcess
except:
    pass
import itertools
from copy import deepcopy

import compas
import compas_fab
from compas.geometry import Frame

from integral_timber_joints.assembly import Assembly, BeamAssemblyMethod
from integral_timber_joints.geometry import Beam, Joint
from integral_timber_joints.process.action import *
from integral_timber_joints.process.dependency import ComputationalResult
from integral_timber_joints.process.movement import *
from integral_timber_joints.process.state import SceneState
from integral_timber_joints.tools import Clamp, Gripper, Tool


######################################################
# Create Actions according to Flow Chart
######################################################

def create_actions_from_sequence(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, Optional[bool]) -> None
    """ Creating Action objects (process.actions) from process.sequence

    Beam attribute 'assembly_method' should be assigned prior.
    Joint attribute 'tool_type' and 'tool_id' should be assigned.

    Action numbers are zeroed at the begining of the Beam, not
    at the begining of the entire Process.

    """
    # * Dispatching which sub functions gets to do the action creation
    assembly_method = process.assembly.get_assembly_method(beam_id)
    if len(process.assembly.get_joint_ids_with_tools_for_beam(beam_id)) == 0:
        result = _create_actions_for_no_tools(process, beam_id, verbose)
    elif assembly_method == BeamAssemblyMethod.CLAMPED:
        result = _create_actions_for_clamped(process, beam_id, verbose)
    elif assembly_method == BeamAssemblyMethod.SCREWED_WITH_GRIPPER or assembly_method == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
        result = _create_actions_for_screwed(process, beam_id, verbose)

    # * Assign Actions Numbers
    if result in ComputationalResult.ValidResults:
        assign_action_numbers(process, beam_id, verbose)
    return result


def _create_actions_for_no_tools(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, Optional[bool]) -> None
    """ Creating Action objects (process.actions)
    for beams with no tools regardless of their assigned Assembly Method

    This is implemented as a gripper pick and place scenario.
    """

    assembly = process.assembly  # type: Assembly
    actions = []  # type: List[Action]
    seq_n = assembly.sequence.index(beam_id)

    if verbose:
        print("Beam %s" % beam_id)

    # * Operator Load Beam
    act = LoadBeamAction(seq_n, 0, beam_id)
    actions.append(act)

    # * Action to attach gripper
    gripper_type = assembly.get_beam_attribute(beam_id, "gripper_type")
    gripper_id = assembly.get_beam_attribute(beam_id, "gripper_id")
    act = PickGripperFromStorageAction(seq_n, 0, gripper_type, gripper_id)
    actions.append(act)

    # * Action to pick beam
    act = PickBeamWithGripperAction(seq_n, 0, beam_id, gripper_id)
    actions.append(act)

    # * Syncronized clamp and move beam action
    act = BeamPlacementWithoutClampsAction(seq_n, 0, beam_id, gripper_id)
    actions.append(act)

    # * Return gripper
    act = PlaceGripperToStorageAction(seq_n, 0, gripper_type, gripper_id)
    actions.append(act)

    # Print out newly added actions and return
    if verbose:
        for act in actions:
            print('|- ' + act.__str__())

    process.assembly.set_beam_attribute(beam_id, 'actions', actions)
    return ComputationalResult.ValidCanContinue


def _create_actions_for_clamped(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, Optional[bool]) -> None
    """ Creating Action objects (process.actions)
    for beams with `BeamAssemblyMethod.CLAMPED`.

    This is specific to the general action framework for a clamp and gripper assembly streategy.
    This is specific to a single robot / multiple clamp and gripper scenario.
    """

    assembly = process.assembly  # type: Assembly
    actions = []  # type: List[Action]
    seq_n = assembly.sequence.index(beam_id)

    if verbose:
        print("Beam %s" % beam_id)

    # * Actions to move clamps from storage to structure
    joint_id_of_clamps = list(assembly.get_joint_ids_with_tools_for_beam(beam_id))
    clamp_ids = [assembly.get_joint_attribute(joint_id, 'tool_id') for joint_id in joint_id_of_clamps]

    for joint_id in joint_id_of_clamps:
        tool_type = assembly.get_joint_attribute(joint_id, 'tool_type')
        tool_id = assembly.get_joint_attribute(joint_id, 'tool_id')
        actions.append(PickClampFromStorageAction(seq_n, 0, tool_type, tool_id))
        actions.append(PlaceClampToStructureAction(seq_n, 0, joint_id, tool_type, tool_id))

    # * Operator Load Beam
    act = LoadBeamAction(seq_n, 0, beam_id)
    actions.append(act)

    # * Actions to attach gripper
    gripper_type = assembly.get_beam_attribute(beam_id, "gripper_type")
    gripper_id = assembly.get_beam_attribute(beam_id, "gripper_id")
    act = PickGripperFromStorageAction(seq_n, 0, gripper_type, gripper_id)
    actions.append(act)

    # pick place beam
    act = PickBeamWithGripperAction(seq_n, 0, beam_id, gripper_id)
    actions.append(act)

    # Syncronized clamp and move beam action
    act = BeamPlacementWithClampsAction(seq_n, 0, beam_id, joint_id_of_clamps, gripper_id, clamp_ids)
    actions.append(act)

    # return gripper
    act = PlaceGripperToStorageAction(seq_n, 0, gripper_type, gripper_id)
    actions.append(act)

    # remove clamps from structure to storage
    for joint_id in reversed(joint_id_of_clamps):
        tool_type = assembly.get_joint_attribute(joint_id, 'tool_type')
        tool_id = assembly.get_joint_attribute(joint_id, 'tool_id')
        actions.append(PickClampFromStructureAction(joint_id=joint_id, tool_type=tool_type, tool_id=tool_id))
        actions.append(PlaceClampToStorageAction(tool_type=tool_type, tool_id=tool_id))

    # Print out newly added actions and return
    if verbose:
        for act in actions:
            print('|- ' + act.__str__())

    process.assembly.set_beam_attribute(beam_id, 'actions', actions)
    return ComputationalResult.ValidCanContinue


def _create_actions_for_screwed(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, Optional[bool]) -> None
    """ Creating Action objects (process.actions)
    for beams with `BeamAssemblyMethod.SCREWED` and .

    This is specific to the general action framework for a clamp and gripper assembly streategy.
    This is specific to a single robot / multiple clamp and gripper scenario.
    """

    # process.assembly.set_beam_attribute(beam_id, 'actions', [])
    # return ComputationalResult.ValidCanContinue

    assembly = process.assembly  # type: Assembly
    actions = []  # type: List[Action]
    assembly_method = process.assembly.get_assembly_method(beam_id)
    seq_n = assembly.sequence.index(beam_id)

    # * Pick Gripper or Screwdriver
    gripper_type = assembly.get_beam_attribute(beam_id, "gripper_type")
    gripper_id = assembly.get_beam_attribute(beam_id, "gripper_id")
    if assembly_method == BeamAssemblyMethod.SCREWED_WITH_GRIPPER:
        # * Action to Pick Gripper From Storage
        act = PickGripperFromStorageAction(tool_type=gripper_type, tool_id=gripper_id)
        actions.append(act)
    else:
        # * Action to Pick Screwdriver From Storage
        act = PickScrewdriverFromStorageAction(tool_type=gripper_type, tool_id=gripper_id)
        actions.append(act)

    # * Gripper Approach Beam Storage
    act = GenericGripperApproachBeamPickupAction(beam_id=beam_id, gripper_id=gripper_id)
    actions.append(act)

    # * Operator Load Beam
    actions.append(LoadBeamAction(beam_id=beam_id))
    actions.append(CloseGripperOnBeamAction(beam_id=beam_id, gripper_id=gripper_id))

    # * Lift Beam and Rotate , Operator Attach Screwdriver
    joint_ids = list(assembly.get_joint_ids_with_tools_for_beam(beam_id))
    tool_ids = [assembly.get_joint_attribute(joint_id, 'tool_id') for joint_id in joint_ids]

    act = PickAndRotateBeamForAttachingScrewdriverAction(beam_id=beam_id, gripper_id=gripper_id)
    actions.append(act)

    for joint_id, tool_id in zip(joint_ids, tool_ids):
        if tool_id == gripper_id:
            continue  # Skipping the screwdriver that is acting as gripper
        tool_type = assembly.get_joint_attribute(joint_id, 'tool_type')
        actions.append(OperatorAttachScrewdriverAction(seq_n, 0, beam_id, joint_id, tool_type, tool_id, 'assembly_wcf_screwdriver_attachment_pose'))

    # * Actions to Assemble
    if assembly_method == BeamAssemblyMethod.SCREWED_WITH_GRIPPER:

        # * Action to Place Beam and Screw it with Screwdriver, not including retract
        act = AssembleBeamWithScrewdriversAction(beam_id=beam_id, joint_ids=joint_ids, gripper_id=gripper_id, screwdriver_ids=tool_ids)
        actions.append(act)

        # * Action to Open Gripper and Retract Screw while robot-sync backing away
        act = RetractGripperFromBeamAction(beam_id=beam_id, gripper_id=gripper_id, additional_attached_objects=tool_ids)
        actions.append(act)

        # * Action to Open Gripper and Retract Screw while robot-sync backing away
        act = PlaceGripperToStorageAction(tool_type=gripper_type, tool_id=gripper_id)
        actions.append(act)

    else:  # SCREWED_WITHOUT_GRIPPER
        flying_tools_id = [t_id for t_id in tool_ids if t_id != gripper_id]

        # * Action to Place Beam and Screw it with Screwdriver, not including retract
        act = AssembleBeamWithScrewdriversAction(beam_id=beam_id, joint_ids=joint_ids, gripper_id=gripper_id, screwdriver_ids=tool_ids)
        actions.append(act)

        # * Action to retract the Screwdriver that was acting as gripper and place it to storage
        grasping_joint_id = process.assembly.get_grasping_joint_id(beam_id)
        actions.append(RetractScrewdriverFromBeamAction(beam_id=beam_id, joint_id=grasping_joint_id, tool_id=gripper_id, additional_attached_objects=flying_tools_id))

        actions.append(PlaceScrewdriverToStorageAction(tool_type=gripper_type, tool_id=gripper_id))

    # * Actions to Detach Remaining Screwdriver from the Structure.
    for joint_id, tool_id in reversed(list(zip(joint_ids, tool_ids))):
        # Skip the screwdriver that acted as gripper
        if assembly_method == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
            if tool_id == gripper_id:
                continue

        tool_type = assembly.get_joint_attribute(joint_id, 'tool_type')

        # * Action to Dock with Screwdriver at Storage
        actions.append(DockWithScrewdriverAction(joint_id=joint_id, tool_position='screwdriver_assembled_attached', tool_type=tool_type, tool_id=tool_id))

        # * Action to retract Screwdriver and place it to storage
        actions.append(RetractScrewdriverFromBeamAction(beam_id=beam_id, joint_id=joint_id, tool_id=tool_id))

        actions.append(PlaceScrewdriverToStorageAction(tool_type=tool_type, tool_id=tool_id))

    # Print out newly added actions and return
    if verbose:
        for act in actions:
            print('|- ' + act.__str__())

    process.assembly.set_beam_attribute(beam_id, 'actions', actions)
    return ComputationalResult.ValidCanContinue


def create_movements_from_action(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, Optional[bool]) -> None
    """Expand a given beam's Actions into low-level Movements.
    State Diff is also created for each resulting Movement.

    The create_movements() functions are are located within each of the Action class.
    They copy tool_ids / target frames from the Action classes,
    occationally performing transformation along the tool chain.
    """
    for action in process.get_actions_by_beam_id(beam_id):
        action.create_movements(process)
        action.assign_movement_ids()
        for mov_n, movement in enumerate(action.movements):
            if verbose:
                print("Processing Seq %i Action %i Movement %i: %s" % (action.seq_n, action.act_n, mov_n, movement.tag))
            movement.create_state_diff(process)
    return ComputationalResult.ValidCanContinue


#############################################################
# Algorithms that concerns the entire process
# Require sequential parsing of all Actions in the process
#############################################################

def assign_action_numbers(process, beam_id, verbose=True):
    # type: (RobotClampAssemblyProcess, str, Optional[bool]) -> None
    """Assigns seq_n (beam ordering) act_n (sequential number) to actions in one Beam.

    First Action in a Beam gets action.act_n = 0.
    `seq_n` is the index of the beam_id in assembly.sequence

    Movement.movement_id are also updated.
    """
    act_n = 0
    seq_n = process.assembly.sequence.index(beam_id)
    for action in process.get_actions_by_beam_id(beam_id):
        action.act_n = act_n
        action.seq_n = seq_n
        act_n += 1
        action.assign_movement_ids()


def assign_unique_action_numbers(process, verbose=True):
    # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
    """Assigns unique act_n to all actions across the entire Process of all Beams.
    action.act_n starts from 0.

    Movement.movement_id are also updated.
    """
    act_n = 0
    for action in process.actions:
        action.act_n = act_n
        act_n += 1
        action.assign_movement_ids()


def optimize_actions_place_pick_gripper(process, verbose=True):
    # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
    """
    If PlaceGripperToStorageAction and followed by PickGripperFromStorageAction.
    They can be canceled out if:
    - They have the same action.tool_type
    - There are no actions in between of type AttachToolAction
    """
    to_be_removed = []  # type: List[int]
    found_PlaceGripperToStorageAction = None  # type: Optional[int]

    for i, action in enumerate(process.actions):
        if verbose:
            print("Action %s : %s" % (i, action))

        # Looking for PlaceGripperToStorageAction
        if isinstance(action, PlaceGripperToStorageAction):
            found_PlaceGripperToStorageAction = i
            continue

        if found_PlaceGripperToStorageAction is not None:
            # Looking for PickGripperFromStorageAction
            if isinstance(action, PickGripperFromStorageAction):
                # Compare action.tool_type to be equal
                if process.actions[found_PlaceGripperToStorageAction].tool_type == action.tool_type:
                    to_be_removed.append(found_PlaceGripperToStorageAction)
                    to_be_removed.append(i)
                    if verbose:
                        print("- - Action (%s) and (%s) will be removed." % (found_PlaceGripperToStorageAction, i))
                found_PlaceGripperToStorageAction = None  # Reset
            # Cancel current search if AttachToolAction is encountered
            if isinstance(action, AttachToolAction):
                found_PlaceGripperToStorageAction = None  # Reset
            continue
    if verbose:
        print("Actions removed: %s" % to_be_removed)
    to_be_removed = set(to_be_removed)
    process.actions = [action for i, action in enumerate(process.actions) if i not in to_be_removed]


def optimize_actions_place_pick_clamp(process, verbose=True):
    # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
    """
    If PlaceClampToStorageAction is followed by PickClampFromStorageAction.
    They can be canceled out if:
    - They have the same action.tool_id
    - There are no actions in between of type AttachToolAction
    """
    to_be_removed = []  # type: List[int]
    found_PlaceClampToStorageAction = None  # type: Optional[int]

    for i, action in enumerate(process.actions):
        if verbose:
            print("Action %s : %s" % (i, action))

        # Looking for PlaceGripperToStorageAction
        if isinstance(action, PlaceClampToStorageAction):
            found_PlaceClampToStorageAction = i
            continue

        if found_PlaceClampToStorageAction is not None:
            # Looking for PickClampFromStorageAction
            if isinstance(action, PickClampFromStorageAction):
                # Compare action.tool_id to be equal
                if process.actions[found_PlaceClampToStorageAction].tool_id == action.tool_id:
                    to_be_removed.append(found_PlaceClampToStorageAction)
                    to_be_removed.append(i)
                    if verbose:
                        print("- - Action (%s) and (%s) will be removed." % (found_PlaceClampToStorageAction, i))
                found_PlaceClampToStorageAction = None  # Reset
            # Cancel current search if AttachToolAction is encountered
            if isinstance(action, AttachToolAction):
                found_PlaceClampToStorageAction = None  # Reset
            continue
    if verbose:
        print("Actions removed: %s" % to_be_removed)
    to_be_removed = set(to_be_removed)
    process.actions = [action for i, action in enumerate(process.actions) if i not in to_be_removed]


def recompute_initial_state(process, verbose=False):
    # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
    """Compute the initial scene state. This is the begining of the assembly process

    State Change
    ------------
    This functions sets the following keys in process.initial_state:
    - (beam_id, 'f')
    - (beam_id, 'a')
    - (tool_id, 'f')
    - (tool_id, 'a')
    - (tool_id, 'c')
    - ('robot', 'c')
    - ('tool_changer', 'a')

    Note that ('robot', 'f') and ('tool_changer', 'f') are computed only if process.robot_model is loaded (FK of config needed).

    Recomputation of the initial state is necessary if
    - tool's definitions or their storage positions are changed.
    - robot initial configuration is changed
    """
    process.initial_state = SceneState(process)
    assembly = process.assembly

    # Beams are all in their storage position
    for beam_id in assembly.sequence:
        process.dependency.compute(beam_id, process.compute_storeage_frame)
        process.initial_state[(beam_id, 'f')] = assembly.get_beam_attribute(beam_id, 'assembly_wcf_storage')
        process.initial_state[(beam_id, 'a')] = False

    # Tools (Clamps, Grippers) are all in their storage position
    for tool_id in process.tool_ids:
        tool = process.tool(tool_id)
        tool.close_gripper()
        # Clamps should have a open clamp jaw state in the begining.
        if isinstance(tool, Clamp):
            tool.open_clamp()
        process.initial_state[(tool_id, 'f')] = tool.tool_storage_frame
        process.initial_state[(tool_id, 'a')] = False
        process.initial_state[(tool_id, 'c')] = tool._get_kinematic_state()

    # Robot is in its initial position
    # TODO robot_state.current_frame should be an FK of the robot form the robot_initial_config
    process.initial_state[('robot', 'c')] = process.robot_initial_config

    # Tool Chagner is attached to the robot
    process.initial_state[('tool_changer', 'a')] = True

    # Perform FK to find the tool_changer location based on initial robot config.
    # This can only happen if the RobotModel is laoded.
    if process.robot_model is not None:
        # * Note that FK result is not scaled by default.
        configuration = process.robot_initial_config.scaled(1000)
        robot_frame = process.robot_model.forward_kinematics(configuration, process.ROBOT_END_LINK)
        if verbose:
            print("Robot Frame computed with FK...")
            print("Robot Config: %s" % configuration)
            print("Robot Frame: %s" % robot_frame)
        process.initial_state[('robot', 'f')] = robot_frame
        process.initial_state[('tool_changer', 'f')] = robot_frame
    else:
        process.initial_state[('tool_changer', 'f')] = Frame([0, 0, 0], [1, 0, 0], [0, 1, 0])

#############################
# Debug and Testing functions
#############################


def debug_print_process_actions_movements(process, file_path=None):
    # type: (RobotClampAssemblyProcess, Optional[str]) -> None
    f = None
    seq_number = None
    if file_path is not None:
        f = open(file_path, 'w')
    for i, action in enumerate(process.actions):
        # Print the separator between each beam
        if action.seq_n != seq_number:
            seq_number = action.seq_n
            line = "\nBeam %s assembly\n" % process.assembly.sequence[seq_number]
            if file_path is not None:
                f.write(line)
            else:
                print(line)

        # Print Action Name
        line = "|- Action %s (%s): %s\n" % (i, action.__class__.__name__, action)
        if file_path is not None:
            f.write(line)
        else:
            print(line)

        # Print Movement Name
        for j, movement in enumerate(action.movements):

            # Print Priority String
            if movement.planning_priority < 0:
                priority_string = 'x'
            else:
                priority_string = '%s' % movement.planning_priority

            line = "|  |- <%s> Movement %s (%s): %s\n" % (priority_string, j, movement.__class__.__name__, movement.tag)
            if file_path is not None:
                f.write(line)
            else:
                print(line)
    if file_path is not None:
        f.close()


def test_process_pathPlan(json_path_in, json_path_out):
    #########################################################################
    # Load process
    #########################################################################

    import jsonpickle
    f = open(json_path_in, 'r')
    json_str = f.read()
    process = jsonpickle.decode(json_str, keys=True)  # type: RobotClampAssemblyProcess
    f.close()

    #########################################################################
    # Connect to path planning backend and initialize robot parameters
    #########################################################################

    pp = RFLPathPlanner(None)
    pp.connect_to_ros_planner("192.168.183.129")
    robot = pp.robot  # type: compas_fab.robots.Robot

    # Add static collision mesh to planning scene
    # TODO: FLoor and other meshes

    current_configuration = pp.rfl_timber_start_configuration()
    print(current_configuration)
    #########################################################################
    # Sequential path planning
    #########################################################################

    # Action 8 to 19 is realted to beam b2
    action = process.actions[9]
    for i, movement in enumerate(action.movements):
        print("Movement (%s) - %s" % (i, movement))
        if isinstance(movement, RoboticFreeMovement):
            # Add already built beams to planning scene

            # Attach Tool and Beam to robot

            # Prepare Starting Config and Goal constraints
            # start_configuration = Configuration.from_revolute_values([0, 4.295, 0, -3.327, 4.755, 0.])
            trajectory = pp.plan_free_motion(movement.target_frame)
            if trajectory:
                print("> > Free Motion Planned (%s pts, %ssecs)" % (len(trajectory.points), trajectory.time_from_start))

    pp.ros_client.close()

    #########################################################################
    # Save Results
    #########################################################################

    f = open(json_path_out, 'w')
    f.write(jsonpickle.encode(process, keys=True))
    f.close()


if __name__ == "__main__":

    pass
