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

###############################################
# Beam level functions (as used in auto update)
###############################################


def assign_tool_id_to_beam_joints(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, Optional[bool]) -> ComputationalResult
    """Assign available tool_ids to joints that require assembly tool.
    Based on the joint attribute `tool_type`
    """
    available_tools = sorted(process.tools, key=lambda tool: tool.name)
    something_failed = False
    something_changed = False

    def pop_tool_by_type(type_name):
        for tool in available_tools:
            if tool.type_name == type_name:
                available_tools.remove(tool)
                return tool
        return None

    for joint_id in process.assembly.get_joint_ids_with_tools_for_beam(beam_id):
        tool_type = process.assembly.get_joint_attribute(joint_id, 'tool_type')
        tool = pop_tool_by_type(tool_type)
        if tool is not None:
            if verbose:
                print("|- Assigning %s(%s) to Joint (%s)" % (tool_type, tool.name, joint_id))
            previous_tool_id = process.assembly.get_joint_attribute(joint_id, 'tool_id')
            if previous_tool_id != tool.name:
                process.assembly.set_joint_attribute(joint_id, 'tool_id', tool.name)
                something_changed = True
        else:
            print("Warning: Tool Type %s Not available for joint %s" % (tool_type, joint_id))
            something_failed = True

    # Return results
    if something_failed:
        return ComputationalResult.ValidCannotContinue
    else:
        if something_changed:
            return ComputationalResult.ValidCanContinue
        else:
            return ComputationalResult.ValidNoChange


def create_actions_from_sequence(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, Optional[bool]) -> None
    """ Creating Action objects (process.actions) from process.sequence

    Beam attribute 'assembly_method' should be assigned prior.
    Joint attribute 'tool_type' and 'tool_id' should be assigned.
    """
    assembly_method = process.assembly.get_assembly_method(beam_id)
    if assembly_method == BeamAssemblyMethod.CLAMPED:
        return _create_actions_for_clamped(process, beam_id, verbose)
    else:
        return _create_actions_for_screwed(process, beam_id, verbose)


def _create_actions_for_clamped(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, Optional[bool]) -> None
    """ Creating Action objects (process.actions)
    for beams with `BeamAssemblyMethod.CLAMPED`.

    This is specific to the general action framework for a clamp and gripper assembly streategy.
    This is specific to a single robot / multiple clamp and gripper scenario.
    """

    assembly = process.assembly  # type: Assembly
    actions = []  # type: List[Action]

    # Action number and sequence number
    def act_n(reset=False):
        # Fuction to keep count of act_n
        if hasattr(act_n, 'counter'):
            act_n.counter += 1
        else:
            act_n.counter = 0
        return act_n.counter
    seq_n = assembly.sequence.index(beam_id)

    beam = assembly.beam(beam_id)  # type: Beam
    if verbose:
        print("Beam %s" % beam_id)

    # move clamps from storage to structure
    joint_id_of_clamps = list(assembly.get_joint_ids_with_tools_for_beam(beam_id))
    clamp_ids = [assembly.get_joint_attribute(joint_id, 'tool_id') for joint_id in joint_id_of_clamps]

    for joint_id in joint_id_of_clamps:
        tool_type = assembly.get_joint_attribute(joint_id, 'tool_type')
        tool_id = assembly.get_joint_attribute(joint_id, 'tool_id')
        actions.append(PickClampFromStorageAction(seq_n, act_n(), tool_type, tool_id))
        if verbose:
            print('|- ' + actions[-1].__str__())
        actions.append(PlaceClampToStructureAction(seq_n, act_n(), joint_id, tool_type, tool_id))
        if verbose:
            print('|- ' + actions[-1].__str__())

        #if verbose: print ("|- Detatch Clamp at Joint %s-%s" % clamp)

    # attach gripper
    gripper_type = assembly.get_beam_attribute(beam_id, "gripper_type")
    gripper_id = assembly.get_beam_attribute(beam_id, "gripper_id")
    act = PickGripperFromStorageAction(seq_n, act_n(), gripper_type, gripper_id)
    actions.append(act)
    if verbose:
        print('|- ' + act.__str__())

    # pick place beam
    act = BeamPickupAction(seq_n, act_n(), beam_id, gripper_id)
    actions.append(act)
    if verbose:
        print('|- ' + act.__str__())

    # Syncronized clamp and move beam action

    if len(joint_id_of_clamps) > 0:
        act = BeamPlacementWithClampsAction(seq_n, act_n(), beam_id, joint_id_of_clamps, gripper_id, clamp_ids)
        actions.append(act)
        if verbose:
            print('|- ' + act.__str__())
    else:
        act = BeamPlacementWithoutClampsAction(seq_n, act_n(), beam_id, gripper_id)
        actions.append(act)
        if verbose:
            print('|- ' + act.__str__())

    # return gripper
    act = PlaceGripperToStorageAction(seq_n, act_n(), gripper_type, gripper_id)
    actions.append(act)
    if verbose:
        print('|- ' + act.__str__())

    # remove clamps from structure to storage
    for joint_id in reversed(joint_id_of_clamps):
        tool_type = assembly.get_joint_attribute(joint_id, 'tool_type')
        tool_id = assembly.get_joint_attribute(joint_id, 'tool_id')
        actions.append(PickClampFromStructureAction(seq_n, act_n(), joint_id, tool_type, tool_id))
        if verbose:
            print('|- ' + actions[-1].__str__())
        actions.append(PlaceClampToStorageAction(seq_n, act_n(), tool_type, tool_id))
        if verbose:
            print('|- ' + actions[-1].__str__())

    process.assembly.set_beam_attribute(beam_id, 'actions', actions)
    # Return results
    return ComputationalResult.ValidCanContinue


def _create_actions_for_screwed(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, Optional[bool]) -> None
    """ Creating Action objects (process.actions)
    for beams with `BeamAssemblyMethod.CLAMPED`.

    This is specific to the general action framework for a clamp and gripper assembly streategy.
    This is specific to a single robot / multiple clamp and gripper scenario.
    """
    assembly = process.assembly  # type: Assembly
    actions = []  # type: List[Action]
    assembly_method = process.assembly.get_assembly_method(beam_id)

    # Action number and sequence number
    def act_n(reset=False):
        # Fuction to keep count of act_n
        if hasattr(act_n, 'counter'):
            act_n.counter += 1
        else:
            act_n.counter = 0
        return act_n.counter
    seq_n = assembly.sequence.index(beam_id)

    # * Operator Load Beam
    act = LoadBeamAction(seq_n, act_n(), beam_id)
    actions.append(act)

    # * Actions to Attach Screwdriver
    joint_ids = list(assembly.get_joint_ids_with_tools_for_beam(beam_id))
    tool_ids = [assembly.get_joint_attribute(joint_id, 'tool_id') for joint_id in joint_ids]

    for joint_id, tool_id in zip(joint_ids, tool_ids):
        tool_type = assembly.get_joint_attribute(joint_id, 'tool_type')
        actions.append(OperatorAttachScrewdriverAction(seq_n, act_n(), beam_id, joint_id, tool_type, tool_id))

    if assembly_method == BeamAssemblyMethod.SCREWED_WITH_GRIPPER:
        # * Action to Pick Gripper From Storage
        gripper_type = assembly.get_beam_attribute(beam_id, "gripper_type")
        gripper_id = assembly.get_beam_attribute(beam_id, "gripper_id")
        act = PickGripperFromStorageAction(seq_n, act_n(), gripper_type, gripper_id)
        actions.append(act)

        # * Action to pick beam with Gripper
        act = PickBeamWithAttachedToolsAction(seq_n, act_n(), beam_id, gripper_id, tool_ids) # ! Todo
        actions.append(act)

        # * Action to Place Beam with Screwdriver and Gripper
        act = AssembleBeamWithScrewdriversAndGripperAction(seq_n, act_n(), beam_id, joint_ids, tool_ids, gripper_id) # ! Todo
        actions.append(act)

        # * Action to Open Gripper and Retract Screw while robot-sync backing away
        act = DetachAndRetractScrewdriverAction(seq_n, act_n(), beam_id, joint_ids, tool_ids, gripper_id) # ! Todo
        actions.append(act)

        # * Action to Open Gripper and Retract Screw while robot-sync backing away
        act = DetachAndRetractScrewdriverAction(seq_n, act_n(), beam_id, joint_ids, tool_ids, gripper_id) # ! Todo
        actions.append(act)


    # Actions to Assemble

    # Actions to Detach Screwdriver



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
    """Assigns act_n to actions in one Beams.
    First Action in a Beam gets action.act_n = 0.

    Movement.movement_id are also updated.
    """
    act_n = 0
    for action in process.get_actions_by_beam_id(beam_id):
        action.act_n = act_n
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

    test_process_prepathplan(
        "examples/process_design_example/frame_ortho_lap_joints_no_rfl_process.json",
        "examples/process_design_example/frame_ortho_lap_joints_no_rfl_pathplan.json",
    )

    test_process_pathPlan(
        "examples/process_design_example/frame_ortho_lap_joints_no_rfl_process.json",
        "examples/process_design_example/frame_ortho_lap_joints_no_rfl_pathplan.json",
    )
    # Result Goal 1: Visualize action, movement, trajectory by visualizing scene objects and robots.

    # Result Goal 2: Execute movements

    pass
