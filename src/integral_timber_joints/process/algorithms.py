try:
    from typing import List, Dict, Tuple, Optional
except:
    pass

from integral_timber_joints.process.action import *
from integral_timber_joints.process.target import *
from integral_timber_joints.process import RobotClampAssemblyProcess, PathPlanner
from integral_timber_joints.process.movement import *
from integral_timber_joints.assembly import Assembly
from integral_timber_joints.geometry import Beam, Joint
from integral_timber_joints.tools import Tool, Clamp, Gripper

###############################################
# Algorithms that concerns the entire process
# Many of which require sequential parsing
# #############################################

def create_actions_from_sequence(process, verbose = True):
    # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
    """ Creating Action objects (process.actions) from process.sequence
    This is specific to the general action framework for a clamp and gripper assembly streategy.
    This is specific to a single robot / multiple clamp and gripper scenario.
    """

    assembly = process.assembly # type: Assembly
    process.actions = []
    actions = process.actions # type: List[Action]

    for beam_id in assembly.sequence:
        beam = assembly.beam(beam_id) # type: Beam
        if verbose: print ("Beam %s" % beam_id)
        actions.append(LoadBeamAction(beam_id))
        if verbose: print ('|- ' + actions[-1].__str__())

        # move clamps from storage to structure
        # joint_id_of_clamps = assembly.get_joints_of_beam_connected_to_already_built(beam_id)

        joint_id_of_clamps = [(neighbour_id, beam_id) for neighbour_id in assembly.get_already_built_neighbors(beam_id) if assembly.get_joint_attribute((neighbour_id, beam_id), "is_clamp_attached_side")]

        for joint_id in joint_id_of_clamps:
            clamp_type = assembly.get_joint_attribute(joint_id, "clamp_type")
            actions.append(PickClampFromStorageAction(clamp_type))
            if verbose: print ('|- ' + actions[-1].__str__())
            actions.append(PlaceClampToStructureAction(joint_id, clamp_type))
            if verbose: print ('|- ' + actions[-1].__str__())

            #if verbose: print ("|- Detatch Clamp at Joint %s-%s" % clamp)

        # attach gripper
        gripper_type = assembly.get_beam_attribute(beam_id, "gripper_type")
        actions.append(PickGripperFromStorageAction(gripper_type))
        if verbose: print ('|- ' + actions[-1].__str__())

        # pick place beam
        actions.append(PickBeamFromStorageAction(beam_id))
        if verbose: print ('|- ' + actions[-1].__str__())

        #Syncronized clamp and move beam action
        if len(joint_id_of_clamps) > 0:
            actions.append(PlaceBeamWithClampsAction(beam_id, joint_id_of_clamps))
            if verbose: print ('|- ' + actions[-1].__str__())
        else:
            actions.append(PlaceBeamWithoutClampsAction(beam_id))
            if verbose: print ('|- ' + actions[-1].__str__())

        # return gripper
        actions.append(PlaceGripperToStorageAction(gripper_type))
        if verbose: print ('|- ' + actions[-1].__str__())

        # remove clamps from structure to storage
        for joint_id in joint_id_of_clamps:
            clamp_type = assembly.get_joint_attribute(joint_id, "clamp_type")
            actions.append(PickClampFromStructureAction(joint_id, clamp_type))
            if verbose: print ('|- ' + actions[-1].__str__())
            actions.append(PlaceClampToStorageAction(clamp_type))
            if verbose: print ('|- ' + actions[-1].__str__())

def assign_tools_to_actions(process, verbose = True):
    # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
    """ Assign Clamp and Gripper instances to actions
    based on the gripper_type and tool_type attributes.

    The functions also checks the consistancy of tool attach/detach actions:
    - no beam is attached to tool before pick-beam
    - beam is attached to tool before place-beam
    - robot is holding a tool before place-tool
    - robot is not holding a tool before pick-tool
    - no beam is attached to tool before place-tool

    """
    # Variables for a procedural simulation to ensure process consistancy
    tools_in_storage = [c for c in process.clamps] + [g for g in process.grippers]
    beam_id_on_pickup_station = None # type: Optional[str]

    tool_at_robot = None # type: Optional[Tool]
    beam_id_at_robot = None # type: Optional[str]

    clamps_on_structure = {} # type: Dict[Tuple[str, str], Tool]

    def PopToolFromStorage(tool_type):
        # type: (str) -> Tool
        for tool in reversed(tools_in_storage):
            if (tool.type_name == tool_type):
                tools_in_storage.remove(tool)
                return tool
        raise Exception("Tool Not Found in Storage Error: tool_type = %s"% tool_type)

    for action in process.actions:
        if verbose: print(action)
        if isinstance(action, PickClampFromStorageAction) or isinstance(action, PickGripperFromStorageAction):
            if tool_at_robot is not None: # Consistancy check to make sure tool is not already attached.
                raise Exception("Unable to attach tool becuase %s is already attached to robot" % tool_at_robot)
            tool = PopToolFromStorage(action.tool_type)
            tool_at_robot = tool
            action.tool_id = tool.name

        if isinstance(action, PlaceClampToStorageAction) or isinstance(action, PlaceGripperToStorageAction):
            if tool_at_robot is None: # Consistancy check to make sure some tool is attached.
                raise Exception("Unable to detach tool becuase No Tool is attached to robot")
            tool = tool_at_robot
            if (action.tool_type != tool_at_robot.type_name):
                raise Exception("Detach action action.tool_type (%s) inconsistant with tool_at_robot.type_name (%s)" % (action.tool_type, tool_at_robot.type_name))
            action.tool_id = tool.name
            tools_in_storage.append(tool)
            tool_at_robot = None

        if isinstance(action, PickClampFromStructureAction):
            if tool_at_robot is not None: # Consistancy check to make sure tool is not already attached.
                raise Exception("Unable to attach tool becuase %s is already attached to robot" % tool_at_robot)
            tool = clamps_on_structure[action.joint_id]
            tool_at_robot = tool
            action.tool_id = tool.name

        if isinstance(action, PlaceClampToStructureAction):
            if tool_at_robot is None: # Consistancy check to make sure some tool is attached.
                raise Exception("Unable to detach tool becuase No Tool is attached to robot")
            tool = tool_at_robot
            if (action.tool_type != tool_at_robot.type_name):
                raise Exception("Detach action action.tool_type (%s) inconsistant with tool_at_robot.type_name (%s)" % (action.tool_type, tool_at_robot.type_name))
            action.tool_id = tool.name
            process.assembly.set_joint_attribute(action.joint_id, 'clamp_id', tool.name)
            clamps_on_structure[action.joint_id] = tool
            tool_at_robot = None

        if isinstance(action, LoadBeamAction):
            if beam_id_on_pickup_station is not None:
                raise Exception("Unable to load new beam (%s) becuase beam (%s) is still occupying pick-up station" % (action.beam_id, beam_id_on_pickup_station))
            beam_id_on_pickup_station = action.beam_id

        if isinstance(action, PickBeamFromStorageAction):
            if tool_at_robot is None:
                raise Exception("Unable to pick beam becuase No Tool is attached to robot")
            if beam_id_on_pickup_station != action.beam_id:
                raise Exception("Unable to pick beam becuase beam_id_on_pickup_station (%s) is inconsistant with action.beam_id (%s)" % (beam_id_on_pickup_station, action.beam_id))
            beam_id_on_pickup_station = None
            beam_id_at_robot = action.beam_id
            gripper_id = tool_at_robot.name
            action.gripper_id = gripper_id
            process.assembly.set_beam_attribute(action.beam_id, 'gripper_id', gripper_id)

        if isinstance(action, PlaceBeamWithoutClampsAction):
            if tool_at_robot is None:
                raise Exception("Unable to place beam becuase No Tool is attached to robot")
            if beam_id_at_robot is None:
                raise Exception("Unable to place beam becuase No Beam is attached to robot")
            if beam_id_at_robot != action.beam_id:
                raise Exception("Unable to place beam becuase beam_id_at_robot (%s) is inconsistant with action.beam_id (%s)" % (beam_id_at_robot, action.beam_id))
            beam_id_at_robot = None
            action.gripper_id = tool_at_robot.name

        if isinstance(action, PlaceBeamWithClampsAction):
            if tool_at_robot is None:
                raise Exception("Unable to place beam becuase No Tool is attached to robot")
            if beam_id_at_robot is None:
                raise Exception("Unable to place beam becuase No Beam is attached to robot")
            if beam_id_at_robot != action.beam_id:
                raise Exception("Unable to place beam becuase beam_id_at_robot (%s) is inconsistant with action.beam_id (%s)" % (beam_id_at_robot, action.beam_id))
            beam_id_at_robot = None
            action.gripper_id = tool_at_robot.name
            for joint_id in action.joint_ids:
                if joint_id not in clamps_on_structure:
                    raise Exception("Joint (%s) require a clamp that was not placed" % joint_id)
            action.clamp_ids = [clamps_on_structure[joint_id].name for joint_id in action.joint_ids]

def optimize_actions_place_pick_gripper(process, verbose = True):
    # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
    """
    If PlaceGripperToStorageAction and followed by PickGripperFromStorageAction.
    They can be canceled out if:
    - They have the same action.tool_type
    - There are no actions in between of type AttachToolAction
    """
    to_be_removed = [] # type: List[int]
    found_PlaceGripperToStorageAction = None # type: Optional[int]

    for i, action in enumerate(process.actions):
        if verbose: print ("Action %s : %s" % (i, action))

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
                    if verbose: print("- - Action (%s) and (%s) will be removed." % (found_PlaceGripperToStorageAction, i))
                found_PlaceGripperToStorageAction = None # Reset
            # Cancel current search if AttachToolAction is encountered
            if isinstance(action, AttachToolAction):
                found_PlaceGripperToStorageAction = None # Reset
            continue
    if verbose: print ("Actions removed: %s" % to_be_removed)
    to_be_removed = set(to_be_removed)
    process.actions = [action for i, action in enumerate(process.actions) if i not in to_be_removed]

def optimize_actions_place_pick_clamp(process, verbose = True):
    # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
    """
    If PlaceClampToStorageAction is followed by PickClampFromStorageAction.
    They can be canceled out if:
    - They have the same action.tool_id
    - There are no actions in between of type AttachToolAction
    """
    to_be_removed = [] # type: List[int]
    found_PlaceClampToStorageAction = None # type: Optional[int]

    for i, action in enumerate(process.actions):
        if verbose: print ("Action %s : %s" % (i, action))

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
                    if verbose: print("- - Action (%s) and (%s) will be removed." % (found_PlaceClampToStorageAction, i))
                found_PlaceClampToStorageAction = None # Reset
            # Cancel current search if AttachToolAction is encountered
            if isinstance(action, AttachToolAction):
                found_PlaceClampToStorageAction = None # Reset
            continue
    if verbose: print ("Actions removed: %s" % to_be_removed)
    to_be_removed = set(to_be_removed)
    process.actions = [action for i, action in enumerate(process.actions) if i not in to_be_removed]

def create_movements_from_actions(process, verbose = True):
    # type: (RobotClampAssemblyProcess, Optional[bool]) -> None

    assembly = process.assembly # type: Assembly
    actions = process.actions # type: List[Action]

    for action in actions:
            action.create_movements(process)

def debug_print_process_actions_movements(process):
    for i, action in enumerate(process.actions):
        if isinstance(action, LoadBeamAction):
            print ("\nBeam %s assembly" % action.beam_id)
        print ("|- Action %s : %s" % (i, action))
        for j, movement in enumerate(action.movements):
            print ("|  |- Movement %s : %s" % (j, movement))

def test_process_prepathplan():
    #########################################################################
    # Load process
    #########################################################################

    import jsonpickle
    json_path = "tests/SequencingScene_01_mm.json"
    f = open(json_path, 'r')
    json_str = f.read()
    process = jsonpickle.decode(json_str, keys=True) # type: RobotClampAssemblyProcess
    f.close()

    #########################################################################
    # Pre computation
    #########################################################################

    # process.assign_clamp_type_to_joints()
    # for beam_id in process.assembly.sequence:
    #     process.compute_clamp_attachapproach_attachretract(beam_id, verbose = True)

    #########################################################################
    # From process.sequence, create actions (high level actions)
    #########################################################################

    print("\n> > > create_actions_from_sequence(process)\n")
    create_actions_from_sequence(process, verbose=False)
    print("\n> > > optimize_actions_place_pick_gripper(process)\n")
    optimize_actions_place_pick_gripper(process, verbose=False)
    print("\n> > > assign_tools_to_actions(process)\n")
    assign_tools_to_actions(process, verbose=False)
    print("\n> > > optimize_actions_place_pick_clamp(process)\n")
    optimize_actions_place_pick_clamp(process, verbose=False)

    #########################################################################
    # Loop through each action, create movements (low level movements)
    #########################################################################

    print("\n> > > create_movements_from_actions(process)\n")
    create_movements_from_actions (process)

    #########################################################################
    # List out actions and movements
    #########################################################################

    print("\n> > > debug_print_process_actions_movements(process)\n")
    debug_print_process_actions_movements(process)

    #########################################################################
    # Save process
    #########################################################################

    json_output_path = "tests/SequencingScene_02_mm.json"
    print("\n> > > Saving Process to %s \n" % json_output_path)
    f = open(json_output_path, 'w')
    f.write(jsonpickle.encode(process, keys=True))
    f.close()

def test_process_pathPlan():
    #########################################################################
    # Load process
    #########################################################################

    import jsonpickle
    json_path = "tests/SequencingScene_02_mm.json"
    f = open(json_path, 'r')
    json_str = f.read()
    process = jsonpickle.decode(json_str, keys=True) # type: RobotClampAssemblyProcess
    f.close()

    #########################################################################
    # Connect to path planning backend and initialize robot parameters
    #########################################################################
    from compas_fab.robots import Configuration
    from compas.geometry import Frame

    pp = PathPlanner(None)
    pp.connect_to_ros_planner("192.168.43.141")
    robot = pp.robot # type: compas_fab.robots.Robot
    robot.scale = 1000 # millimeters
    # Add static collision mesh to planning scene
    # TODO: FLoor and other meshes

    # Set robot home position
    # home = {
    #     "robot22_joint_EA_Z" : -4915,
    #     "robot22_joint_EA_Y" : -12237,
    #     "robot21_joint_EA_Z" : -4915,
    #     "robot21_joint_EA_Y" : 0.0,
    #     "bridge2_joint_EA_X" : 39805,
    #     "robot12_joint_EA_Z" : -4915,
    #     "robot12_joint_EA_Y" : -12237,
    #     "robot11_joint_EA_Z" : -4915,
    #     "robot11_joint_EA_Y" : 0.0,
    #     "bridge1_joint_EA_X" : 0.0}
    # values = []
    # types = []
    # joint_names = []

    # for joint in robot.get_configurable_joints():
    #     types.append(joint.type)
    #     joint_names.append(joint.name)
    #     if "EA" not in joint.name:
    #         values.append(0.0)
    #     else:
    #         values.append(home[joint.name])
    # home_configuration = Configuration(values, types, joint_names)

    start_configuration = Configuration(
    values = (
        0,
        0, -4915,
        0, 0, 0, 0, 0, 0,
        -12237, -4915,
        0, 0, 0, 0, 0, 0,
        38000,
        0, -4915,
        0, 0, 0, 0, 0, 0,
        -12237, -4915,
        0, 0, 0, 0, 0, 0
        ),
    types = (
        2,
        2, 2,
        0, 0, 0, 0, 0, 0,
        2, 2,
        0, 0, 0, 0, 0, 0,
        2,
        2, 2,
        0, 0, 0, 0, 0, 0,
        2, 2,
        0, 0, 0, 0, 0, 0
        ),
    joint_names = (
        'bridge1_joint_EA_X',
        'robot11_joint_EA_Y', 'robot11_joint_EA_Z',
        'robot11_joint_1', 'robot11_joint_2', 'robot11_joint_3', 'robot11_joint_4', 'robot11_joint_5', 'robot11_joint_6',
        'robot12_joint_EA_Y', 'robot12_joint_EA_Z',
        'robot12_joint_1', 'robot12_joint_2', 'robot12_joint_3', 'robot12_joint_4', 'robot12_joint_5', 'robot12_joint_6',
        'bridge2_joint_EA_X',
        'robot21_joint_EA_Y', 'robot21_joint_EA_Z',
        'robot21_joint_1', 'robot21_joint_2', 'robot21_joint_3', 'robot21_joint_4', 'robot21_joint_5', 'robot21_joint_6',
        'robot22_joint_EA_Y', 'robot22_joint_EA_Z',
        'robot22_joint_1', 'robot22_joint_2', 'robot22_joint_3', 'robot22_joint_4', 'robot22_joint_5', 'robot22_joint_6'
        )
    )

    current_configuration = start_configuration
    print(current_configuration)
    #########################################################################
    # Sequential path planning
    #########################################################################


    # Action 8 to 19 is realted to beam b2
    action = process.actions[9]
    for i, movement in enumerate(action.movements):
        print ("Movement (%s) - %s" % (i, movement))
        if isinstance(movement, RoboticFreeMovement):
            print ("\n> > > Plan RoboticFreeMovement")

            # Add already built beams to planning scene

            # Attach Tool and Beam to robot

            # Prepare Starting Config and Goal constraints
            # start_configuration = Configuration.from_revolute_values([0, 4.295, 0, -3.327, 4.755, 0.])
            group = robot.main_group_name
            print ("robot.main_group_name = %s" % robot.main_group_name)
            tolerance_position = 0.001
            tolerance_axes = [0.01] * 3
            goal_constraints = robot.constraints_from_frame(movement.target_frame, tolerance_position, tolerance_axes, group)
            trajectory = robot.plan_motion(goal_constraints, start_configuration = current_configuration, group = group, planner_id='RRT')

            print("Computed kinematic path with %d configurations." % len(trajectory.points))
            print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)

    pp.ros_client.close()
    #########################################################################
    # Save Results
    #########################################################################


if __name__ == "__main__":

    # test_process_prepathplan()
    test_process_pathPlan()

    # Result Goal 1: Visualize action, movement, trajectory by visualizing scene objects and robots.

    # Result Goal 2: Execute movements

    pass
