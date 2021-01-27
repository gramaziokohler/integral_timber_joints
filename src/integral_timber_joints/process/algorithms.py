try:
    from typing import List, Dict, Tuple, Optional
    from integral_timber_joints.process import RobotClampAssemblyProcess
    from integral_timber_joints.process import RFLPathPlanner
except:
    pass

from integral_timber_joints.process.action import *
from integral_timber_joints.process.movement import *
from integral_timber_joints.assembly import Assembly
from integral_timber_joints.geometry import Beam, Joint
from integral_timber_joints.tools import Tool, Clamp, Gripper
from integral_timber_joints.process.state import ObjectState
import itertools

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

    for seq_n, beam_id in enumerate(assembly.sequence):
        # Fuction to keep count of act_n

        def act_n(reset = False):
            if reset: act_n.counter = 0
            else: act_n.counter += 1
            return act_n.counter

        beam = assembly.beam(beam_id) # type: Beam
        if verbose: print ("Beam %s" % beam_id)
        actions.append(LoadBeamAction(seq_n, act_n(True), beam_id))
        if verbose: print ('|- ' + actions[-1].__str__())

        # move clamps from storage to structure
        # joint_id_of_clamps = assembly.get_joints_of_beam_connected_to_already_built(beam_id)

        joint_id_of_clamps = [(neighbour_id, beam_id) for neighbour_id in assembly.get_already_built_neighbors(beam_id) if assembly.get_joint_attribute((neighbour_id, beam_id), "is_clamp_attached_side")]

        for joint_id in joint_id_of_clamps:
            clamp_type = assembly.get_joint_attribute(joint_id, "clamp_type")
            actions.append(PickClampFromStorageAction(seq_n, act_n(), clamp_type))
            if verbose: print ('|- ' + actions[-1].__str__())
            actions.append(PlaceClampToStructureAction(seq_n, act_n(), joint_id, clamp_type))
            if verbose: print ('|- ' + actions[-1].__str__())

            #if verbose: print ("|- Detatch Clamp at Joint %s-%s" % clamp)

        # attach gripper
        gripper_type = assembly.get_beam_attribute(beam_id, "gripper_type")
        actions.append(PickGripperFromStorageAction(seq_n, act_n(), gripper_type))
        if verbose: print ('|- ' + actions[-1].__str__())

        # pick place beam
        actions.append(BeamPickupAction(seq_n, act_n(), beam_id))
        if verbose: print ('|- ' + actions[-1].__str__())

        #Syncronized clamp and move beam action
        if len(joint_id_of_clamps) > 0:
            actions.append(BeamPlacementWithClampsAction(seq_n, act_n(), beam_id, joint_id_of_clamps))
            if verbose: print ('|- ' + actions[-1].__str__())
        else:
            actions.append(BeamPlacementWithoutClampsAction(seq_n, act_n(), beam_id))
            if verbose: print ('|- ' + actions[-1].__str__())

        # return gripper
        actions.append(PlaceGripperToStorageAction(seq_n, act_n(), gripper_type))
        if verbose: print ('|- ' + actions[-1].__str__())

        # remove clamps from structure to storage
        for joint_id in joint_id_of_clamps:
            clamp_type = assembly.get_joint_attribute(joint_id, "clamp_type")
            actions.append(PickClampFromStructureAction(seq_n, act_n(), joint_id, clamp_type))
            if verbose: print ('|- ' + actions[-1].__str__())
            actions.append(PlaceClampToStorageAction(seq_n, act_n(), clamp_type))
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
        if isinstance(action, PickToolFromStorageAction):
            if tool_at_robot is not None: # Consistancy check to make sure tool is not already attached.
                raise Exception("Unable to attach tool becuase %s is already attached to robot" % tool_at_robot)
            tool = PopToolFromStorage(action.tool_type)
            tool_at_robot = tool
            action.tool_id = tool.name

        if isinstance(action, PlaceToolToStorageAction):
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

        if isinstance(action, BeamPickupAction):
            if tool_at_robot is None:
                raise Exception("Unable to pick beam becuase No Tool is attached to robot")
            if beam_id_on_pickup_station != action.beam_id:
                raise Exception("Unable to pick beam becuase beam_id_on_pickup_station (%s) is inconsistant with action.beam_id (%s)" % (beam_id_on_pickup_station, action.beam_id))
            beam_id_on_pickup_station = None
            beam_id_at_robot = action.beam_id
            gripper_id = tool_at_robot.name
            action.gripper_id = gripper_id
            process.assembly.set_beam_attribute(action.beam_id, 'gripper_id', gripper_id)

        if isinstance(action, BeamPlacementWithoutClampsAction):
            if tool_at_robot is None:
                raise Exception("Unable to place beam becuase No Tool is attached to robot")
            if beam_id_at_robot is None:
                raise Exception("Unable to place beam becuase No Beam is attached to robot")
            if beam_id_at_robot != action.beam_id:
                raise Exception("Unable to place beam becuase beam_id_at_robot (%s) is inconsistant with action.beam_id (%s)" % (beam_id_at_robot, action.beam_id))
            beam_id_at_robot = None
            action.gripper_id = tool_at_robot.name

        if isinstance(action, BeamPlacementWithClampsAction):
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

    # The functions to create_movements are located within each of the Action class.
    for action in actions:
            action.create_movements(process)

def debug_print_process_actions_movements(process, file_path = None):
    f = None
    if file_path is not None:
        f = open(file_path, 'w')
    for i, action in enumerate(process.actions):
        # Print the separator between each beam
        if isinstance(action, LoadBeamAction):
            line = "\nBeam %s assembly\n" % action.beam_id
            if file_path is not None: f.write(line)
            else: print (line)

        # Print Action Name
        line = "|- Action %s (%s): %s\n" % (i, action.__class__.__name__, action)
        if file_path is not None: f.write(line)
        else: print (line)

        # Print Movement Name
        for j, movement in enumerate(action.movements):
            line = "|  |- Movement %s (%s): %s\n" % (j, movement.__class__.__name__, movement)
            if file_path is not None: f.write(line)
            else: print (line)
    if file_path is not None:
        f.close()

def compute_initial_state(process, verbose = True):
    # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
    """Compute the initial scene state. This is the begining of the assembly process
    
    State Change
    ------------
    This functions sets the following process attribute:
    - 'initial_state' dict(str, ObjectState)
    
    """
    process.initial_state = {}
    assembly = process.assembly

    # Beams are all in their storage position
    for beam_id in assembly.sequence:
        state = ObjectState()
        state.current_frame = assembly.get_beam_attribute(beam_id, 'assembly_wcf_storage')
        process.initial_state[beam_id] = state
    
    # Tools (Clamps, Grippers) are all in their storage position
    for tool in itertools.chain(process.clamps, process.grippers):
        state = ObjectState()
        state.current_frame = tool.tool_storage_frame
        process.initial_state[tool.name] = state

    # Robot is in its initial position

    # Environment models are in its own position. They dont move anyways. The State is simply empty.
    for env_id, env_model in process.environment_models.items():
        state = ObjectState()
        process.initial_state[env_id] = state
     

def compute_intermediate_states(process, verbose = True):
    # type: (RobotClampAssemblyProcess, Optional[bool]) -> None
    """Compute the intermediate scenes according to the Movement(s) in `process.action`

    Movements and Actions should be computed before.

    State Change
    ------------
    This functions sets the following process attribute:
    - 'intermediate_states'  list(dict(str, ObjectState))

    """
    pass

def test_process_prepathplan(json_path_in, json_path_out):
    #########################################################################
    # Load process
    #########################################################################

    import jsonpickle
    f = open(json_path_in, 'r')
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

    print("\n> > > Saving Process to %s \n" % json_path_out)
    f = open(json_path_out, 'w')
    f.write(jsonpickle.encode(process, keys=True))
    f.close()

def test_process_pathPlan(json_path_in, json_path_out):
    #########################################################################
    # Load process
    #########################################################################

    import jsonpickle
    f = open(json_path_in, 'r')
    json_str = f.read()
    process = jsonpickle.decode(json_str, keys=True) # type: RobotClampAssemblyProcess
    f.close()

    #########################################################################
    # Connect to path planning backend and initialize robot parameters
    #########################################################################

    pp = RFLPathPlanner(None)
    pp.connect_to_ros_planner("192.168.183.129")
    robot = pp.robot # type: compas_fab.robots.Robot

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
        print ("Movement (%s) - %s" % (i, movement))
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
