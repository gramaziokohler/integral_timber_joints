import json
import os

import Rhino  # type: ignore
import rhinoscriptsyntax as rs
from compas.utilities import DataDecoder
from compas_rhino.geometry import RhinoMesh, RhinoPoint
from compas_rhino.ui import CommandMenu
from compas_rhino.utilities.objects import get_object_name

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.process.dependency import ComputationalDependency, ComputationalResult
from integral_timber_joints.rhino.load import get_activedoc_path_no_ext, get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions
from integral_timber_joints.tools import Clamp, Gripper, PickupStation, StackedPickupStation


def reset_dependency_graph(process):
    # type: (RobotClampAssemblyProcess) -> None
    process.attributes['dependency'] = ComputationalDependency(process)
    print("Dependency graph is reset.")


def compute_states(process, verbose=False, beam_ids=None):
    # type: (RobotClampAssemblyProcess, bool, str) -> None
    """User triggered function to compute Actions from Assembly Sequence
    """
    process.recompute_initial_state()

    # Make sure everything is computed and nothing is missing
    if beam_ids is None:
        beam_ids = process.assembly.sequence
    for beam_id in beam_ids:
        process.dependency.invalidate_all(beam_id)
        process.dependency.compute_all(beam_id, attempt_all_parents_even_failure=False, verbose=verbose)

    invalid_beams = process.dependency.get_invalid_beam_ids()
    if len(invalid_beams) > 0:
        print("Warning: The following beams are not yet computationally valid: %s" % invalid_beams)
        print("States are not valid.")
        return

    # Assign unique numbers across the entire process file.
    process.assign_unique_action_numbers()

    # Save action description to log file.
    log_file_path = get_activedoc_path_no_ext() + "_process.log"
    process.debug_print_process_actions_movements(log_file_path)
    print("Action Log saved to: %s" % log_file_path)

    # Print out some information to user
    diff_count = 0
    for movement in process.movements:
        diff_count += len(movement.state_diff)
    print("Total: %i diffs computed for %i object states." % (diff_count, len(process.initial_state)))
    print("Total: %i Movements in %i Acttions for %i Beams." % (len(process.movements), len(process.actions), len(process.assembly.sequence)))


def compute_states_verbose(process):
    # type: (RobotClampAssemblyProcess) -> None
    """Similar to compute states.
    Users can pick chich beam_id or seq_n to recompute.
    """
    sequence = process.assembly.sequence
    str = rs.GetString('Which seq_n (0 to %i) or beam_id to recompute [default = all]' % (len(sequence)), 'all')
    if str == 'all':
        return compute_states(process, verbose=True)

    if str in sequence:
        beam_id = str
        return compute_states(process, verbose=True, beam_ids=[beam_id])

    if all([char.isdigit() for char in str]):
        seq_n = int(str)
        if seq_n >= 0 and seq_n < len(sequence):
            beam_id = sequence[seq_n]
            return compute_states(process, verbose=True, beam_ids=[beam_id])

    print("Input %s is not valid" % str)


def remove_actions(process):
    # type: (RobotClampAssemblyProcess) -> None

    for beam_id in process.assembly.sequence:
        process.assembly.set_beam_attribute(beam_id, 'actions', [])
        process.dependency.invalidate(beam_id, process.create_actions_from_sequence)

    # Legacy file process level actions removal
    if 'actions' in process.attributes:
        process.attributes.pop('actions')

    print("Actions and states Removed")

def load_tamp_results(process):
    # type: (RobotClampAssemblyProcess) -> None
    artist = get_process_artist()
    from integral_timber_joints.process.action import PickGripperFromStorageAction, PlaceGripperToStorageAction
    from integral_timber_joints.process.action import PickBeamWithGripperAction, BeamPlacementWithoutClampsAction, LoadBeamAction

    # Ask user for a json file

    path = rs.OpenFileName("Open Result File (*.json)|*.json|All Files (*.*)|*.*||")
    if path:
        with open(path, 'r') as f:
            # Deserialize asert correctness and add to Process
            sequences = json.load(f, cls=DataDecoder) #type: list
            print(sequences)
    
    # remove old actions from beams
    for beam_id in process.assembly.beam_ids():
        print ("Removing actions from beam %s" % beam_id)
        process.assembly.set_beam_attribute(beam_id, 'actions', [])
    
    # Change assembly sequence
    assembly_sequence = []
    for sequence in sequences:
        beam_id = sequence['beam_id']
        assembly_sequence.append(beam_id)
    process.assembly.sequence = assembly_sequence

    # Add actions to beams attributes
    from compas_fab.robots import JointTrajectory
    from integral_timber_joints.process import RoboticMovement
    for sequence in sequences:
        seq_n = sequence['seq_n']
        beam_id = sequence['beam_id']
        pddl_actions = sequence['actions']
        actions = []
        trajectory = None
        trajectory_action_type = None
        for pddl_action in pddl_actions:
            print("seq_n: %i, act_n: %i, action_name: %s" % (seq_n, pddl_action['act_n'], pddl_action['action_name']))
            act_n = pddl_action['act_n']
            action_name = pddl_action['action_name']
            args = pddl_action['args']
            if action_name == 'pick_gripper_from_storage':
                actions.append(PickGripperFromStorageAction(seq_n, act_n, args[1], args[0]))
            elif action_name == 'place_gripper_to_storage':
                actions.append(PlaceGripperToStorageAction(seq_n, act_n, args[1], args[0]))
            elif action_name == 'assemble_beam_with_gripper':
                actions.append(LoadBeamAction(seq_n, act_n, beam_id))
                actions.append(PickBeamWithGripperAction(seq_n, act_n, args[0], args[1]))
                actions.append(BeamPlacementWithoutClampsAction(seq_n, act_n, args[0], args[1]))
                if type(args[3]) is JointTrajectory:
                    trajectory = args[3]
                    trajectory_action_type = BeamPlacementWithoutClampsAction

        # Reorder action number
        for i, action in enumerate(actions):
            action.act_n = i

        process.assembly.set_beam_attribute(beam_id, 'actions', actions)
        process.create_movements_from_action(beam_id, verbose=True)

        def find_action_needing_trajectory():
            for action in process.get_actions_by_beam_id(beam_id):
                if type(action) is trajectory_action_type:
                    return action
            return None
        
        # Set trajectory if exist
        if trajectory is not None:
            print ("trajectory: %s (%i points)" % (trajectory, len(trajectory.points)))
            action = find_action_needing_trajectory()
            print ("action_with_traj: %s (%i movements)" % (action, len(action.movements)))
            j = 0
            for movement in action.movements:
                # skip non-robotic movements
                if not isinstance(movement, RoboticMovement):
                    continue
                if movement.target_configuration is not None:
                    continue
                print ("movement to be filled with trajectory: %s" % movement)
                configuration = trajectory.points[j]
                print ("configuration %s" % configuration) 
                # movement.trajectory = trajectory                       
                movement.state_diff[('robot', 'c')] = configuration
                j = j+1

def not_implemented(process):
    #
    print('This function is not implemented')


def show_menu(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    while (True):
        # Create Menu
        # Check if it is ready to compute Actions and Movements
        message = "Compute Actions and Movements:"

        config = {
            'message': message,
            'options': [
                {'name': 'Finish', 'action': 'Exit'
                 },
                {'name': 'ResetDependencyGraph', 'action': reset_dependency_graph
                 },
                {'name': 'ComputeStates', 'action': compute_states
                 },
                {'name': 'ComputeStatesVerbose', 'action': compute_states_verbose
                 },
                {'name': 'RemoveActionsAndStates', 'action': remove_actions
                 },
                {'name': 'LoadTAMPResults', 'action': load_tamp_results
                 },
            ]

        }

        result = CommandMenu(config).select_action()
        # User cancel command by Escape
        if result is None or 'action' not in result:
            print('Exit Function')
            return Rhino.Commands.Result.Cancel

        action = result['action']

        # User click Exit Button
        if action == 'Exit':
            print('Exit Function')
            return Rhino.Commands.Result.Cancel
        if action == 'Back':
            # Back is simply redisplaying the menu again from root.
            continue
        else:
            # Run the selected command
            action(process)


######################
# Rhino Entry Point
######################
# Below is the functions that get evoked when user press UI Button
# Put this in the Rhino button ! _-RunPythonScript 'integral_timber_joints.rhino.tools_environment.py'
if __name__ == '__main__':
    process = get_process()
    if process_is_none(process):
        print("Load json first")
    else:
        show_menu(process)
