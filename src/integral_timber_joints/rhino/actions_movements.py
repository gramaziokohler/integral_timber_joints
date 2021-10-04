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


def compute_states(process, verbose=False):
    # type: (RobotClampAssemblyProcess, bool) -> None
    """User triggered function to compute Actions from Assembly Sequence
    """
    # Optimization is not yet pully implemented
    # process.optimize_actions_place_pick_gripper()
    # process.optimize_actions_place_pick_clamp()

    # Ask user if recompute actions
    # recompute_initial_states = rs.GetString("Recompute Initial States?", "No", ["No", "Yes"])
    # if recompute_initial_states == "Yes":
    process.recompute_initial_state()

    # Make sure everything is computed and nothing is missing
    for beam_id in process.assembly.sequence:
        process.dependency.compute_all(beam_id, attempt_all_parents_even_failure=True, verbose=verbose)

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
    compute_states(process, True)


def remove_actions(process):
    # type: (RobotClampAssemblyProcess) -> None

    for beam_id in process.assembly.sequence:
        process.assembly.set_beam_attribute(beam_id, 'actions', [])
        process.dependency.invalidate(beam_id, process.create_actions_from_sequence)

    # Legacy file process level actions removal
    if 'actions' in process.attributes:
        process.attributes.pop('actions')

    print("Actions and states Removed")


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
