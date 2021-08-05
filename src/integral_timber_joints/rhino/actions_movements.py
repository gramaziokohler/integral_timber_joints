import json
import os

import Rhino  # type: ignore
import rhinoscriptsyntax as rs
from compas.utilities import DataDecoder
from compas_rhino.ui import CommandMenu
from compas_rhino.geometry import RhinoMesh, RhinoPoint

from compas_rhino.utilities.objects import get_object_name

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none, get_activedoc_path_no_ext
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions
from integral_timber_joints.tools import Clamp, Gripper, PickupStation, StackedPickupStation
from integral_timber_joints.process.dependency import ComputationalDependency, ComputationalResult


def reset_dependency_graph(process):
    # type: (RobotClampAssemblyProcess) -> None
    process.attributes['dependency'] = ComputationalDependency(process)
    print("Dependency graph is reset.")


def compute_states(process):
    # type: (RobotClampAssemblyProcess) -> None
    """User triggered function to compute Actions from Assembly Sequence
    """
    # Optimization is not yet pully implemented
    # process.optimize_actions_place_pick_gripper()
    # process.optimize_actions_place_pick_clamp()

    # Make sure everything is computed and nothing is missing
    invalid_beams = []
    for beam_id in process.assembly.sequence:
        process.dependency.compute(beam_id, process.compute_all, attempt_all_parents_even_failure=True)
        if not process.dependency.get_solution_validity(beam_id, process.compute_all) in ComputationalResult.ValidResults:
            invalid_beams.append(beam_id)
    if len(invalid_beams) > 0:
        print("Warning: The following beams are not yet computationally valid:" % invalid_beams)
        print("Export cannnot continue.")
        return

    # Save action description to log file.
    log_file_path = get_activedoc_path_no_ext() + "_process.log"
    process.debug_print_process_actions_movements(log_file_path)
    print ("Action Log saved to: %s" % log_file_path)

    # Call function to create actions
    process.assign_unique_action_numbers()
    process.compute_initial_state()
    process.compute_intermediate_states(verbose=False)

    # Print out some information to user
    full_state_count = 0
    for state in process.intermediate_states:
        if len(state) > 0:
            full_state_count += 1
    print("Total: %i of %i intermediate states computed. for %i objects." % (full_state_count, len(process.intermediate_states), len(process.initial_state)))


def remove_actions(process):
    # type: (RobotClampAssemblyProcess) -> None

    for beam_id in process.assembly.sequence:
        process.assembly.set_beam_attribute(beam_id, 'actions', [])
        process.dependency.invalidate(beam_id, process.create_actions_from_sequence)

    # Legacy file process level actions removal
    if 'actions' in process.attributes: process.attributes.pop('actions')

    print ("Actions and states Removed")

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
