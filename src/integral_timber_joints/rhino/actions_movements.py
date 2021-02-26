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
from integral_timber_joints.process.dependency import ComputationalDependency


def reset_dependency_graph(process):
    # type: (RobotClampAssemblyProcess) -> None
    process.attributes['dependency'] = ComputationalDependency(process)
    print("Dependency graph is reset.")


def compute_action(process, verbose = False):
    # type: (RobotClampAssemblyProcess, bool) -> None
    """User triggered function to compute Actions from Assembly Sequence
    """
    # Make sure everything is computed and nothing is missing
    for beam_id in process.assembly.sequence:
        process.dependency.compute(beam_id, process.compute_all, attempt_all_parents_even_failure=True)

    # Call function to create actions
    print ("Compute 1 of 4 - Create Actions")
    process.create_actions_from_sequence(verbose=verbose)
    print ("Compute 2 of 4 - Assign Tools")
    process.assign_tools_to_actions(verbose=verbose)
    # process.optimize_actions_place_pick_gripper()
    # process.optimize_actions_place_pick_clamp()
    print ("Compute 3 of 4 - Create Movements")
    process.create_movements_from_actions(verbose=verbose)

    # Save result to log file.
    print ("Compute 4 of 4 - Export Movements Log")
    log_file_path = get_activedoc_path_no_ext() + "_process.log"
    process.debug_print_process_actions_movements(log_file_path)
    print ("Compute Action Completed")


def compute_states(process):
    # type: (RobotClampAssemblyProcess) -> None
    """User triggered function to compute Actions from Assembly Sequence
    """

    # Call function to create actions
    process.compute_initial_state()
    process.compute_intermediate_states(verbose=False)

    # Print out some information to user
    full_state_count = 0
    for state in process.intermediate_states:
        if len(state) > 0:
            full_state_count += 1
    print("Total: %i of %i intermediate states computed. for %i objects." % (full_state_count, len(process.intermediate_states), len(process.initial_state)))


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
                {'name': 'ComputeActions', 'action': compute_action
                 },
                {'name': 'ComputeStates', 'action': compute_states
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
