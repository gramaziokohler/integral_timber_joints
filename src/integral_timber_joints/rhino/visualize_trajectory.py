import json
import os

import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext as sc  # type: ignore
from compas.utilities import DataDecoder
from compas_rhino.geometry import RhinoMesh
from compas_rhino.ui import CommandMenu
from compas_rhino.utilities.objects import get_object_name

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.geometry.env_model import EnvironmentModel
from integral_timber_joints.process import Movement, RobotClampAssemblyProcess, RoboticMovement
from integral_timber_joints.rhino.load import get_activedoc_process_path, get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.process_artist import ProcessArtist
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions
from integral_timber_joints.rhino.visualize_actions import (print_current_state_info, ui_goto_state_by_beam_seq, ui_goto_state_by_state_index, ui_hide_env_meshes, ui_next_step,
                                                            ui_prev_step, ui_show_env_meshes, ui_next_robotic_movement, ui_prev_robotic_movement)
from integral_timber_joints.tools import Clamp, Gripper, RobotWrist, ToolChanger


# ########################################
# Function to drawing scene and trajectory
# ########################################

def load_selected_external_movment_if_exist(process):
    # type: (RobotClampAssemblyProcess) -> None
    artist = get_process_artist()
    state_id = artist.selected_state_id

    # * Skip if the state is at initial_state
    if state_id == 0:
        return
    # * Skip if the movement is not RoboticMovement
    prev_movement = process.movements[state_id - 1]
    if not isinstance(prev_movement, RoboticMovement):
        return

    external_movement_path = os.path.join(get_activedoc_process_path(), '..\\results')
    result = process.load_external_movement(external_movement_path, prev_movement, subdir='smoothed_movements', verbose=False)

    if result is not None:
        print("Smoothed Trajectory Loaded")
    else:
        external_movement_path = os.path.join(get_activedoc_process_path(), '..\\results')
        process.load_external_movement(external_movement_path, prev_movement, subdir='movements', verbose=False)
        print("Rough Trajectory Loaded")


def redraw_state_and_trajectory(artist, process):
    # type: (ProcessArtist, RobotClampAssemblyProcess) -> None
    artist.delete_state(redraw=False)
    artist.delete_sweep_trajectory(redraw=False)
    artist.draw_state(redraw=False)  # Visualize the state
    artist.draw_sweep_trajectory(redraw=False)
    print_current_state_info(process, print_next=False)

# ##########################
# Function on entry and exit
# ##########################


def hide_interactive_beams(process):
    artist = get_process_artist()
    for seq, beam_id in enumerate(process.assembly.sequence):
        artist.hide_interactive_beam(beam_id)
        artist.hide_beam_all_positions(beam_id)
        artist.hide_gripper_all_positions(beam_id)
        artist.hide_asstool_all_positions(beam_id)


def show_interactive_beams_delete_state_vis(process):
    artist = get_process_artist()
    artist.delete_state(redraw=False)
    [artist.show_interactive_beam(beam_id) for beam_id in process.assembly.sequence]
    rs.EnableRedraw(True)
    sc.doc.Views.Redraw()




##############
# Show UI Menu
##############

def show_menu(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    # Check if all computatations are up to date.
    for beam_id in assembly.sequence:
        if not process.dependency.beam_all_valid(beam_id):
            process.dependency.compute_all(beam_id)
            if not process.dependency.beam_all_valid(beam_id):
                print("WARNING: Beam (%s) contains invalid computations that cannot be resolved. Visualization maybe incorrect." % beam_id)

    # Hide interactive beams and beams in positions
    rs.EnableRedraw(False)

    hide_interactive_beams(process)

    # Draw trajectory
    load_selected_external_movment_if_exist(process)
    redraw_state_and_trajectory(artist, process)

    # Show Environment Meshes
    artist.draw_all_env_mesh(True, redraw=False)
    rs.EnableRedraw(True)

    def construct_menu():
        # Menu for user
        artist = get_process_artist()
        state_id = artist.selected_state_id
        prev_movement_id = state_id - 1
        menu = {
            'message': "Choose Options or Type in State Number directly: (ESC to exit)",
            'options': [
                {'name': 'NextRoboticMovement', 'action': ui_next_robotic_movement
                 },
                {'name': 'PrevRoboticMovement', 'action': ui_prev_robotic_movement
                 },
                {'name': 'GoToBeam', 'action': ui_goto_state_by_beam_seq
                 },
                {'name': 'GoToState', 'action': ui_goto_state_by_state_index
                 },
                {'name': 'ShowEnv', 'action': ui_show_env_meshes
                 },
                {'name': 'HideEnv', 'action': ui_hide_env_meshes
                 },

            ]
        }
        # Add the repeat options when command_to_run is available
        if command_to_run is not None:
            menu['options'].insert(0, {'name': 'Repeat', 'action': 'Repeat'})

        return menu

    command_to_run = None
    while (True):

        # Create Menu
        result = CommandMenu(construct_menu()).select_action()

        def on_exit_ui():
            print('Exit Function')
            artist.hide_all_env_mesh()
            artist.hide_robot(redraw=False)
            artist.delete_sweep_trajectory(redraw=False)
            show_interactive_beams_delete_state_vis(process)
            return Rhino.Commands.Result.Cancel

        # User cancel command by Escape
        if result is None:
            return on_exit_ui()

        # Shortcut to allow user to type in state directly
        if isinstance(result, str):
            if result.isnumeric():
                ui_goto_state_by_state_index(process, int(result))
            elif result.startswith("Cancel"):
                return on_exit_ui()
            else:
                continue  # Catch other unknown input that are not numbers.

        # Run command
        if 'action' in result:
            # Set new command-to-run according to selection
            if result['action'] != 'Repeat':
                command_to_run = result['action']
            command_to_run(process)
        # Run the selected command
        # The commands should not handle redrawing the main state.

        # Draw the trajectory
        load_selected_external_movment_if_exist(process)
        redraw_state_and_trajectory(artist, process)
        rs.EnableRedraw(True)

######################
# Rhino Entry Point
######################
# Below is the functions that get evoked when user press UI Button
# Put this in the Rhino button ! _-RunPythonScript 'integral_timber_joints.rhino.sequence.py'


if __name__ == '__main__':
    process = get_process()
    if process_is_none(process):
        print("Load json first")
    else:
        show_menu(process)
