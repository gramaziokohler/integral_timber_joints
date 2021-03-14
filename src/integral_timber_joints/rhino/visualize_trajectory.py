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
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions
from integral_timber_joints.tools import Clamp, Gripper, RobotWrist, ToolChanger


def ui_next_step(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements

    if artist.selected_state_id <= len(all_movements):
        artist.selected_state_id += 1
        artist.delete_state(redraw=False)
        artist.draw_state(redraw=True)  # Visualize the state
        print_current_state_info(process)


def ui_prev_step(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements

    if artist.selected_state_id > 0:
        artist.selected_state_id -= 1
        artist.delete_state(redraw=False)
        artist.draw_state(redraw=True)  # Visualize the state
        print_current_state_info(process)


def ui_goto_state_by_beam_seq(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements

    # Ask use to chosse which beam (seq_id) to view
    selected_seq_id = rs.GetInteger("Which Beam to view, by assembly sequence: sequence_id = [0 to %i]" % (
        len(assembly.sequence) - 1), 0, 0, len(assembly.sequence) - 1)
    if selected_seq_id is None:
        return
    if selected_seq_id > len(assembly.sequence) - 1:
        print("error: sequence_id must be between [0 to %i]" % (len(assembly.sequence) - 1))

    # get the first movement before a selected beam
    selected_movement = process.get_movements_by_beam_id(assembly.sequence[selected_seq_id])[0]
    if selected_movement is None:
        return

    # Select the start state of the first movement.
    mov_id = all_movements.index(selected_movement)
    artist.selected_state_id = mov_id
    artist.delete_state(redraw=False)
    artist.draw_state(redraw=True)  # Visualize the state
    print_current_state_info(process)


def print_current_state_info(process):
    # type: (RobotClampAssemblyProcess) -> None
    artist = get_process_artist()
    all_movements = process.movements
    state_id = artist.selected_state_id
    # Retrive prev and next movement for information print out
    prev_movement = all_movements[state_id - 1] if state_id > 0 else None  # type: Movement
    next_movement = all_movements[state_id] if state_id <= len(all_movements) else None  # type: Movement

    config_string = " (! Robot config is SET !)" if process.states[state_id]['robot'].kinematic_config is not None else ""
    if prev_movement is not None:
        print("The prev movement is: %s" % (prev_movement.tag))
    print("The current state is (%i of %s).%s" % (state_id, len(process.states), config_string))
    if next_movement is not None:
        print("The next movement is: %s" % (next_movement.tag))
        if next_movement.operator_stop_before is not None:
            print("! Confirm Before Next movement: %s" % next_movement.operator_stop_before)
        if next_movement.operator_stop_after is not None:
            print("! Confirm After Next movement: %s" % next_movement.operator_stop_after)
    if isinstance(next_movement, RoboticMovement):
        if next_movement.allowed_collision_matrix != []:
            print("Next Movement allowed_collision_matrix: %s" % next_movement.allowed_collision_matrix)


def ui_show_env_meshes(process):
    # type: (RobotClampAssemblyProcess) -> None
    artist = get_process_artist()
    artist.show_all_env_mesh()


def ui_hide_env_meshes(process):
    # type: (RobotClampAssemblyProcess) -> None
    artist = get_process_artist()
    artist.hide_all_env_mesh()


def show_menu(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    # First ask user which beam to view.
    ui_goto_state_by_beam_seq(process)

    # Hide interactive beams and beams in positions
    rs.EnableRedraw(False)
    for seq, beam_id in enumerate(assembly.sequence):
        artist.hide_interactive_beam(beam_id)
        artist.hide_beam_all_positions(beam_id)
        artist.hide_gripper_all_positions(beam_id)
        artist.hide_clamp_all_positions(beam_id)

    # Show Environment Meshes
    artist.draw_all_env_mesh(True, redraw=False)
    rs.EnableRedraw(True)

    def show_interactive_beams_delete_state_vis():
        artist.delete_state(redraw=False)
        [artist.show_interactive_beam(beam_id) for beam_id in assembly.sequence]
        rs.EnableRedraw(True)
        sc.doc.Views.Redraw()

    # Menu for user
    config = {
        'message': "Visualize Actions and Movements:",
        'options': [
            {'name': 'Finish', 'action': 'Exit'
             },
            {'name': 'NextStep', 'action': ui_next_step
             },
            {'name': 'PrevStep', 'action': ui_prev_step
             },
            {'name': 'GoToBeam', 'action': ui_goto_state_by_beam_seq
             },
            {'name': 'ShowEnv', 'action': ui_show_env_meshes
             },
            {'name': 'HideEnv', 'action': ui_hide_env_meshes
             },
        ]

    }

    command_to_run = None
    while (True):

        # Create Menu
        result = CommandMenu(config).select_action()

        # User cancel command by Escape
        if result is None or 'action' not in result:
            print('Exit Function')
            artist.hide_all_env_mesh()
            show_interactive_beams_delete_state_vis()
            return Rhino.Commands.Result.Cancel

        # User click Exit Button
        if result['action'] == 'Exit':
            print('Exit Function')
            artist.hide_all_env_mesh()
            show_interactive_beams_delete_state_vis()
            return Rhino.Commands.Result.Cancel

        # Add the repeat options after the first loop
        if command_to_run is None:
            config['options'].insert(0, {'name': 'Repeat', 'action': 'Repeat'})

        # Set command-to-run according to selection, else repeat previous command.
        if result['action'] != 'Repeat':
            command_to_run = result['action']

        # Run the selected command
        command_to_run(process)


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
