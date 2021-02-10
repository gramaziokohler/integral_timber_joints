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
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions
from integral_timber_joints.tools import Clamp, Gripper, RobotWrist, ToolChanger


def next_step(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements

    if artist.selected_state_id <= len(all_movements):
        artist.selected_state_id += 1
        artist.delete_state(redraw=False)
        artist.draw_state(redraw=True)  # Visualize the state
        print_current_state_info(process)


def prev_step(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements

    if artist.selected_state_id > 0:
        artist.selected_state_id -= 1
        artist.delete_state(redraw=False)
        artist.draw_state(redraw=True)  # Visualize the state
        print_current_state_info(process)


def goto_state_by_beam_seq(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements

    # Ask use to chosse which beam (seq_id) to view
    selected_seq_id = rs.GetInteger("Which Beam to view, by assembly sequence: sequence_id = [0 to %i]" % (len(assembly.sequence) - 1), 0, 0, len(assembly.sequence) - 1)
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
    prev_movement = all_movements[state_id - 1] if state_id > 0 else None
    next_movement = all_movements[state_id] if state_id <= len(all_movements) else None

    print("The prev movement is: %s" % (prev_movement))
    print("The current state is (%i of %s)" % (state_id, len(process.states)))
    print("The next movement is: %s" % (next_movement))


def show_menu(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    # First ask user which beam to view.
    goto_state_by_beam_seq(process)

    # Hide interactive beams and beams in positions
    rs.EnableRedraw(False)
    for seq, beam_id in enumerate(assembly.sequence):
        artist.hide_interactive_beam(beam_id)
        artist.hide_beam_all_positions(beam_id)
        artist.hide_gripper_all_positions(beam_id)
        artist.hide_clamp_all_positions(beam_id)
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
            {'name': 'NextStep', 'action': next_step
             },
            {'name': 'PrevStep', 'action': prev_step
             },
            {'name': 'GoToBeam', 'action': goto_state_by_beam_seq
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
            show_interactive_beams_delete_state_vis()
            return Rhino.Commands.Result.Cancel

        # User click Exit Button
        if result['action'] == 'Exit':
            print('Exit Function')
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
