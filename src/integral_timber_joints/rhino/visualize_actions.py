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

try:
    from typing import Dict, List, Optional, Tuple, cast
except:
    pass

def redraw_state(process):
    artist = get_process_artist()
    artist.delete_state(redraw=False)
    artist.draw_state(redraw=True)  # Visualize the state
    print_current_state_info(process)

def ui_next_step(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements

    if artist.selected_state_id < len(all_movements):
        artist.selected_state_id += 1
        redraw_state(process)


def ui_prev_step(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements

    if artist.selected_state_id > 0:
        artist.selected_state_id -= 1
        redraw_state(process)


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
    redraw_state(process)


def ui_goto_state_by_state_index(process, selected_state_id = None):
    # type: (RobotClampAssemblyProcess, Optional[int]) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements

    # Ask use to chosse which beam (seq_id) to view
    if not selected_state_id:
        selected_state_id = rs.GetInteger("Which State to view, state_id = [0 to %i]" % (
            len(all_movements)), artist.selected_state_id, 0, len(all_movements))
        if selected_state_id is None:
            return
        if selected_state_id > len(all_movements):
            print("error: sequence_id must be between [0 to %i]" % (len(all_movements)))
            return

    # Select the start state of the first movement.
    artist.selected_state_id = selected_state_id
    redraw_state(process)


def print_current_state_info(process):
    # type: (RobotClampAssemblyProcess) -> None
    """Note when state_id = 1, it is referring to end of the first (0) movement."""
    artist = get_process_artist()
    all_movements = process.movements
    state_id = artist.selected_state_id
    # Retrive prev and next movement for information print out
    prev_movement = all_movements[state_id - 1] if state_id > 0 else None  # type: Movement
    next_movement = all_movements[state_id] if state_id < len(all_movements) else None  # type: Movement

    print("-----------------------------------------------")

    # * Printing Previous Movement Info
    if prev_movement is not None:
        print("- Prev Movement: %s (%s)" % (prev_movement.tag, prev_movement.movement_id))
        if isinstance(prev_movement, RoboticMovement):
            if prev_movement.allowed_collision_matrix != []:
                print("  - Prev Movement allowed_collision_matrix: %s" % prev_movement.allowed_collision_matrix)
            if prev_movement.operator_stop_after is not None:
                print("  - Operator Confirm: %s" % prev_movement.operator_stop_after)

    # * Printing Current State Info
    if state_id == 0:
        config_string = "(Robot Initial State)"
    elif process.movement_has_end_robot_config(prev_movement):
        config_string = " (Robot Config is Fixed!)"
    else:
        config_string = ""
    print("- Current Scene: #%i [0 to %i]. %s" % (state_id, len(all_movements), config_string))

    # * Printing Next Movement Info
    if next_movement is not None:
        print("- Next Movement: %s (%s)" % (next_movement.tag, next_movement.movement_id))
        if next_movement.operator_stop_before is not None:
            print("  - Operator Confirm: %s" % next_movement.operator_stop_before)

    print("-----------------------------------------------")


def ui_get_ik(process):
    # type: (RobotClampAssemblyProcess) -> None
    """Retriving an IK solution for the current state.
    The actual call would be to get the IK solution of the ending state of the previous movement.
    """
    # * Check against computing for initial state.
    artist = get_process_artist()
    state_id = artist.selected_state_id
    if state_id == 0:
        print("Sorry - Cannot get IK solution for the initial state.")
        return

    # * Check against computing for non Robotic Movements
    all_movements = process.movements
    prev_movement = all_movements[state_id - 1]
    if not isinstance(prev_movement, RoboticMovement):
        print("Sorry - Cannot get IK solution for State after non Robotic Movement.")
        return

    # * Get Ik call to Yijiang's rhino_interface
    print("-----------------------------------------------")
    print("Computing IK :")

    from compas.rpc import Proxy
    rhino_interface = Proxy('integral_timber_joints.planning.rhino_interface', autoreload=False)
    try:
        success, conf, msg = rhino_interface.get_ik_solutions(process, state_id - 1)
    except Exception as err_msg:
        success, conf, msg = (False, None, err_msg)

    if success:
        print("IK Success: {} | {}".format(msg, conf))
    else:
        print("IK Failed: {} ".format(msg))

    # Draw Robot with IK results
    if success:
        artist.draw_robot(conf, True, True, True)

    print("-----------------------------------------------")


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

    # Check if all computatations are up to date.
    for beam_id in assembly.sequence:
        if not process.dependency.beam_all_valid(beam_id):
            process.dependency.compute_all(beam_id)
            if not process.dependency.beam_all_valid(beam_id):
                print("WARNING: Beam (%s) contains invalid computations that cannot be resolved. Visualization maybe incorrect." % beam_id)

    # Initial draw of the state (using previous selected_state_id)
    rs.EnableRedraw(False)
    artist.delete_state(redraw=False)
    artist.draw_state(redraw=False)  # Visualize the state
    print_current_state_info(process)

    # Hide interactive beams and beams in positions
    for seq, beam_id in enumerate(assembly.sequence):
        artist.hide_interactive_beam(beam_id)
        artist.hide_beam_all_positions(beam_id)
        artist.hide_gripper_all_positions(beam_id)
        artist.hide_asstool_all_positions(beam_id)

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
        'message': "Choose Options or Type in State Number directly:",
        'options': [
            {'name': 'Finish', 'action': 'Exit'
             },
            {'name': 'NextStep', 'action': ui_next_step
             },
            {'name': 'PrevStep', 'action': ui_prev_step
             },
            {'name': 'GoToBeam', 'action': ui_goto_state_by_beam_seq
             },
            {'name': 'GoToState', 'action': ui_goto_state_by_state_index
             },
            {'name': 'ShowEnv', 'action': ui_show_env_meshes
             },
            {'name': 'HideEnv', 'action': ui_hide_env_meshes
             },
            {'name': 'GetIK', 'action': ui_get_ik
             },
        ]

    }

    command_to_run = None
    while (True):

        # Create Menu
        result = CommandMenu(config).select_action()

        def on_exit_ui():
            print('Exit Function')
            artist.hide_all_env_mesh()
            artist.hide_robot()
            show_interactive_beams_delete_state_vis()
            return Rhino.Commands.Result.Cancel

        # User cancel command by Escape
        if result is None:
            return on_exit_ui()

        # Shortcut to allow user to type in state directly
        if isinstance(result, str):
            if result.isnumeric():
                ui_goto_state_by_state_index(process, int(result))
                continue
            else:
                continue # CAtch other unknown input that are not numbers.

        # User click Exit Button
        if result['action'] == 'Exit':
            return on_exit_ui()

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
