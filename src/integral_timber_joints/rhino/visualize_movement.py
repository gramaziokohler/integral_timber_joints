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
from integral_timber_joints.process import Movement, RobotClampAssemblyProcess, RoboticMovement, OperatorAttachToolMovement, RoboticFreeMovement
from integral_timber_joints.rhino.process_artist import ProcessArtist
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.utility import get_existing_beams_filter, purge_objects, recompute_dependent_solutions
from integral_timber_joints.tools import Clamp, Gripper, RobotWrist, ToolChanger
from integral_timber_joints.rhino.process_artist import AddAnnotationText

try:
    from typing import Dict, List, Optional, Tuple, cast
except:
    pass


############
# Drawing
############

def redraw_state(process):
    artist = get_process_artist()
    # artist.delete_state(redraw=False)
    artist.draw_state(redraw=False)  # Visualize the state
    draw_tool_id_tag(artist, redraw=False)  # Visualize tool id tag
    print_current_state_info(process)
    rs.EnableRedraw(True)
    sc.doc.Views.Redraw()


def print_current_state_info(process, print_prev=True, print_state=True, print_next=True):
    # type: (RobotClampAssemblyProcess, bool, bool, bool) -> None
    """Note when state_id = 1, it is referring to end of the first (0) movement."""
    artist = get_process_artist()
    all_movements = process.movements
    state_id = artist.selected_state_id
    # Retrive prev and next movement for information print out
    prev_movement = all_movements[state_id - 1] if state_id > 0 else None  # type: Movement
    next_movement = all_movements[state_id] if state_id < len(all_movements) else None  # type: Movement

    print("-----------------------------------------------")

    # * Printing Previous Movement Info
    if prev_movement is not None and print_prev:
        action = process.get_action_of_movement(prev_movement)
        print("- Prev Movement: %s (%s) (A:%s M:%s)" % (prev_movement.tag, prev_movement.movement_id, action.__class__.__name__, prev_movement.__class__.__name__))
        if isinstance(prev_movement, RoboticMovement):
            print("  - Prev Movement allowed_collision_matrix: %s" % prev_movement.allowed_collision_matrix)
        if prev_movement.operator_stop_after is not None:
            print("  - Operator Confirm: %s" % prev_movement.operator_stop_after)

    # * Printing Current State Info
    if print_state:
        if state_id == 0:
            config_string = "(Robot Initial State)"
        elif process.movement_has_end_robot_config(prev_movement):
            config_string = " (Robot Config is Fixed!)"
        else:
            config_string = ""
        print("- Current Scene: #%i [0 to %i]. %s" % (state_id, len(all_movements), config_string))

    # * Printing Next Movement Info
    if next_movement is not None and print_next:
        action = process.get_action_of_movement(next_movement)
        print("- Next Movement: %s (%s) (A:%s M:%s)" % (next_movement.tag, next_movement.movement_id, action.__class__.__name__, next_movement.__class__.__name__))
        if isinstance(next_movement, RoboticMovement):
            print("  - Next Movement allowed_collision_matrix: %s" % next_movement.allowed_collision_matrix)
        if next_movement.operator_stop_before is not None:
            print("  - Operator Confirm: %s" % next_movement.operator_stop_before)

    print("-----------------------------------------------")


def ui_show_env_meshes(process):
    # type: (RobotClampAssemblyProcess) -> None
    artist = get_process_artist()
    artist.show_all_env_mesh()


def ui_hide_env_meshes(process):
    # type: (RobotClampAssemblyProcess) -> None
    artist = get_process_artist()
    artist.hide_all_env_mesh()


############
# Navigation
############


def ui_next_step(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements

    if artist.selected_state_id < len(all_movements):
        artist.selected_state_id += 1


def ui_prev_step(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements

    if artist.selected_state_id > 0:
        artist.selected_state_id -= 1


def ui_next_robotic_movement(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements
    while artist.selected_state_id < len(all_movements):
        artist.selected_state_id += 1
        movement = all_movements[artist.selected_state_id - 1]
        if isinstance(movement, RoboticMovement):
            return


def ui_prev_robotic_movement(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements
    while artist.selected_state_id > 0:
        artist.selected_state_id -= 1
        movement = all_movements[artist.selected_state_id - 1]
        if isinstance(movement, RoboticMovement):
            return


def ui_next_robotic_free_movement(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements
    while artist.selected_state_id < len(all_movements):
        artist.selected_state_id += 1
        movement = all_movements[artist.selected_state_id - 1]
        if isinstance(movement, RoboticFreeMovement):
            return


def ui_prev_robotic_free_movement(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements
    while artist.selected_state_id > 0:
        artist.selected_state_id -= 1
        movement = all_movements[artist.selected_state_id - 1]
        if isinstance(movement, RoboticFreeMovement):
            return


def ui_next_operator_attach_tool(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements
    while artist.selected_state_id < len(all_movements):
        artist.selected_state_id += 1
        movement = all_movements[artist.selected_state_id - 1]
        if isinstance(movement, OperatorAttachToolMovement):
            return


def ui_goto_state_by_beam_seq(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements

    # Ask use to chosse which beam (seq_id) to view
    beam_input = rs.GetString("Which Beam to view, by assembly sequence: sequence_id = [0 to %i], or by beam_id" % (
        len(assembly.sequence) - 1), 0)


    if beam_input is None:
        return
    elif beam_input in assembly.sequence:
        beam_id = beam_input
    elif beam_input.isdigit():
        seq_n = int(beam_input)
        if seq_n > len(assembly.sequence) - 1:
            print("error: sequence_id must be between [0 to %i]" % (len(assembly.sequence) - 1))
            return
        beam_id = assembly.sequence[seq_n]

    # get the first movement before a selected beam
    selected_movement = process.get_movements_by_beam_id(beam_id)[0]
    if selected_movement is None:
        return

    # Select the start state of the first movement.
    mov_id = all_movements.index(selected_movement)
    artist.selected_state_id = mov_id


def ui_goto_state_by_state_index(process, selected_state_id=None):
    # type: (RobotClampAssemblyProcess, Optional[int]) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()
    all_movements = process.movements

    # Ask use to chosse which beam (seq_id) to view
    if selected_state_id is None:
        selected_state_id = rs.GetInteger("Which State to view, state_id = [0 to %i]" % (
            len(all_movements)), artist.selected_state_id, 0, len(all_movements))
        if selected_state_id is None:
            return

    # Check if entry is within bound
    if selected_state_id > len(all_movements):
        print("error: sequence_id must be between [0 to %i]" % (len(all_movements)))
        return

    # Select the start state of the first movement.
    artist.selected_state_id = selected_state_id

############
# Get IK
############


def ui_get_ik(process):
    # type: (RobotClampAssemblyProcess) -> None
    """Retriving an IK solution for the current state.
    The actual call would be to get the IK solution of the ending state of the previous movement.
    """
    # * Check against computing for initial state.
    artist = get_process_artist()
    state_id = artist.selected_state_id
    movement_id = state_id - 1

    if state_id == 0:
        print("Sorry - Cannot get IK solution for the initial state.")
        return

    # * Check against computing for non Robotic Movements
    all_movements = process.movements
    prev_movement = all_movements[movement_id]
    if not isinstance(prev_movement, RoboticMovement):
        print("Sorry - Cannot get IK solution for State after non Robotic Movement.")
        return

    # * Get Ik call to Yijiang's rhino_interface
    print("-----------------------------------------------")
    print("Computing IK :")

    from compas.rpc import Proxy
    rhino_interface = Proxy('integral_timber_joints.planning.rhino_interface', autoreload=False)
    try:
        success, configuration, msg = rhino_interface.get_ik_solutions(process, movement_id)
    except Exception as err_msg:
        success, configuration, msg = (False, None, err_msg)

    if success:
        print("IK Success: {} | {}".format(msg, configuration))
        # Delete rob_wrist visualization if present
        purge_objects(artist.state_visualization_guids('rob_wrist'), redraw=False)
        # Draw Robot with IK results
        artist.draw_robot(configuration, True, True, True)
        process.temp_ik[movement_id] = configuration
    else:
        print("IK Failed: {} ".format(msg))

    print("-----------------------------------------------")


def ui_save_ik(process):
    # type: (RobotClampAssemblyProcess) -> None
    """Retriving an IK solution for the current state.
    The actual call would be to get the IK solution of the ending state of the previous movement.
    """
    artist = get_process_artist()
    state_id = artist.selected_state_id
    movement_id = state_id - 1

    # * Save IK
    all_movements = process.movements
    prev_movement = all_movements[movement_id]  # type: RoboticMovement
    if movement_id in process.temp_ik and process.temp_ik[movement_id] is not None:
        prev_movement.target_configuration = process.temp_ik[movement_id]
        print("IK Saved at end of %s : %s" % (prev_movement.__class__.__name__, prev_movement.tag))
        del process.temp_ik[movement_id]


def draw_tool_id_tag(self, scene=None, redraw=True):
    # type: (ProcessArtist, any, bool) -> None
    """Draw tags of tool_id
    This is a hot fix function that is not yet imcoperated into Process artist
    """
    if scene is None:
        # Limit range
        if self.selected_state_id < 0:
            self.selected_state_id = 0
        if self.selected_state_id > len(self.process.movements):
            self.selected_state_id = len(self.process.movements)
        scene = self.get_current_selected_scene_state()

    # Layer:
    rs.EnableRedraw(False)
    layer = 'itj::interactive::beams_seqtag'
    rs.CurrentLayer(layer)

    delete_tool_id_tag(self)

    # * Draw meshes to Rhino and add guids to tracking dict
    for object_id in self.process.tool_ids:
        tool_base_frame = scene[(object_id, 'f')]
        layer = 'itj::interactive::beams_seqtag'
        guid = AddAnnotationText(tool_base_frame, object_id, 50, layer, redraw=False)
        self.tool_id_tags.append(guid)

    # Enable Redraw
    if redraw:
        rs.EnableRedraw(True)
        sc.doc.Views.Redraw()


def delete_tool_id_tag(self, redraw=True):

    if hasattr(self, 'tool_id_tags'):
        purge_objects(self.tool_id_tags, redraw=False)
    self.tool_id_tags = []

    # Enable Redraw
    if redraw:
        rs.EnableRedraw(True)
        sc.doc.Views.Redraw()


def show_menu(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    # Temporary storage of IK solution when using ui_get_ik and ui_save_ik
    if not hasattr(process, 'temp_ik'):
        process.temp_ik = {}

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
    draw_tool_id_tag(artist, redraw=False)  # Visualize the state
    print_current_state_info(process)

    # Hide interactive beams and beams in positions
    for beam_id in assembly.beam_ids():
        artist.hide_interactive_beam(beam_id)
        artist.hide_beam_all_positions(beam_id)
        artist.hide_gripper_all_positions(beam_id)
        artist.hide_asstool_all_positions(beam_id)

    # Show Environment Meshes
    artist.draw_all_env_mesh(True, redraw=False)
    rs.EnableRedraw(True)

    def show_interactive_beams_delete_state_vis():
        artist.delete_state(redraw=False)
        [artist.show_interactive_beam(beam_id) for beam_id in assembly.beam_ids()]
        rs.EnableRedraw(True)
        sc.doc.Views.Redraw()

    def construct_menu():
        # Menu for user
        artist = get_process_artist()
        state_id = artist.selected_state_id
        prev_movement_id = state_id - 1
        menu = {
            'message': "Choose Options or Type in State Number directly:",
            'options': [
                {'name': 'Finish', 'action': 'Exit'
                 },
                {'name': 'NextStep', 'action': ui_next_step
                 },
                {'name': 'PrevStep', 'action': ui_prev_step
                 },
                {'name': 'NextRoboticMovement', 'action': ui_next_robotic_movement
                 },
                {'name': 'PrevRoboticMovement', 'action': ui_prev_robotic_movement
                 },
                {'name': 'NextOperatorAttachTool', 'action': ui_next_operator_attach_tool
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
        # Add the repeat options when command_to_run is available
        if command_to_run is not None:
            menu['options'].insert(0, {'name': 'Repeat', 'action': 'Repeat'})

        if prev_movement_id in process.temp_ik and process.temp_ik[prev_movement_id] is not None:
            menu['options'].append({'name': 'SaveIK', 'action': ui_save_ik})

        return menu

    command_to_run = None
    while (True):

        # Create Menu
        result = CommandMenu(construct_menu()).select_action()

        def on_exit_ui():
            print('Exit Function')
            artist.hide_all_env_mesh()
            artist.hide_robot()
            show_interactive_beams_delete_state_vis()
            delete_tool_id_tag(artist)
            return Rhino.Commands.Result.Cancel

        # User cancel command by Escape
        if result is None:
            return on_exit_ui()

        # Shortcut to allow user to type in state directly
        if isinstance(result, str):
            if result.isnumeric():
                ui_goto_state_by_state_index(process, int(result))
                redraw_state(process)
                continue
            elif result.startswith("A"):
                movement_ids = process.movement_ids
                if result in movement_ids:
                    ui_goto_state_by_state_index(process, movement_ids.index(result) + 1)
                    redraw_state(process)
                    continue
            elif result.startswith("Cancel"):
                return on_exit_ui()
            else:
                continue  # Catch other unknown input that are not numbers.

        # User click Exit Button
        if result['action'] == 'Exit':
            return on_exit_ui()

        # Set command-to-run according to selection, else repeat previous command.
        if result['action'] != 'Repeat':
            command_to_run = result['action']

        # Run the selected command
        command_to_run(process)

        # The commands should not handle redrawing the main state.
        # Redraw State after running commands
        redraw_state(process)

        # ! Draw Robot Config for temp IK
        state_id = artist.selected_state_id
        movement_id = state_id - 1
        # if ('robot', 'c') in scene and scene[('robot', 'c')] is not None:
        if movement_id in process.temp_ik and process.temp_ik[movement_id] is not None:
            print("Temp IK Exist")
            artist = get_process_artist()
            artist.draw_robot(process.temp_ik[movement_id], True, False, True)

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
