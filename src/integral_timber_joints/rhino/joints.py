import json
import os

import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext as sc  # type: ignore
from compas.utilities import DataDecoder
from compas_rhino.geometry import RhinoMesh
from compas_rhino.ui import CommandMenu
from compas_rhino.utilities.objects import get_object_name, get_object_names

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.geometry import EnvironmentModel, JointNonPlanarLap, Joint_halflap
from integral_timber_joints.process import Movement, RobotClampAssemblyProcess, RoboticMovement
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions
from integral_timber_joints.tools import Clamp, Gripper, RobotWrist, ToolChanger

try:
    from typing import Dict, List, Optional, Tuple, cast
except:
    pass


def change_joint_parameters(process):
    # type: (RobotClampAssemblyProcess) -> None
    artist = get_process_artist()
    # Ask user to pick two beams
    guid1 = rs.GetObject('Select first beam of joint for parameter change:', custom_filter=get_existing_beams_filter(process))
    if guid1 is None:
        return
    beam_id1 = get_object_name(guid1)
    guid2 = rs.GetObject('Select second beam of joint for parameter change:', custom_filter=get_existing_beams_filter(process, exclude_beam_ids=[beam_id1]))
    if guid2 is None:
        return
    beam_id2 = get_object_name(guid2)
    joint_id = (beam_id1, beam_id2)

    if joint_id not in process.assembly.joint_ids():
        print("Joint (%s-%s) does not exist." % (beam_id1, beam_id2))
    joint = process.assembly.joint(joint_id)
    print("Joint (%s-%s) (%s)" % (beam_id1, beam_id2, joint.__class__.__name__))

    affected_beams = set()
    if isinstance(joint, Joint_halflap):
        current_height = joint.height
        new_height = rs.GetReal("New thickness of the lap joint: (current is %s)" % current_height, current_height)
        if new_height is None or new_height == current_height:
            return
        difference = new_height - current_height
        process.assembly.joint((beam_id1, beam_id2)).height += difference
        process.assembly.joint((beam_id2, beam_id1)).height -= difference
        print("Height of joint pair changed to %s and %s." % (
            process.assembly.joint((beam_id1, beam_id2)).height,
            process.assembly.joint((beam_id2, beam_id1)).height
        ))
        affected_beams.add(beam_id1)
        affected_beams.add(beam_id2)

    for beam_id in affected_beams:
        artist.redraw_interactive_beam(beam_id, force_update=True, redraw=False)
    rs.EnableRedraw(True)


def show_menu(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    # Ensure interactive beams are shown initially
    rs.EnableRedraw(False)
    artist.hide_robot()
    artist.hide_all_env_mesh()
    artist.hide_all_tools_in_storage()
    [artist.hide_beam_all_positions(beam_id) for beam_id in assembly.sequence]
    [artist.hide_asstool_all_positions(beam_id) for beam_id in assembly.sequence]
    [artist.hide_gripper_all_positions(beam_id) for beam_id in assembly.sequence]
    [artist.show_interactive_beam(beam_id) for beam_id in assembly.sequence]
    rs.EnableRedraw(True)
    sc.doc.Views.Redraw()

    def construct_menu():
        # Menu for user
        menu = {
            'message': "Choose Options",
            'options': [
                {'name': 'Finish', 'action': 'Exit'
                 },
                {'name': 'JointParameters', 'action': change_joint_parameters
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
            return Rhino.Commands.Result.Cancel

        # User cancel command by Escape
        if result is None:
            return on_exit_ui()

        # Shortcut to allow user to type in state directly
        if isinstance(result, str):
            if result.isnumeric():
                pass
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
