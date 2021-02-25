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


def print_sequence(process):
    # type: (RobotClampAssemblyProcess) -> None
    message = ""
    for i, beam_id in enumerate(process.assembly.sequence):
        print("S%i, %s" % (i, beam_id))
        message += "%i, %s\n" % (i + 1, beam_id)
    rs.EditBox(message, "Sequence (starts from 1), Beam ID", "Sequence")


def show_menu(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    # Menu for user
    config = {
        'message': "Visualize Actions and Movements:",
        'options': [
            {'name': 'Finish', 'action': 'Exit'
             },
            {'name': 'PrintSequence', 'action': print_sequence
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
            return Rhino.Commands.Result.Cancel

        # User click Exit Button
        if result['action'] == 'Exit':
            print('Exit Function')
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
