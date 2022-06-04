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
from integral_timber_joints.report.beam import beam_report
from integral_timber_joints.report.tool import tool_report
from integral_timber_joints.report.screw import screw_report


def print_sequence(process):
    # type: (RobotClampAssemblyProcess) -> None
    from integral_timber_joints.assembly import BeamAssemblyMethod
    messages = ""
    assembly = process.assembly
    for i, beam_id in enumerate(assembly.sequence):

        assembly_method = assembly.get_assembly_method(beam_id)
        if assembly_method == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            messages += "%i;%s;Manual\n" % (i + 1, beam_id)
            continue

        gripper = process.get_gripper_of_beam(beam_id)
        gripper_string = "%s(%s)"%(gripper.name, gripper.type_name)

        if assembly_method not in [BeamAssemblyMethod.CLAMPED] + BeamAssemblyMethod.screw_methods :
            messages += "%i;%s;%s\n" % (i + 1, beam_id, gripper_string)
            continue

        tools_strings = [gripper_string]
        for joint_id in assembly.get_joint_ids_with_tools_for_beam(beam_id):
            tool_id = assembly.get_joint_attribute(joint_id, "tool_id")
            tool_type = assembly.get_joint_attribute(joint_id, "tool_type")
            tools_strings.append("%s(%s)"%(tool_id, tool_type))
        messages += "%i;%s;%s\n" % (i + 1, beam_id, '+'.join(tools_strings))


    print (messages)

    rs.EditBox(messages, "Sequence (starts from 1), Beam ID", "Sequence")


def ui_preproduction_report(process):
    # type: (RobotClampAssemblyProcess) -> None
    beam_report(process)
    tool_report(process)
    screw_report(process)


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
            {'name': 'PreProductionReport', 'action': ui_preproduction_report
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

        # Run the selected command
        command_to_run = result['action']
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
