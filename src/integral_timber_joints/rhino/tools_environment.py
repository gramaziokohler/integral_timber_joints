import json
import os

import Rhino
import rhinoscriptsyntax as rs
from compas.utilities import DataDecoder
from compas_rhino.ui import CommandMenu
from compas_rhino.utilities.objects import get_object_name

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions
from integral_timber_joints.tools import Clamp, Gripper

def list_tools(process):
    # type: (RobotClampAssemblyProcess) -> None
    print("-- Clamps --")
    for clamp in process.clamps:
        print("  type: %s (id = %s))" % (clamp.type_name, clamp.name))

    print("-- Gripper --")
    for gripper in process.grippers:
        print("  type: %s (id = %s))" % (gripper.type_name, gripper.name))

    print("-- Tool Changer --")
    print("  type: %s (id = %s))" % (process.robot_toolchanger.type_name, process.robot_toolchanger.name))

    print("-- Robot Wrist Meshes (collision sim only)--")
    for mesh in process.robot_wrist_collision_mesh:
        print("  mesh with %i vertices))" % (len(mesh.vertices)))

    print("-- Env Meshes --")
    for mesh in process.environment_meshes:
        print("  mesh with %i vertices))" % (len(mesh.vertices)))

def get_next_clamp_id(process):
    # type: (RobotClampAssemblyProcess) -> str
    clamp_ids = [tool.name for tool in process.clamps]
    for i in range (1, 100):
        id = 'c%i'%i
        if id not in clamp_ids:
            return id

def get_next_gripper_id(process):
    gripper_ids = [tool.name for tool in process.grippers]
    for i in range (1, 100):
        id = 'g%i'%i
        if id not in gripper_ids:
            return id

def add_clamp(process):
    # type: (RobotClampAssemblyProcess) -> None
    # Ask user for a json file
    path = rs.OpenFileName("Open", "Clamp File (*.json)|*.json|All Files (*.*)|*.*||")
    if path:
        with open(path, 'r') as f:
            # Deserialize asert correctness and add to Process
            tool = json.load(f, cls=DataDecoder)
            assert isinstance(tool, Clamp)
            # Ask user for a new id 
            name = rs.GetString("Type in clamp id typically c1,c2, etc..", get_next_clamp_id(process))
            if name is not None: 
                tool.name = name
                process.add_clamp(tool)
                print("Clamp added: %s " % tool)

def add_gripper(process):
    # type: (RobotClampAssemblyProcess) -> None
    # Ask user for a json file
    path = rs.OpenFileName("Open", "Gripper File (*.json)|*.json|All Files (*.*)|*.*||")
    if path:
        with open(path, 'r') as f:
            # Deserialize asert correctness and add to Process
            tool = json.load(f, cls=DataDecoder)
            assert isinstance(tool, Gripper)
            # Ask user for a new id 
            name = rs.GetString("Type in gripper id typically g1,g2, etc..", get_next_gripper_id(process))
            if name is not None: 
                tool.name = name
                process.add_gripper(tool)
                print("Clamp added: %s " % tool)

def replace_toolchanger(process):
    # type: (RobotClampAssemblyProcess) -> None
    # Ask user for a json file
    path = rs.OpenFileName("Open", "ToolChanger File (*.json)|*.json|All Files (*.*)|*.*||")
    if path:
        with open(path, 'r') as f:
            # Deserialize asert correctness and add to Process
            tool = json.load(f, cls=DataDecoder)
            assert tool.type_name == "ToolChanger"
            process.toolchanger = tool
            print("Tool Changer replaced with %s" % tool)


def replace_robot_wrist(process):
    # type: (RobotClampAssemblyProcess) -> None
    # Ask user for a json file
    path = rs.OpenFileName("Open", "ToolChanger File (*.json)|*.json|All Files (*.*)|*.*||")
    if path:
        with open(path, 'r') as f:
            # robot_wrist asert correctness and add to Process
            tool = json.load(f, cls=DataDecoder)
            assert tool.type_name == "RobotWrist"
            process.robot_wrist_collision_mesh = tool
            print("Robot Wrist Collision Object replaced with %s" % tool)


def something(process):
    #
    print('something not implemented')


def show_menu(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    while (True):
        # Create Menu
        config = {
            'message': 'Assembly contains %i Clamps, %i Grippers, %i EnvMeshs:' % (len(list(process.clamps)), len(list(process.grippers)), len(list(process.environment_meshes))),
            'options': [
                {'name': 'Finish', 'action': 'Exit'
                 },
                {'name': 'ListTools', 'action': list_tools
                 },
                {'name': 'Clamps', 'message': 'Process have %i Clamps:' % len(list(process.clamps)), 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'AddClamp', 'action': add_clamp},
                    {'name': 'DeleteClamp', 'action': something},
                ]},
                {'name': 'Gripper', 'message': 'Process have %i Gripper:' % len(list(process.grippers)), 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'AddGripper', 'action': add_gripper},
                    {'name': 'DeleteGripper', 'action': something},
                ]},
                {'name': 'RobotToolChanger', 'message': 'Process robot_toolchanger:', 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'ReplaceToolChanger', 'action': replace_toolchanger},
                ]},
                {'name': 'RobotWristColliMesh', 'message': 'Process robot_wrist_collision_mesh:', 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'ReplaceRobotWristColliMesh', 'action': replace_robot_wrist},
                ]},
                {'name': 'EnvModels', 'message':  'We have %i Env Meshes:' % len(list(process.environment_meshes)), 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'AddEnvModel', 'action': something},
                    {'name': 'DeleteEnvModel', 'action': something},
                    {'name': 'DeleteAllModel', 'action': something},
                ]},
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
