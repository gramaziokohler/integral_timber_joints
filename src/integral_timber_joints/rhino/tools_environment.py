import json
import os

import Rhino
import rhinoscriptsyntax as rs
from compas.utilities import DataDecoder
from compas_rhino.ui import CommandMenu
from compas_rhino.geometry import RhinoMesh

from compas_rhino.utilities.objects import get_object_name

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions
from integral_timber_joints.tools import Clamp, Gripper, ToolChanger, RobotWrist

def list_tools(process):
    # type: (RobotClampAssemblyProcess) -> None
    print("-- Clamps --")
    for clamp in process.clamps:
        print("  Clamp (id = %s) type = %s)" % (clamp.name, clamp.type_name))
        for link in clamp.links:
            print("  - Link %s with %i meshes" % (link.name, len(link.visual)))

    print("-- Gripper --")
    for gripper in process.grippers:
        print("  type: %s (id = %s))" % (gripper.type_name, gripper.name))
        for link in gripper.links:
            print("  - Link %s with %i meshes" % (link.name, len(link.visual)))
            
    print("-- Tool Changer --")
    print("  type: %s (id = %s))" % (process.robot_toolchanger.type_name, process.robot_toolchanger.name))

    print("-- Robot Wrist Meshes (collision sim only)--")
    print("  type: %s (id = %s))" % (process.robot_wrist.type_name, process.robot_wrist.name))
    for i, mesh in enumerate(process.robot_wrist.collision_mesh):
        print("  Mesh %i with %i vertices" % (i, mesh.number_of_vertices()))

    print("-- Env Meshes --")
    for i, mesh in enumerate(process.environment_meshes):
        print("  Mesh %i with %i vertices" % (i, mesh.number_of_vertices()))


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

def delete_clamp(process):
    # type: (RobotClampAssemblyProcess) -> None
    if len(process.clamps) == 0:
        print ("No clamps exist for you to delete.")
        return

    # List out current clamps for users to choose
    print ("Current Clamps:")
    ids = []
    for clamp in process.clamps:
        ids.append(clamp.name)
        print ("- clamp_id: %s, type: %s" %(clamp.name, clamp.type_name))
    # Ask user which clamp to delete
    id = rs.GetString("Which clamp to delete?", "Cancel", ["Cancel"] + ids)

    if id in ids:
        print ("%s is deleted" % (id))
        process.delete_clamp(id)
    
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

def delete_gripper(process):
    # type: (RobotClampAssemblyProcess) -> None
    if len(process.grippers) == 0:
        print ("No gripper exist for you to delete.")
        return

    # List out current gripper for users to choose
    print ("Current Gripper:")
    ids = []
    for gripper in process.grippers:
        ids.append(gripper.name)
        print ("- %s : %s" %(gripper.name, gripper.type_name))
    # Ask user which gripper to delete
    id = rs.GetString("Which gripper to delete?", "Cancel", ["Cancel"] + ids)

    if id in ids:
        print ("%s is deleted" % (id))
        process.delete_gripper(id)

def replace_toolchanger(process):
    # type: (RobotClampAssemblyProcess) -> None
    # Ask user for a json file
    path = rs.OpenFileName("Open", "ToolChanger File (*.json)|*.json|All Files (*.*)|*.*||")
    if path:
        with open(path, 'r') as f:
            # Deserialize asert correctness and add to Process
            tool = json.load(f, cls=DataDecoder)
            assert isinstance(tool, ToolChanger)
            process.robot_toolchanger = tool
            print("Tool Changer replaced with %s" % tool)


def replace_robot_wrist(process):
    # type: (RobotClampAssemblyProcess) -> None
    # Ask user for a json file
    path = rs.OpenFileName("Open", "ToolChanger File (*.json)|*.json|All Files (*.*)|*.*||")
    if path:
        with open(path, 'r') as f:
            # robot_wrist asert correctness and add to Process
            tool = json.load(f, cls=DataDecoder)
            assert isinstance(tool, RobotWrist)
            process.robot_wrist = tool
            print("Robot Wrist Object replaced with %s" % tool)

def add_env_model(process):
    # type: (RobotClampAssemblyProcess) -> None
    # Ask user to pick one or more mesh object
    guids = rs.GetObjects("Select Environment Mesh(es) (cannot contain convex hull)", filter=rs.filter.mesh)
    if guids is None:
        print("No mesh selected.")
        return
    # Show all current env mesh
    # todo

    # Create compas mesh and add it to process.environment_meshes
    for guid in guids:
        rhinomesh = RhinoMesh.from_guid(guid) 
        mesh = rhinomesh.to_compas()
        process.environment_meshes.append(mesh)
        # todo trigger artist to draw that mesh in correct layer

def delete_all_env_model(process):
    # type: (RobotClampAssemblyProcess) -> None

    # Show all current env mesh
    # todo
    
    # Ask user to reconfirm
    reconfirm = rs.GetString("Are You Sure?", "Yes", ["Yes", "No"])
    if reconfirm is None:
        return
    if reconfirm.startswith("Y") or reconfirm.startswith("y") :
        del process.environment_meshes[:]
        # process.environment_meshes.clear()
        # todo trigger artist to remove that mesh
        print ("All EnvMeshes are removed")

def copy_tools(process):
    # type: (RobotClampAssemblyProcess) -> None

    # If there are existing tools
    delete_old = False
    if len(list(process.clamps)) > 0 or len(list(process.grippers)) > 0:
        reconfirm = rs.GetString("There are %i Clamps, %i Grippers in current file, do you want to delete them?" % (len(list(process.clamps)), len(list(process.grippers))), "Delete", ["Delete", "Keep"])
        if reconfirm is None:
            return
        if reconfirm.startswith("D") or reconfirm.startswith("d") :
            delete_old = True

    # Ask user for a json file
    path = rs.OpenFileName("Open another process file to copy from", "Process File (*.json)|*.json|All Files (*.*)|*.*||")
    if path:
        with open(path, 'r') as f:
            # robot_wrist asert correctness and add to Process
            another_process = json.load(f, cls=DataDecoder) # type: RobotClampAssemblyProcess
        if delete_old:
            process.attributes['clamps'] = another_process.attributes['clamps']
            process.attributes['grippers'] = another_process.attributes['grippers']
            print("Old clamps and Grippers deleted, %i Clamps, %i Grippers imported." % (len(list(process.clamps)), len(list(process.grippers))))
        else:
            for id in another_process.attributes['clamps']:
                process.add_clamp(another_process.clamp(id))
            for id in another_process.attributes['grippers']:
                process.add_gripper(another_process.gripper(id))
            print("%i Clamps, %i Grippers imported. Now total: %i Clamps, %i Grippers" % (len(list(another_process.clamps)), len(list(another_process.grippers)), len(list(process.clamps)), len(list(process.grippers))))

def copy_tool_changer(process):
    # type: (RobotClampAssemblyProcess) -> None

    # Ask user for a json file
    path = rs.OpenFileName("Open another process file to copy from", "Process File (*.json)|*.json|All Files (*.*)|*.*||")
    if path:
        with open(path, 'r') as f:
            another_process = json.load(f, cls=DataDecoder) # type: RobotClampAssemblyProcess
            # Asert correctness and add to Process
            if another_process.robot_toolchanger is None:
                print("Opened file do not have robot_toolchanger")
                return
            process.robot_toolchanger = another_process.robot_toolchanger
            print("robot_toolchanger replaced by %s. " % process.robot_toolchanger.name)

def copy_rob_wrist(process):
    # type: (RobotClampAssemblyProcess) -> None

    # Ask user for a json file
    path = rs.OpenFileName("Open another process file to copy from", "Process File (*.json)|*.json|All Files (*.*)|*.*||")
    if path:
        with open(path, 'r') as f:
            another_process = json.load(f, cls=DataDecoder) # type: RobotClampAssemblyProcess
            # Asert correctness and add to Process
            if another_process.robot_wrist_collision_mesh is None:
                print("Opened file do not have robot_wrist_collision_mesh")
                return
            process.robot_wrist_collision_mesh = another_process.robot_wrist_collision_mesh
            print("robot_wrist_collision_mesh replaced by %s. " % process.robot_wrist_collision_mesh.name)

def copy_env_models(process):
    # type: (RobotClampAssemblyProcess) -> None

    # If there are existing tools
    delete_old = False
    if len(list(process.environment_meshes)) > 0 :
        reconfirm = rs.GetString("There are %i Environment Mesh(es) in current file, do you want to delete them?" % (len(list(process.environment_meshes))), "Delete", ["Delete", "Keep"])
        if reconfirm is None:
            return
        if reconfirm.startswith("D") or reconfirm.startswith("d") :
            delete_old = True

    # Ask user for a json file
    path = rs.OpenFileName("Open another process file to copy from", "Process File (*.json)|*.json|All Files (*.*)|*.*||")
    if path:
        with open(path, 'r') as f:
            # robot_wrist asert correctness and add to Process
            another_process = json.load(f, cls=DataDecoder) # type: RobotClampAssemblyProcess
        if delete_old:
            process.environment_meshes = another_process.environment_meshes
            print("Old Environment Mesh(es) deleted, %i Environment Mesh(es) imported." % (len(list(process.environment_meshes))))
        else:
            for mesh in another_process.environment_meshes:
                process.environment_meshes.append(mesh)
            print("%i Environment Mesh(es) imported. Now total: %i Environment Mesh(es)" % (len(list(another_process.environment_meshes)), len(list(process.environment_meshes))))



def not_implemented(process):
    #
    print('This function is not implemented')


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
                    {'name': 'DeleteClamp', 'action': delete_clamp},
                ]},
                {'name': 'Gripper', 'message': 'Process have %i Gripper:' % len(list(process.grippers)), 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'AddGripper', 'action': add_gripper},
                    {'name': 'DeleteGripper', 'action': delete_gripper},
                ]},
                {'name': 'RobotToolChanger', 'message': 'Process robot_toolchanger:', 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'ReplaceToolChanger', 'action': replace_toolchanger},
                ]},
                {'name': 'RobotWrist', 'message': 'Process robot_wrist', 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'ReplaceRobotWrist', 'action': replace_robot_wrist},
                ]},
                {'name': 'EnvModels', 'message':  'We have %i Env Meshes:' % len(list(process.environment_meshes)), 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'AddEnvModel', 'action': add_env_model},
                    {'name': 'DeleteEnvModel', 'action': not_implemented},
                    {'name': 'DeleteAllModel', 'action': delete_all_env_model},
                ]},
                {'name': 'CopyFromAnotherFile', 'message':  'Copy from another file:', 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'CopyTools', 'action': copy_tools},
                    {'name': 'CopyToolChanger', 'action': copy_tool_changer},
                    {'name': 'CopyRobWrist', 'action': copy_rob_wrist},
                    {'name': 'CopyEnvModel', 'action': copy_env_models},
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
