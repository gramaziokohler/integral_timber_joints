import json
import os

import Rhino  # type: ignore
import rhinoscriptsyntax as rs  # type: ignore
from compas.geometry.primitives.frame import Frame
from compas.utilities import DataDecoder
from compas_fab.robots.configuration import Configuration
from compas_rhino.geometry import RhinoMesh
from compas_rhino.ui import CommandMenu
from compas_rhino.utilities.objects import get_object_name

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.geometry.env_model import EnvironmentModel
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions
from integral_timber_joints.tools import Clamp, Screwdriver, Gripper, RobotWrist, ToolChanger

try:
    from typing import Dict, List, Optional, Tuple, Type
except:
    pass


def list_tools(process):
    # type: (RobotClampAssemblyProcess) -> None
    print("-- Clamps --")
    for clamp in sorted(process.clamps, key=lambda tool: tool.name):
        print("  Clamp (id = %s) type = %s)" % (clamp.name, clamp.type_name))
        for link in clamp.links:
            print("  - Link %s with %i meshes" % (link.name, len(link.visual)))

    print("-- Screwdrivers --")
    for screwdriver in sorted(process.screwdrivers, key=lambda tool: tool.name):
        print("  Screwdriver (id = %s) type = %s)" % (screwdriver.name, screwdriver.type_name))
        for link in screwdriver.links:
            print("  - Link %s with %i meshes" % (link.name, len(link.visual)))

    print("-- Gripper --")
    for gripper in sorted(process.grippers, key=lambda tool: tool.name):
        print("  type: %s (id = %s))" % (gripper.type_name, gripper.name))
        for link in gripper.links:
            print("  - Link %s with %i meshes" % (link.name, len(link.visual)))

    print("-- Tool Changer --")
    print("  type: %s (id = %s))" % (process.robot_toolchanger.type_name, process.robot_toolchanger.name))
    for i, mesh in enumerate(process.robot_toolchanger.collision_mesh):
        print("  Mesh %i with %i vertices" % (i, mesh.number_of_vertices()))

    print("-- Robot Wrist Meshes (collision sim only)--")
    print("  type: %s (id = %s))" % (process.robot_wrist.type_name, process.robot_wrist.name))
    for i, mesh in enumerate(process.robot_wrist.collision_mesh):
        print("  Mesh %i with %i vertices" % (i, mesh.number_of_vertices()))

    print("-- Env Meshes --")
    print("  %i Environment Model(es) in current file." % len(process.environment_models))


def get_next_tool_id(process, cls):
    # type: (RobotClampAssemblyProcess, Type) -> str
    if cls is Clamp:
        prefix = 'c'
    elif cls is Screwdriver:
        prefix = 's'
    elif cls is Gripper:
        prefix = 'g'
    else:
        raise TypeError("%s Class is weird." % cls.__name__)

    existing_ids = process.tool_ids
    for i in range(1, 100):
        id = prefix + '%i' % i
        if id not in existing_ids:
            return id

###########
# Add Tools
###########


def add_tool(process, cls):
    # type: (RobotClampAssemblyProcess, Type) -> None
    artist = get_process_artist()

    # Ask user for a json file
    tool_type_name = cls.__name__
    path = rs.OpenFileName("Open", tool_type_name + " File (*.json)|*.json|All Files (*.*)|*.*||")
    if path:
        with open(path, 'r') as f:
            # Deserialize asert correctness and add to Process
            tool = json.load(f, cls=DataDecoder)
            assert tool.__class__ is cls
            # Ask user for a new id
            name = rs.GetString("Type in " + tool_type_name + " id typically c1,c2, etc.. Or use Default ", get_next_tool_id(process, cls))
            if name is not None:
                tool.name = name
                process.add_tool(tool)
                artist.draw_tool_in_storage(tool.name)
                print("Tool added: %s(%s) " % (tool_type_name, tool.name))


def add_clamp(process):
    # type: (RobotClampAssemblyProcess) -> None
    add_tool(process, Clamp)


def add_screwdriver(process):
    # type: (RobotClampAssemblyProcess) -> None
    add_tool(process, Screwdriver)


def add_gripper(process):
    # type: (RobotClampAssemblyProcess) -> None
    add_tool(process, Gripper)

##############
# Delete Tools
##############


def delete_tool(process, cls):
    # type: (RobotClampAssemblyProcess, Type) -> None
    tool_type_name = cls.__name__

    # Gather available tools
    existing_tools_ids = [tool_id for tool_id in process.tool_ids
                          if process.tool(tool_id).__class__ is cls]
    if len(existing_tools_ids) == 0:
        print("No %s exist for you to delete." % tool_type_name)
        return

    # List out current clamps for users to choose
    print("Current %s:" % tool_type_name)
    for tool_id in existing_tools_ids:
        tool = process.tool(tool_id)
        print("- tool_id: %s, type: %s" % (tool.name, tool.type_name))

    # Ask user which clamp to delete
    tool_id = rs.GetString("Which %s to delete?" % tool_type_name, "Cancel", ["Cancel"] + existing_tools_ids)
    if tool_id in existing_tools_ids:
        process.delete_tool(tool_id)
        artist = get_process_artist()
        artist.delete_tool_in_storage(tool_id)


def delete_clamp(process):
    # type: (RobotClampAssemblyProcess) -> None
    delete_tool(process, Clamp)


def delete_screwdriver(process):
    # type: (RobotClampAssemblyProcess) -> None
    delete_tool(process, Screwdriver)


def delete_gripper(process):
    # type: (RobotClampAssemblyProcess) -> None
    delete_tool(process, Gripper)

###############
# Replace Tools
###############


def replace_tool(process, cls):
    # type: (RobotClampAssemblyProcess, Type) -> None
    tool_type_name = cls.__name__

    # Gather available tools
    existing_tools_ids = [tool_id for tool_id in process.tool_ids
                          if process.tool(tool_id).__class__ is cls]
    if len(existing_tools_ids) == 0:
        print("No %s exist for you to delete." % tool_type_name)
        return

    # List out current tools for users to choose
    print("Current %s:" % tool_type_name)
    for tool_id in existing_tools_ids:
        tool = process.tool(tool_id)
        print("- tool_id: %s, type: %s" % (tool.name, tool.type_name))

    # Ask user which tool to delete
    tool_id = rs.GetString("Which %s to replace? (id and storage frame are kept)" % tool_type_name, "Cancel", ["Cancel"] + existing_tools_ids)

    if tool_id in existing_tools_ids:
        old_tool = process.tool(tool_id)
        path = rs.OpenFileName("Open", "%s File (*.json)|*.json|All Files (*.*)|*.*||" % tool_type_name)
        with open(path, 'r') as f:
            # Deserialize asert correctness and add to Process
            new_tool = json.load(f, cls=DataDecoder)
            assert tool.__class__ is cls

            # Copy over old attributes
            new_tool.name = old_tool.name
            new_tool.tool_storage_frame = old_tool.tool_storage_frame
            new_tool.tool_storage_configuration = old_tool.tool_storage_configuration
        artist = get_process_artist()
        artist.delete_tool_in_storage(tool_id)
        process.delete_tool(tool_id)
        process.add_tool(new_tool)
        artist.draw_tool_in_storage(tool_id)
        print("%s is replaced by %s." % (old_tool, new_tool))


def replace_clamp(process):
    # type: (RobotClampAssemblyProcess) -> None
    replace_tool(process, Clamp)


def replace_screwdriver(process):
    # type: (RobotClampAssemblyProcess) -> None
    replace_tool(process, Screwdriver)


def replace_gripper(process):
    # type: (RobotClampAssemblyProcess) -> None
    replace_tool(process, Gripper)


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


#######
# Robot
#######

def set_robot_initial_config(process):
    # type: (RobotClampAssemblyProcess) -> None
    # Ask user to pick a Configuration json
    path = rs.OpenFileName("Open a Configuration json, representing initial robot config.", "Configuration File (*.json)|*.json|All Files (*.*)|*.*||")
    if path:
        with open(path, 'r') as f:
            configuration = json.load(f, cls=DataDecoder)  # type: Configuration
            assert configuration.__class__.__name__ == Configuration.__name__
            process.robot_initial_config = configuration
            print("Robot Initial Config is successfully loaded from %s" % path)


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


############
# Env Models
############


def add_env_model(process):
    # type: (RobotClampAssemblyProcess) -> None
    # Ask user to pick one or more mesh object
    guids = rs.GetObjects("Select Mesh(es) for Environment Model (cannot contain convex hull)", filter=rs.filter.mesh)
    if guids is None:
        print("No mesh selected.")
        return
    # Show all current env mesh
    artist = get_process_artist()
    artist.show_all_env_mesh()

    # Create compas mesh and add it to process.environment_models
    for guid in guids:
        rhinomesh = RhinoMesh.from_guid(guid)
        environment_model = rhinomesh.to_compas(EnvironmentModel)  # type: EnvironmentModel
        process.add_environment_model(environment_model)
    # trigger artist to draw that mesh in correct layer
    artist.draw_all_env_mesh(delete_old=True)

    for beam_id in process.assembly.sequence:
        process.dependency.invalidate(beam_id, process.create_actions_from_sequence)



def delete_all_env_model(process):
    # type: (RobotClampAssemblyProcess) -> None

    # Show all current env mesh
    # todo

    # Ask user to reconfirm
    reconfirm = rs.GetString("Are You Sure?", "Yes", ["Yes", "No"])
    if reconfirm is None:
        return
    if reconfirm.startswith("Y") or reconfirm.startswith("y"):
        artist = get_process_artist()
        artist.delete_all_env_mesh()
        process.environment_models = {}

        for beam_id in process.assembly.sequence:
            process.dependency.invalidate(beam_id, process.create_actions_from_sequence)
        print("All EnvMeshes are removed")


def copy_tools(process):
    # type: (RobotClampAssemblyProcess) -> None
    """Copying tools from another process file."""

    # If there are existing tools
    delete_old = False
    if len(list(process.tool_ids)):
        reconfirm = rs.GetString("There are %i Clamps, %i Screwdrivers, %i Grippers in current file, do you want to delete them?" %
                                 (len(list(process.clamps)), len(list(process.screwdrivers)), len(list(process.grippers))), "Delete", ["Delete", "Keep"])
        if reconfirm is None:
            return
        if reconfirm.startswith("D") or reconfirm.startswith("d"):
            delete_old = True

    # Ask user for a json file
    path = rs.OpenFileName("Open another process file to copy from", "Process File (*.json)|*.json|All Files (*.*)|*.*||")
    if path:
        with open(path, 'r') as f:
            # robot_wrist asert correctness and add to Process
            another_process = json.load(f, cls=DataDecoder)  # type: RobotClampAssemblyProcess
        if delete_old:
            process.attributes['clamps'] = another_process.attributes['clamps']
            process.attributes['screwdrivers'] = another_process.attributes['screwdrivers']
            process.attributes['grippers'] = another_process.attributes['grippers']
            print("Old clamps and Grippers deleted, %i Clamps, %i Screwdrivers, %i Grippers imported." % (len(list(process.clamps)), len(list(process.screwdrivers)), len(list(process.grippers))))
        else:
            for tool in another_process.tools:
                process.add_tool(tool)

            print("%i Clamps, %i Screwdrivers, %i Grippers imported. Now total: %i Clamps, %i Clamps, %i Grippers" % (
                len(list(another_process.clamps)), len(list(another_process.screwdrivers)), len(list(another_process.grippers)),
                len(list(process.clamps)), len(list(process.screwdrivers)), len(list(process.grippers))))

    artist = get_process_artist()
    artist.delete_all_tools_in_storage()
    for tool_id in process.tool_ids:
        artist.draw_tool_in_storage(tool_id)


def copy_tool_changer(process):
    # type: (RobotClampAssemblyProcess) -> None

    # Ask user for a json file
    path = rs.OpenFileName("Open another process file to copy from", "Process File (*.json)|*.json|All Files (*.*)|*.*||")
    if path:
        with open(path, 'r') as f:
            another_process = json.load(f, cls=DataDecoder)  # type: RobotClampAssemblyProcess
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
            another_process = json.load(f, cls=DataDecoder)  # type: RobotClampAssemblyProcess
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
    if len(process.environment_models) > 0:
        reconfirm = rs.GetString("There are %i Environment Model(es) in current file, do you want to delete them?" %
                                 (len(process.environment_models)), "Delete", ["Delete", "Keep"])
        if reconfirm is None:
            return
        if reconfirm.startswith("D") or reconfirm.startswith("d"):
            delete_old = True

    # Ask user for a json file
    path = rs.OpenFileName("Open another process file to copy from", "Process File (*.json)|*.json|All Files (*.*)|*.*||")
    if path:
        with open(path, 'r') as f:
            # robot_wrist asert correctness and add to Process
            another_process = json.load(f, cls=DataDecoder)  # type: RobotClampAssemblyProcess
        if delete_old:
            process.environment_models = another_process.environment_models
            print("Old Environment Model(es) deleted, %i Environment Model(es) imported." % (len(process.environment_models)))
        else:
            for env_id, env_model in process.environment_models.items():
                process.environment_models[env_id] = env_model
            print("%i Environment Model(es) imported. Now total: %i Environment Model(es)" % (len(another_process.environment_models), len(process.environment_models)))


def copy_all(process):
    # type: (RobotClampAssemblyProcess) -> None
    reconfirm = rs.GetString("There are %i Clamps, %i Screwdrivers, %i Grippers, ToolChanger, RobotWrist, %i Environment Model(es) in current file, do you want to delete them and replace by new ones?" % (
        len(process.environment_models),
        len(list(process.clamps)),
        len(list(process.screwdrivers)),
        len(list(process.grippers))),
        "Delete", ["DeleteAndContinue", "Cancel"])
    if reconfirm is None:
        return
    if reconfirm.startswith("C") or not reconfirm.startswith("D"):
        return

    # Ask user for a json file
    path = rs.OpenFileName("Open another process file to copy from", "Process File (*.json)|*.json|All Files (*.*)|*.*||")
    if path:
        with open(path, 'r') as f:
            # robot_wrist asert correctness and add to Process
            another_process = json.load(f, cls=DataDecoder)  # type: RobotClampAssemblyProcess

        process.attributes['clamps'] = another_process.attributes['clamps']
        process.attributes['grippers'] = another_process.attributes['grippers']
        process.attributes['screwdrivers'] = another_process.attributes['screwdrivers']
        print("Old clamps and Grippers deleted, %i Clamps, %i Screwdrivers, %i Grippers imported." % (len(list(process.clamps)), len(list(process.screwdrivers)), len(list(process.grippers))))

        process.robot_toolchanger = another_process.robot_toolchanger
        process.robot_wrist = another_process.robot_wrist
        print("Toolchanger is replaced by %s, RobotWrist is replaced by %s." % (process.robot_toolchanger.type_name, process.robot_wrist.type_name))

        process.environment_models = another_process.environment_models
        print("Old Environment Model(es) deleted, %i Environment Model(es) imported." % (len(process.environment_models)))


##############################
# Tool Storage / Configuration
##############################


def ui_ask_user_for_tool_id(process, message="Which Tool ID", print_existing_tools=True, include_clamp=True, inclde_gripper=True, inclde_screwdriver=True):
    # type: (RobotClampAssemblyProcess,str, bool, bool, bool, bool) -> str
    """Note that the list of tools names available must start with a letter character, cannot be numbers"""
    ids = []
    if include_clamp:
        if print_existing_tools:
            print("-- Existing Clamps --")
        for clamp in process.clamps:
            ids.append(clamp.name)
            if print_existing_tools:
                print("  Clamp (id = %s) type = %s)" % (clamp.name, clamp.type_name))
    if inclde_screwdriver:
        if print_existing_tools:
            print("-- Existing Clamps --")
        for screwdriver in process.screwdrivers:
            ids.append(screwdriver.name)
            if print_existing_tools:
                print("  Screwdriver (id = %s) type = %s)" % (screwdriver.name, screwdriver.type_name))

    if inclde_gripper:
        if print_existing_tools:
            print("-- Existing Gripper --")
        for gripper in process.grippers:
            ids.append(gripper.name)
            if print_existing_tools:
                print("  type: %s (id = %s))" % (gripper.type_name, gripper.name))

    # Return wNone if nothing is available for picking
    if len(ids) == 0:
        print("Error: No existing tool for picking.")
        return None

    # Ask user to pick from a list
    tool_id = rs.GetString(message, "Finish", ["Finish"] + sorted(ids))
    # Ask user for 3 point input
    if tool_id in ids:
        return tool_id
    elif tool_id == "Finish":
        return None
    else:
        print("%s is not a valid input." % tool_id)
        return None


def set_tool_storage_frame(process, set_from_robot_flange=False):
    # type: (RobotClampAssemblyProcess, bool) -> None
    """Function invoked by user to set the storage position."""

    while True:
        # Ask user which tool to set
        tool_id = ui_ask_user_for_tool_id(process, "Change storage position for which tool?")

        # Ask user for 3 point input
        if tool_id is None:
            return

        # Ask for origin
        if set_from_robot_flange:
            origin = rs.GetPoint("Pick point as robot flange origin for %s in storage." % tool_id)
            x_point = rs.GetPoint("Pick point on robot flange X direction for %s in storage." % tool_id)
            y_point = rs.GetPoint("Pick point on robot flange Y direction for %s in storage." % tool_id)

            robot_flange_frame = Frame(origin, x_point - origin, y_point - origin)
            tool_changer = process.robot_toolchanger
            tool_changer.current_frame = robot_flange_frame
            tool_storage_frame = tool_changer.current_tcf
        else:
            origin = rs.GetPoint("Pick point as frame origin for %s in storage." % tool_id)
            x_point = rs.GetPoint("Pick point on frame X direction for %s in storage." % tool_id)
            y_point = rs.GetPoint("Pick point on frame Y direction for %s in storage." % tool_id)
            tool_storage_frame = Frame(origin, x_point - origin, y_point - origin)

        # Alignment frame
        process.tool(tool_id).tool_storage_frame = tool_storage_frame
        print("Tool %s Storage Frame changed to : %s " % (tool_id, tool_storage_frame))

        # Invalidate tool storage related calculations
        # process.dependency.invalidate(something about storage position)
        # Storage positions do not actually have much computation. It is not visualized in Grasp Pose
        # It is only used directly when Actions create Movements.

        # Redraw Tool
        artist = get_process_artist()
        artist.draw_tool_in_storage(tool_id, delete_old=True)
        rs.EnableRedraw(True)


def set_tool_storage_frame_from_robot_flange(process):
    # type: (RobotClampAssemblyProcess) -> None
    """Function invoked by user to set the storage position."""
    if process.robot_toolchanger is None:
        print("robot_toolchanger is None. Load the toolchanger first.")
        return False
    set_tool_storage_frame(process, True)


def set_tool_storage_configuration(process):
    # type: (RobotClampAssemblyProcess) -> None
    """Function invoked by user to set the storage configuration."""
    while True:
        # Ask user which tool to set
        tool_id = ui_ask_user_for_tool_id(process, "Set storage configuration for which tool?")
        if tool_id is None:
            return

        # Ask user for 3 point input
        path = rs.OpenFileName("Open a Configuration json, for Tool %s storage configuration." % tool_id, "Configuration File (*.json)|*.json|All Files (*.*)|*.*||")
        if path:
            with open(path, 'r') as f:
                configuration = json.load(f, cls=DataDecoder)  # type: Configuration
                assert isinstance(configuration, Configuration)
                process.tool(tool_id).tool_storage_configuration = configuration
                print("Tool Storage Configuration is successfully loaded from %s" % path)

                # Perform FK with robot model to get flange frame and tool frame
                configuration_mm = configuration.scaled(1000)
                robot_flange_frame = process.robot_model.forward_kinematics(configuration_mm, process.ROBOT_END_LINK)
                toolchanger = process.robot_toolchanger
                toolchanger.current_frame = robot_flange_frame
                tool_frame = toolchanger.current_tcf
                process.tool(tool_id).tool_storage_frame = tool_frame
                print("Tool storage frame set at %s" % tool_frame)

        else:
            print("Function Canceled")


def remove_tool_storage_configuration(process):
    # type: (RobotClampAssemblyProcess) -> None
    """Function invoked by user to remove the storage configuration."""

    while True:
        # Ask user which tool to set
        tool_id = ui_ask_user_for_tool_id(process, "Remove storage configuration for which tool?")
        if tool_id is None:
            return

        process.tool(tool_id).tool_storage_configuration = None
        print("Tool %s Storage Configuration is successfully removed." % tool_id)


def not_implemented(process):
    #
    print('This function is not implemented')


def show_menu(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    while (True):

        # Have artist paint all the tools in storage position and env mesh
        for tool_id in process.tool_ids:
            artist.draw_tool_in_storage(tool_id, delete_old=True)

        artist.draw_robot(process.robot_initial_config)
        artist.draw_all_env_mesh(delete_old=True)
        rs.EnableRedraw(True)

        # Create Menu
        config = {
            'message': 'Process contains %i Clamps, %i Screwdrivers, %i Grippers, %i EnvMeshs:' % (len(list(process.clamps)), len(list(process.screwdrivers)), len(list(process.grippers)), len(process.environment_models)),
            'options': [
                {'name': 'Finish', 'action': 'Exit'
                 },
                {'name': 'ListTools', 'action': list_tools
                 },
                {'name': 'Clamps', 'message': 'Process have %i Clamps:' % len(list(process.clamps)), 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'AddClamp', 'action': add_clamp},
                    {'name': 'DeleteClamp', 'action': delete_clamp},
                    {'name': 'ReplaceClamp', 'action': replace_clamp},
                ]},
                {'name': 'Screwdrivers', 'message': 'Process have %i Screwdrivers:' % len(list(process.screwdrivers)), 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'AddScrewdriver', 'action': add_screwdriver},
                    {'name': 'DeleteScrewdriver', 'action': delete_screwdriver},
                    {'name': 'ReplaceScrewdrivers', 'action': replace_screwdriver},
                ]},
                {'name': 'Gripper', 'message': 'Process have %i Gripper:' % len(list(process.grippers)), 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'AddGripper', 'action': add_gripper},
                    {'name': 'DeleteGripper', 'action': delete_gripper},
                    {'name': 'ReplaceGripper', 'action': replace_gripper},
                ]},
                {'name': 'SetToolStorage', 'message': 'Setting Tool Storage', 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'SetFrameFromToolFrame', 'action': set_tool_storage_frame},
                    {'name': 'SetFrameFromRobotFlangeFrame', 'action': set_tool_storage_frame_from_robot_flange},
                    {'name': 'SetConfiguration', 'action': set_tool_storage_configuration},
                    {'name': 'RemoveConfiguration', 'action': remove_tool_storage_configuration},
                ]},
                {'name': 'RobotToolChanger', 'message': 'Process robot_toolchanger:', 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'ReplaceToolChanger', 'action': replace_toolchanger},
                ]},
                {'name': 'Robot', 'message': 'Process Robot is: %s' % process.robot_model, 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'SetInitialConfig', 'action': set_robot_initial_config},
                ]},
                {'name': 'RobotWrist', 'message': 'Process robot_wrist', 'options': [
                    {'name': 'Back', 'action': 'Back'},
                    {'name': 'ReplaceRobotWrist', 'action': replace_robot_wrist},
                ]},
                {'name': 'EnvModels', 'message':  'We have %i Env Meshes:' % len(process.environment_models), 'options': [
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
                    {'name': 'CopyAll', 'action': copy_all},
                ]},

            ]

        }

        result = CommandMenu(config).select_action()
        # User cancel command by Escape
        if result is None or 'action' not in result:
            artist.hide_all_tools_in_storage()
            artist.hide_all_env_mesh()
            artist.hide_robot()
            rs.EnableRedraw(True)
            print('Exit Function')
            return Rhino.Commands.Result.Cancel

        action = result['action']

        # User click Exit Button
        if action == 'Exit':
            artist.hide_all_tools_in_storage()
            artist.hide_all_env_mesh()
            artist.hide_robot()
            rs.EnableRedraw(True)
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
