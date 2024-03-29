import json
import os

import Rhino  # type: ignore
import rhinoscriptsyntax as rs
from compas.geometry.primitives.frame import Frame
from compas.geometry.primitives.vector import Vector
from compas.utilities import DataDecoder
from compas_fab.robots.configuration import Configuration
from compas_rhino.geometry import RhinoMesh, RhinoPoint
from compas_rhino.ui import CommandMenu
from compas_rhino.utilities.objects import get_object_name

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.tools_environment import ui_ask_user_for_tool_id
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions
from integral_timber_joints.tools import Clamp, Gripper, GripperAlignedPickupStation, PickupStation, StackedPickupStation
from integral_timber_joints.tools.beam_storage import BeamStorage


def create_pickup_station(process):
    # type: (RobotClampAssemblyProcess) -> None
    station = PickupStation()
    artist = get_process_artist()

    # Ask user for collision model
    guids = rs.GetObjects("Pick mesh(es) as collision objects.", preselect=False, select=False, group=False, filter=rs.filter.mesh)
    if guids is None:
        guids = []
    for guid in guids:
        cm = RhinoMesh.from_guid(guid).to_compas()
        station.collision_meshes.append(cm)
        print(cm)

    # frame = Frame() # type: compas.geometry.primitives.frame.Frame

    # Ask for origin
    origin = rs.GetPoint("Pick point as alignment origin.")
    # Ask for X axis and Y Axis
    x_point = rs.GetPoint("Pick point on X direction.")
    y_point = rs.GetPoint("Pick point on Y direction.")

    # Alignment frame
    alignment_frame = Frame(origin, x_point - origin, y_point - origin)
    station.alignment_frame = alignment_frame
    print(alignment_frame)

    # Ask for pickup retract direction
    retraction_point = rs.GetPoint("Pick point on retraction direction at correct distance from origin.")
    pickup_retract_vector = Vector.from_start_end(origin, retraction_point)
    station.pickup_retract_vector = pickup_retract_vector

    # Set X Y Z alignment

    # Save station to process
    process.pickup_station = station

    # Invalidate process
    [process.dependency.invalidate(beam_id, process.compute_pickup_frame) for beam_id in process.assembly.sequence]

    # Artist deltete related geometry
    for beam_id in process.assembly.sequence:
        artist.delete_beam_at_position(beam_id, 'assembly_wcf_pickupapproach')
        artist.delete_beam_at_position(beam_id, 'assembly_wcf_pickup')
        artist.delete_beam_at_position(beam_id, 'assembly_wcf_pickupretract')
        artist.delete_gripper_at_position(beam_id, 'assembly_wcf_pickupapproach')
        artist.delete_gripper_at_position(beam_id, 'assembly_wcf_pickup')
        artist.delete_gripper_at_position(beam_id, 'assembly_wcf_pickupretract')


def create_gripper_aligning_pickup_station(process):
    # type: (RobotClampAssemblyProcess) -> None
    station = GripperAlignedPickupStation()
    artist = get_process_artist()

    # Ask user for collision model
    guids = rs.GetObjects("Pick mesh(es) as collision objects.", preselect=False, select=False, group=False, filter=rs.filter.mesh)
    if guids is None:
        guids = []
    for guid in guids:
        cm = RhinoMesh.from_guid(guid).to_compas()
        station.collision_meshes.append(cm)

    # Ask for origin
    origin = rs.GetPoint("Pick point as default alignment origin.")
    # Ask for X axis and Y Axis
    x_point = rs.GetPoint("Pick point on X direction. (length of beam)")
    y_point = rs.GetPoint("Pick point on Y direction. (side)")

    # Alignment frame
    alignment_frame = Frame(origin, x_point - origin, y_point - origin)
    station.alignment_frame = alignment_frame
    print(alignment_frame)

    # Ask for pickup retract direction
    retraction_point = rs.GetPoint("Pick point on retraction direction at correct distance from origin.")
    pickup_retract_vector = Vector.from_start_end(origin, retraction_point)
    station.pickup_retract_vector = pickup_retract_vector

    # Save station to process
    process.pickup_station = station

    # Invalidate process
    [process.dependency.invalidate(beam_id, process.compute_pickup_frame) for beam_id in process.assembly.sequence]

    # Artist deltete related geometry
    for beam_id in process.assembly.sequence:
        artist.delete_beam_at_position(beam_id, 'assembly_wcf_pickupapproach')
        artist.delete_beam_at_position(beam_id, 'assembly_wcf_pickup')
        artist.delete_beam_at_position(beam_id, 'assembly_wcf_pickupretract')
        artist.delete_gripper_at_position(beam_id, 'assembly_wcf_pickupapproach')
        artist.delete_gripper_at_position(beam_id, 'assembly_wcf_pickup')
        artist.delete_gripper_at_position(beam_id, 'assembly_wcf_pickupretract')


def create_stacked_pickup_station(process):
    pass


def import_pickup_station(process):
    pass


def export_pickup_station(process):
    pass


def create_beam_storage(process):
    # type: (RobotClampAssemblyProcess) -> None
    beam_storage = BeamStorage()
    artist = get_process_artist()

    # Ask for origin
    origin = rs.GetPoint("Pick point as Storage origin.")
    # Ask for X axis and Y Axis
    x_point = rs.GetPoint("Pick point on X direction (Beam Length).")
    y_point = rs.GetPoint("Pick point on Y direction. (Row Direction)")

    # Alignment frame
    alignment_frame = Frame(origin, x_point - origin, y_point - origin)
    beam_storage.frame = alignment_frame

    # Ask User how many columns
    columns = rs.GetInteger("How many columns of beams to stack", 5)
    if columns is None:
        return
    beam_storage.y_count = columns

    # Save beam_storage to process and recompute storage frame
    process.beam_storage = beam_storage
    for beam_id in process.assembly.sequence:
        process.dependency.invalidate(beam_id, process.compute_storeage_frame)
        artist.delete_beam_at_position(beam_id, 'assembly_wcf_storage')

    print('BeamStorage Defined')


def set_pickup_robot_config(process):
    # type: (RobotClampAssemblyProcess) -> None
    """
    Allow user to set robot configuration for pickup pose.

    If GripperAlignedPickupStation, a per-tool setting is possible
    Otherwise, a global pickup configuration is used.
    """

    while True:

        # Ask user which tool
        if isinstance(process.pickup_station, GripperAlignedPickupStation):
            tool_id = ui_ask_user_for_tool_id(process, message="Which tool to set Pickup config? (ESC to finish)", include_clamp=False)
            if tool_id is None:
                return

        # Ask user to pick a Configuration json
        path = rs.OpenFileName("Open a Configuration json, representing the pick up location robot config.", "Configuration File (*.json)|*.json|All Files (*.*)|*.*||")
        if path:
            with open(path, 'r') as f:
                configuration = json.load(f, cls=DataDecoder)  # type: Configuration
                assert isinstance(configuration, Configuration)
                if isinstance(process.pickup_station, GripperAlignedPickupStation):
                    process.pickup_station.robot_config_at_pickup[tool_id] = configuration

                    # Perform FK to set also the tool tip frame
                    configuration_mm = configuration.scaled(1000)
                    robot_flange_frame = process.robot_model.forward_kinematics(configuration_mm, process.ROBOT_END_LINK)
                    toolchanger = process.robot_toolchanger
                    toolchanger.current_frame = robot_flange_frame
                    tool = process.tool(tool_id)
                    tool.current_frame = toolchanger.current_tcf
                    gripper_tcp_frame_at_pickup = tool.current_tcf

                    process.pickup_station.gripper_tcp_frame_at_pickup[tool_id] = gripper_tcp_frame_at_pickup
                    print("Tool TCP frame for pickup set at %s" % gripper_tcp_frame_at_pickup)

                else:
                    process.pickup_station.beam_pickup_configuration = configuration
                    raise RuntimeError("This pickup station is no longer supported.")


                print("Pickup station Beam Pickup Configuration is successfully loaded from %s" % path)
        else:
            print("No Config File opened.")
            return


def modify_tool_specific_pickup_frame(process):
    # type: (RobotClampAssemblyProcess) -> None
    """
    Allow user to set robot configuration for pickup pose.

    If GripperAlignedPickupStation, a per-tool setting is possible
    Otherwise, a global pickup configuration is used.
    """
    # Ask user which tool
    if not isinstance(process.pickup_station, GripperAlignedPickupStation):
        print ("Tool Specific pickup frame is only possible for GripperAlignedPickupStation")
        return

    while True:
        tool_id = ui_ask_user_for_tool_id(process, message="Which tool to set Pickup Frame (Gripper TCP)? (ESC to finish)", include_clamp=False)
        if tool_id is None:
            return

        # Ask for origin
        origin = rs.GetPoint("Pick point as default alignment origin. (Gripper TCP)")
        if origin is None:
            return
        # Ask for X axis and Y Axis
        x_point = rs.GetPoint("Pick point on X direction. (length of beam)")
        if x_point is None:
            return

        y_point = rs.GetPoint("Pick point on Y direction. (side)")
        if y_point is None:
            return

        # Alignment frame
        alignment_frame = Frame(origin, x_point - origin, y_point - origin)
        process.pickup_station.gripper_tcp_frame_at_pickup[tool_id] = alignment_frame

        print("New pickup tcp frame for %s set to %s" % (tool_id, alignment_frame))

def not_implemented(process):
    #
    print('This function is not implemented')


def show_menu(process):
    # type: (RobotClampAssemblyProcess) -> None
    assembly = process.assembly  # type: Assembly
    artist = get_process_artist()

    while (True):
        # Create Menu
        if process.pickup_station is None:
            message = "Material Pickup is undefined"
        elif isinstance(process.pickup_station, GripperAlignedPickupStation):
            message = "Material Pickup is defined from a Gripper-Aligned Pickup Station"
        elif isinstance(process.pickup_station, StackedPickupStation):
            message = "Material Pickup is defined from a Stacked Pickup Station"
        elif isinstance(process.pickup_station, PickupStation):
            message = "Material Pickup is defined from a Single-position Pickup Station"
        else:
            message = "Material Pickup is weird"

        config = {
            'message': message,
            'options': [
                {'name': 'Finish', 'action': 'Exit'
                 },
                {'name': 'DefineGripperAlignedPickupStation', 'action': create_gripper_aligning_pickup_station
                 },
                {'name': 'ModifyGripperSpecificPickupFrame', 'action': modify_tool_specific_pickup_frame
                 },
                {'name': 'DefineStackedPickupStation', 'action': create_stacked_pickup_station
                 },
                {'name': 'DefineBeamStorage', 'action': create_beam_storage
                 },
                {'name': 'SetPickupRobotConfig', 'action': set_pickup_robot_config
                 },
                {'name': 'Import', 'action': import_pickup_station
                 },
                {'name': 'Export', 'action': export_pickup_station
                 },

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
