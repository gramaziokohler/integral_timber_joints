import json
import os

import Rhino  # type: ignore
import rhinoscriptsyntax as rs
from compas.geometry.primitives.frame import Frame
from compas.geometry.primitives.vector import Vector
from compas.utilities import DataDecoder
from compas_rhino.geometry import RhinoMesh, RhinoPoint
from compas_rhino.ui import CommandMenu
from compas_rhino.utilities.objects import get_object_name

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions
from integral_timber_joints.tools import Clamp, Gripper, PickupStation, StackedPickupStation
from integral_timber_joints.tools.beam_storage import BeamStorage


def create_pickup_station(process):
    # type: (RobotClampAssemblyProcess) -> None
    station = PickupStation()

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
        process.compute_storeage_frame(beam_id)
        artist.delete_beam_at_position(beam_id, 'assembly_wcf_storage')

    print('BeamStorage Defined')


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
                {'name': 'DefinePickupStation', 'action': create_pickup_station
                 },
                {'name': 'DefineStackedPickupStation', 'action': create_stacked_pickup_station
                 },
                {'name': 'DefineBeamStorage', 'action': create_beam_storage
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
