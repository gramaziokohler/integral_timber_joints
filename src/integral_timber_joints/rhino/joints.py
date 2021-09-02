import json
import os

import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import Rhino.Geometry as rg

import scriptcontext as sc  # type: ignore
from compas.geometry import Cylinder, Polyhedron
from compas.utilities import DataDecoder
from compas_rhino.geometry import RhinoMesh
from compas_rhino.ui import CommandMenu
from compas_rhino.utilities import clear_layer, delete_objects, draw_breps, draw_cylinders, draw_mesh
from compas_rhino.utilities.objects import get_object_name, get_object_names

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.geometry import EnvironmentModel, Joint_halflap, JointNonPlanarLap
from integral_timber_joints.process import Movement, RobotClampAssemblyProcess, RoboticMovement
from integral_timber_joints.rhino.artist import mesh_to_brep, vertices_and_faces_to_brep_struct
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.utility import get_existing_beams_filter, recompute_dependent_solutions
from integral_timber_joints.tools import Clamp, Gripper, RobotWrist, ToolChanger

try:
    from typing import Dict, List, Optional, Tuple, cast
except:
    pass
add_brep = sc.doc.Objects.AddBrep
find_object = sc.doc.Objects.Find

# ######################################################
# Functions tp draw, redraw and delete selectable joints
# ######################################################


def draw_selectable_joint(process, joint_id, redraw=True):
    # type: (RobotClampAssemblyProcess, tuple[str,str],  bool) -> None

    artist = get_process_artist()
    if not hasattr(artist, '_joint_features'):
        artist._joint_features = {}

    # Collect all the feature shapes from the joint and Boolean Union into one object
    rs.EnableRedraw(False)
    joint = process.assembly.joint(joint_id)
    beam = process.assembly.beam(joint_id[0])
    shapes = joint.get_feature_shapes(beam)
    guids_for_union = []
    for shape in shapes:
        if isinstance(shape, Polyhedron):
            vertices_and_faces = shape.to_vertices_and_faces()
            struct = vertices_and_faces_to_brep_struct(vertices_and_faces)
            # print("Polyhedron :", struct)
            guids_for_union.extend(draw_breps(struct, join=True, redraw=False))
        elif isinstance(shape, Cylinder):
            cylinder = shape
            start = cylinder.center + cylinder.normal.scaled(cylinder.height / 2)
            end = cylinder.center - cylinder.normal.scaled(cylinder.height / 2)
            struct = {'start': list(start), 'end': list(end), 'radius': cylinder.circle.radius}
            # print("Cylinder : ", struct)
            guids_for_union.extend(draw_cylinders([struct], cap=True, redraw=False))
    breps = [rs.coercebrep(guid) for guid in guids_for_union]
    boolean_result = rg.Brep.CreateBooleanUnion(breps, sc.doc.ModelAbsoluteTolerance)
    delete_objects(guids_for_union, purge=True, redraw=False)

    if boolean_result is None:
        print("ERROR: joints.py draw_joint_boolean_feature(%s) Boolean Failure" % joint_id)

    # Add boolean result into Rhino Doc and save their guids
    artist._joint_features[joint_id] = []
    for brep in boolean_result:
        # New guids from boolean results
        guid = add_brep(brep)
        if not guid:
            continue
        # Naming
        rs.ObjectName(guid, "%s-%s" % (joint_id[0], joint_id[1]))
        # Red Green Coloring
        if process.assembly.sequence.index(joint_id[0]) < process.assembly.sequence.index(joint_id[1]):
            rs.ObjectColor(guid, (30, 190, 85))
        else:
            rs.ObjectColor(guid, (250, 80, 60))
        # Add to guid dict
        artist._joint_features[joint_id].append(guid)

    if redraw:
        rs.EnableRedraw(True)


def draw_all_selectable_joints(process, redraw=True):
    for joint_id in process.assembly.joint_ids():
        draw_selectable_joint(process, joint_id, redraw=False)
    if redraw:
        rs.EnableRedraw(True)


def delete_selectable_joint(process, joint_id, redraw=True):
    # type: (RobotClampAssemblyProcess, tuple[str,str], bool) -> None
    artist = get_process_artist()
    rs.EnableRedraw(False)
    if joint_id in artist._joint_features:
        delete_objects(artist._joint_features[joint_id], purge=True, redraw=False)
    artist._joint_features[joint_id] = []
    if redraw:
        rs.EnableRedraw(True)


def delete_all_selectable_joints(process, redraw=True):
    # type: (RobotClampAssemblyProcess, bool) -> None
    artist = get_process_artist()
    rs.EnableRedraw(False)
    for joint_id in artist._joint_features:
        delete_selectable_joint(process, joint_id, redraw=False)
    if redraw:
        rs.EnableRedraw(True)


def users_select_feature(process, joint_types=None):
    # type: (RobotClampAssemblyProcess, list[type]) -> list[Tuple[str, str]]
    # Menu for user
    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt("Select joints:")
    go.EnablePreSelect(False, True)

    # Create a list of selectable guids
    artist = get_process_artist()
    beam_guids = []
    for joint_id in process.assembly.joint_ids():
        joint = process.assembly.joint(joint_id)
        if joint_types is not None:
            if not any([isinstance(joint, joint_type) for joint_type in joint_types]):
                continue
        # Add the guids that have been drawn to the list
        beam_guids.extend(artist._joint_features[joint_id])

    # Set getObjects geometry filter
    def joint_feature_filter(rhino_object, geometry, component_index):
        return rhino_object.Attributes.ObjectId in beam_guids
    go.SetCustomGeometryFilter(joint_feature_filter)

    result = go.GetMultiple(0, 0)
    if result is None:
        return None

    # Retrive joint_ids from object name
    joint_ids = []
    for object in go.Objects():
        guid = object.ObjectId
        name = get_object_name(guid)  # type: str
        ids = name.split('-')
        joint_ids.append((ids[0], ids[1]))
    return joint_ids


def cull_double_selected_joint_ids(process, joint_ids):
    # type: (RobotClampAssemblyProcess, list[Tuple[str, str]]) -> list[Tuple[str, str]]
    filtered_joint_ids = []
    sequence = process.assembly.sequence
    # Add joint_id that are earlier
    for joint_id in joint_ids:
        beam_id_0, beam_id_1 = joint_id
        if sequence.index(beam_id_0) < sequence.index(beam_id_1):
            filtered_joint_ids.append(joint_id)
    # Add joint_id that are later, only if the earlier id is not in the list
    for joint_id in joint_ids:
        beam_id_0, beam_id_1 = joint_id
        if sequence.index(beam_id_0) > sequence.index(beam_id_1):
            if (beam_id_1, beam_id_0) not in filtered_joint_ids:
                filtered_joint_ids.append(joint_id)
    return filtered_joint_ids

# ##############################
# Functions to operate on Joints
# ##############################


def change_lap_joint_parameters(process):
    # type: (RobotClampAssemblyProcess) -> None
    artist = get_process_artist()

    # Ask user to pick two joints
    joint_ids = users_select_feature(process, joint_types=[Joint_halflap])
    joint_ids = cull_double_selected_joint_ids(process, joint_ids)
    if len(joint_ids) == 0:
        return

    # Print out current joint parameters
    existing_heights = set()
    for joint_id in joint_ids:
        joint = process.assembly.joint(joint_id)
        current_height = joint.height
        existing_heights.add(current_height)
        print("Joint (%s-%s) height = %s" % (joint_id[0], joint_id[1], current_height))

    # Ask user for new paramter value
    new_height = rs.GetReal("New thickness of the lap joint: (Existing Heights are: %s" % existing_heights)
    if new_height is None:
        return

    # Make changes to all the selected joints
    affected_beams = set()
    for joint_id in joint_ids:
        beam_id1, beam_id2 = joint_id
        joint = process.assembly.joint(joint_id)
        current_height = joint.height
        if new_height == current_height:
            continue
        difference = new_height - current_height
        process.assembly.joint((beam_id1, beam_id2)).height += difference
        process.assembly.joint((beam_id2, beam_id1)).height -= difference
        print("Height of joint pair changed to %s and %s." % (
            process.assembly.joint((beam_id1, beam_id2)).height,
            process.assembly.joint((beam_id2, beam_id1)).height
        ))
        affected_beams.add(beam_id1)
        affected_beams.add(beam_id2)
        # Redraw new selectable joint feature
        delete_selectable_joint(process, (beam_id1, beam_id2))
        delete_selectable_joint(process, (beam_id2, beam_id1))
        draw_selectable_joint(process, (beam_id1, beam_id2))
        draw_selectable_joint(process, (beam_id2, beam_id1))

    # Redraw beams with new joints
    for beam_id in affected_beams:
        artist.redraw_interactive_beam(beam_id, force_update=True, redraw=False)

    # Redraw new selectable joint feature
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
    draw_all_selectable_joints(process, redraw=False)
    rs.EnableRedraw(True)
    sc.doc.Views.Redraw()

    # On exist function
    def on_exit_ui():
        delete_all_selectable_joints(process)
        print('Exit Function')
        return Rhino.Commands.Result.Cancel

    command_to_run = None

    def construct_menu():
        # Menu for user
        menu = {
            'message': "Choose Options",
            'options': [
                {'name': 'Finish', 'action': 'Exit'
                 },
                {'name': 'LapJoint', 'action': change_lap_joint_parameters
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
