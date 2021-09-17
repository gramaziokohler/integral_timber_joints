import json
import os
import uuid

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
from integral_timber_joints.geometry import EnvironmentModel, JointHalfLap, JointNonPlanarLap
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
guid = uuid.UUID
# ######################################################
# Functions tp draw, redraw and delete selectable joints
# ######################################################


def draw_selectable_joint(process, joint_id, redraw=True, color=None):
    # type: (RobotClampAssemblyProcess, tuple[str,str], bool, Tuple) -> None
    """Draw joint feature of a specific joint.

    If color is specified, will use that colour.
    Otherwise apply a colour scheme based on which side the joint is on.
    Green for  joint_id[0] < joint_id[1], Red otherwise.

    """
    PURPLE = (64, 31, 62)
    LIGHT_PURPLE = (151, 73, 146)
    BLUE = (87, 115, 128)
    LIGHT_BLUE = (170, 189, 197)
    BROWN = (162, 97, 21)
    LIGHT_BROWN = (239, 187, 129)

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
        joint_is_forward = process.assembly.sequence.index(joint_id[0]) < process.assembly.sequence.index(joint_id[1])
        # Apply Color to the geometry
        if color is not None:
            rs.ObjectColor(guid, color)
        elif isinstance(joint, JointHalfLap):
            rs.ObjectColor(guid, PURPLE) if joint_is_forward else rs.ObjectColor(guid, LIGHT_PURPLE)
        elif isinstance(joint, JointNonPlanarLap):
            rs.ObjectColor(guid, BLUE) if joint_is_forward else rs.ObjectColor(guid, LIGHT_BLUE)
        else:
            rs.ObjectColor(guid, BROWN) if joint_is_forward else rs.ObjectColor(guid, LIGHT_BROWN)
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
    if not hasattr(artist, '_joint_features'):
        return
    rs.EnableRedraw(False)
    for joint_id in artist._joint_features:
        delete_selectable_joint(process, joint_id, redraw=False)
    if redraw:
        rs.EnableRedraw(True)


def _get_guids_of_selectable_joints(process, joint_types=[], forward_joint=True, backward_joint=True):
    # type: (RobotClampAssemblyProcess, list[type], bool, bool) -> Tuple[list[guid], list[guid]]
    artist = get_process_artist()
    sequence = process.assembly.sequence
    selectable_joint_guids = []
    non_selectable_joint_guids = []
    for joint_id in process.assembly.joint_ids():
        joint = process.assembly.joint(joint_id)

        # * Skip based on forward backwards
        if not forward_joint and sequence.index(joint_id[0]) < sequence.index(joint_id[1]):
            non_selectable_joint_guids.extend(artist._joint_features[joint_id])
            continue
        if not backward_joint and sequence.index(joint_id[0]) > sequence.index(joint_id[1]):
            non_selectable_joint_guids.extend(artist._joint_features[joint_id])
            continue

        # * Add based on joint_types
        if joint.__class__ in joint_types:
            selectable_joint_guids.extend(artist._joint_features[joint_id])
        else:
            non_selectable_joint_guids.extend(artist._joint_features[joint_id])

    return (selectable_joint_guids, non_selectable_joint_guids)


def show_selectable_joints_by_types(process, joint_types=[], forward_joint=True, backward_joint=True, redraw=False):
    # type: (RobotClampAssemblyProcess, list[type], bool, bool, bool) -> None
    artist = get_process_artist()
    rs.EnableRedraw(False)

    # * Hide joints that are not the right type:
    selectable_joint_guids, non_selectable_joint_guids = _get_guids_of_selectable_joints(process, joint_types=joint_types, forward_joint=forward_joint, backward_joint=backward_joint)
    [rs.ShowObjects(guids) for guids in selectable_joint_guids]
    [rs.HideObjects(guids) for guids in non_selectable_joint_guids]

    if redraw:
        rs.EnableRedraw(True)


def show_all_selectable_joints(process, redraw=False):
    # type: (RobotClampAssemblyProcess, bool) -> None
    artist = get_process_artist()
    rs.EnableRedraw(False)

    # * Show all joints
    for joint_id in process.assembly.joint_ids():
        guids = artist._joint_features[joint_id]
        rs.ShowObjects(guids)
    if redraw:
        rs.EnableRedraw(True)


def users_select_feature(process, joint_types=None, forward_joint=True, backward_joint=True, prompt='Select joints:'):
    # type: (RobotClampAssemblyProcess, list[type], bool, bool, str) -> list[Tuple[str, str]]
    artist = get_process_artist()

    # * Menu for user
    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt(prompt)
    go.EnablePreSelect(False, True)

    # Set getObjects geometry filter
    selectable_joint_guids, non_selectable_joint_guids = _get_guids_of_selectable_joints(process, joint_types=joint_types, forward_joint=forward_joint, backward_joint=backward_joint)

    def joint_feature_filter(rhino_object, geometry, component_index):
        return rhino_object.Attributes.ObjectId in selectable_joint_guids
    go.SetCustomGeometryFilter(joint_feature_filter)

    result = go.GetMultiple(0, 0)

    if result is None or result == Rhino.Input.GetResult.Cancel:
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


def change_joint_type(process):
    # type: (RobotClampAssemblyProcess) -> None
    print("Function not implemented yet. :(")
    return


def change_joint_half_lap_parameters(process):
    # type: (RobotClampAssemblyProcess) -> None
    artist = get_process_artist()

    while True:
        # * Hide joints that are not the right type:
        show_selectable_joints_by_types(process, joint_types=[JointHalfLap], redraw=True)

        # Ask user to pick joints
        joint_ids = users_select_feature(process, joint_types=[JointHalfLap])
        if joint_ids is None or len(joint_ids) == 0:
            show_all_selectable_joints(process, redraw=True)
            return None
        joint_ids = cull_double_selected_joint_ids(process, joint_ids)

        # Flip selected joint_ids because it is more intuitive to select the positive
        joint_ids = [(i, j) for (j, i) in joint_ids]

        # * Print out current joint parameters
        existing_thickness = set()
        for joint_id in joint_ids:
            joint = process.assembly.joint(joint_id)
            current_thickness = joint.thickness
            existing_thickness.add(current_thickness)
            print("Joint (%s-%s) thickness = %s" % (joint_id[0], joint_id[1], current_thickness))

        # * Ask user for new paramter value
        new_thickness = rs.GetReal("New thickness of the lap joint: (Existing Thickness are: %s" % existing_thickness)
        if new_thickness is None:
            show_all_selectable_joints(process, redraw=True)
            return None

        # * Make changes to selected joints
        affected_beams = set()
        for joint_id in joint_ids:
            # Update this joint and its neighbour
            beam_id1, beam_id2 = joint_id
            joint_id_nbr = (beam_id2, beam_id1)
            joint = process.assembly.joint(joint_id)
            joint_nbr = process.assembly.joint(joint_id_nbr)

            # Skip if there are no change
            current_thickness = joint.thickness
            if new_thickness == current_thickness:
                continue
            difference = new_thickness - current_thickness

            # * Logic to update this joint and its neighbour
            joint.set_parameter('thickness', new_thickness)
            joint_nbr.set_parameter('thickness', joint_nbr.get_parameter('thickness')-difference)
            print("Thickness of joint pair changed to %s and %s." % (joint.thickness, joint_nbr.thickness))

            affected_beams.add(beam_id1)
            affected_beams.add(beam_id2)
            # Redraw new selectable joint feature
            delete_selectable_joint(process, joint_id)
            delete_selectable_joint(process, joint_id_nbr)
            draw_selectable_joint(process, joint_id)
            draw_selectable_joint(process, joint_id_nbr)

        # Redraw affected beams with new joints
        for beam_id in affected_beams:
            artist.redraw_interactive_beam(beam_id, force_update=True, redraw=False)

    # * Show all joints agains on exit of this function
    show_all_selectable_joints(process, redraw=True)


def _change_joint_non_planar_lap_thickness(process, joint_ids):
    # type: (RobotClampAssemblyProcess, list[Tuple[str, str]]) -> Tuple[list[str], list[str, str]]

    """
    Returns

    If user pressed cancel in the data entering process, return None
    """
    # * Print out current joint parameters
    existing_thickness = set()
    for joint_id in joint_ids:
        joint = process.assembly.joint(joint_id)
        current_thickness = joint.thickness
        existing_thickness.add(current_thickness)
        print("Joint (%s-%s) thickness = %s" % (joint_id[0], joint_id[1], current_thickness))

    # * Ask user for new paramter value
    new_thickness = rs.GetReal("New thickness of the lap joint: (Existing Thickness are: %s" % existing_thickness)
    if new_thickness is None:
        show_all_selectable_joints(process, redraw=True)
        return

    # * Make changes to selected joints
    affected_beams = set()
    affected_joints = set()
    for joint_id in joint_ids:
        # Update this joint and its neighbour
        beam_id1, beam_id2 = joint_id
        joint_id_nbr = (beam_id2, beam_id1)
        joint = process.assembly.joint(joint_id)  # type:(JointNonPlanarLap)
        joint_nbr = process.assembly.joint(joint_id_nbr)  # type:(JointNonPlanarLap)

        # Skip if there are no change
        current_thickness = joint.thickness
        if new_thickness == current_thickness:
            continue

        # Warn and Skip if thickness is => beam thickness
        beam = process.assembly.beam(beam_id1)
        beam_depth = beam.get_face_height(joint.beam_move_face_id)
        if new_thickness >= beam_depth:
            print("Warning: Cannot set joint thickness >= beam depth (=%s)" % (beam_depth))
            print("Thickness of joint pair (%s) unchanged (=%s)." % (joint_id, joint_nbr.thickness))
            continue

        # * Logic to update this joint and its neighbour
        joint.set_parameter('thickness', new_thickness)
        joint_nbr.set_parameter('thickness', new_thickness)
        print("Thickness of joint pair (%s) changed to %s." % (joint_id, joint_nbr.thickness))
        affected_beams.add(beam_id1)
        affected_beams.add(beam_id2)
        affected_joints.add(joint_id)
        affected_joints.add(joint_id_nbr)

    return (affected_beams, affected_joints)


def change_joint_non_planar_lap_parameters(process):
    # type: (RobotClampAssemblyProcess) -> None
    artist = get_process_artist()

    while True:
        # * Hide joints that are not the right type:
        show_selectable_joints_by_types(process, joint_types=[JointNonPlanarLap], backward_joint=False, redraw=True)

        # Ask user to pick joints
        joint_ids = users_select_feature(process, joint_types=[JointNonPlanarLap], backward_joint=False)
        if joint_ids is None or len(joint_ids) == 0:
            show_all_selectable_joints(process, redraw=True)
            return
        joint_ids = [(i, j) for (j, i) in joint_ids]

        # Flip selected joint_ids because it is more intuitive to select the positive
        result = _change_joint_non_planar_lap_thickness(process, joint_ids)
        if result is None:
            show_all_selectable_joints(process, redraw=True)
            return
        affected_beams, affected_joints = result

        # * Redraw new selectable joint feature
        for joint_id in affected_joints:
            delete_selectable_joint(process, joint_id, redraw=False)
            draw_selectable_joint(process, joint_id, redraw=False)

        # * Redraw affected beams with new joints
        for beam_id in affected_beams:
            artist.redraw_interactive_beam(beam_id, force_update=True, redraw=False)


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
                {'name': 'ChangeJointType', 'action': change_joint_type
                 },
                {'name': 'LapJointParameters', 'action': change_joint_half_lap_parameters
                 },
                {'name': 'NonPlanarLapJointParameters', 'action': change_joint_non_planar_lap_parameters
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
