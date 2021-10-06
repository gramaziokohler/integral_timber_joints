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
from integral_timber_joints.geometry import EnvironmentModel, JointHalfLap, JointNonPlanarLap, JointPolylineLap
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


def draw_selectable_joint(process, joint_id, redraw=False, color=None):
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

    # ! First attempt at boolean all objects together
    success = [brep.MergeCoplanarFaces(sc.doc.ModelAbsoluteTolerance) for brep in breps]
    # print("MergeCoplanarFaces success : %s" % success)
    if len(breps) > 1:
        boolean_result = rg.Brep.CreateBooleanUnion(breps, sc.doc.ModelAbsoluteTolerance)
    else:
        boolean_result = breps
    # print (boolean_result)

    # ! Second attempt at boolean objects iteratively together
    if boolean_result is None:
        print("Warning: joints.py draw_joint_boolean_feature(%s-%s) Group Boolean Union Failure" % joint_id)
        temp_result = [breps[0]]
        for brep in breps[1:]:
            temp_result.append(brep)
            temp_result = rg.Brep.CreateBooleanUnion(temp_result, sc.doc.ModelAbsoluteTolerance)
            if temp_result is None:
                print("Warning: joints.py draw_joint_boolean_feature(%s-%s) Iterative Boolean Union Failure" % joint_id)
                continue
            print("Warning: Still OK")

            temp_result = list(temp_result)
        boolean_result = temp_result

    if boolean_result is None:
        print("ERROR: joints.py draw_joint_boolean_feature(%s-%s) Boolean Union Failure" % joint_id)
        boolean_result = breps
    else:
        delete_objects(guids_for_union, purge=True, redraw=False)

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


def delete_selectable_joint(process, joint_id, redraw=False):
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


def _get_filter_of_selectable_joints(process, joint_types=[], forward_joint=True, backward_joint=True):
    # type: (RobotClampAssemblyProcess, list[type], bool, bool) -> Tuple[list[guid], list[guid]]
    selectable_joint_guids, _ = _get_guids_of_selectable_joints(process, joint_types=joint_types, forward_joint=forward_joint, backward_joint=backward_joint)

    def joint_feature_filter(rhino_object, geometry, component_index):
        return rhino_object.Attributes.ObjectId in selectable_joint_guids
    return joint_feature_filter


def show_selectable_joints_by_id(process, joint_ids=[], redraw=False):
    # type: (RobotClampAssemblyProcess, list[Tuple[str,str]], bool) -> None
    artist = get_process_artist()
    rs.EnableRedraw(False)

    # * Hide joints that are not the right type:
    for joint_id in process.assembly.joint_ids():
        if joint_id in joint_ids:
            rs.ShowObjects(artist._joint_features[joint_id])
        else:
            rs.HideObjects(artist._joint_features[joint_id])

    if redraw:
        rs.EnableRedraw(True)


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


def _joint_id_from_rhino_guids(guids):
    joint_ids = []
    for guid in guids:
        name = get_object_name(guid)  # type: str
        ids = name.split('-')
        joint_ids.append((ids[0], ids[1]))
    return joint_ids


def users_select_feature(process, joint_types=None, forward_joint=True, backward_joint=True, prompt='Select joints:'):
    # type: (RobotClampAssemblyProcess, list[type], bool, bool, str) -> list[Tuple[str, str]]
    """Returns a list of selected joints.
    Multi select is possible as an option but by default can only be belonging to one type.

    If user presses Cancel, return None.
    """
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

    # * First Selection is single select and return
    go.AddOption("MultiSelect")
    result = go.Get()

    # * If user opted for muiltiselect, another selection is prompted
    if result == Rhino.Input.GetResult.Option and go.Option().EnglishName == "MultiSelect":
        go.ClearCommandOptions()
        first_result = go.Get()
        if first_result is None or first_result == Rhino.Input.GetResult.Cancel:
            return None

        # Hide joints of different types from the first selection
        first_result_joint_id = _joint_id_from_rhino_guids([obj.ObjectId for obj in go.Objects()])[0]
        first_result_type = process.assembly.joint(first_result_joint_id).__class__
        show_selectable_joints_by_types(process, [first_result_type], backward_joint=False, redraw=True)

        # Prompt user for further selection, without removing the first selection.
        go.SetCommandPrompt(prompt + " (MultiSelect)")
        go.EnableClearObjectsOnEntry(False)
        go.DeselectAllBeforePostSelect = False
        result = go.GetMultiple(0, 0)

    if result is None or result == Rhino.Input.GetResult.Cancel:
        return None

    if isinstance(result, str):
        if result.startswith("Cancel"):
            return None

    # Retrive joint_ids from object name
    joint_ids = _joint_id_from_rhino_guids([obj.ObjectId for obj in go.Objects()])
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


def change_joint_type(process, joint_ids):
    # type: (RobotClampAssemblyProcess, list[Tuple[str, str]]) -> Tuple[list[str], list[str, str]]
    artist = get_process_artist()
    assembly = process.assembly

    current_joint_type = process.assembly.joint(joint_ids[0]).__class__
    if current_joint_type == JointPolylineLap:
        options = ['JointHalfLap', 'Cancel']
    elif current_joint_type == JointHalfLap:
        options = ['JointPolylineLap', 'Cancel']

    result = rs.GetString("Change to HalfLap or PolylineLap :", strings=options)
    if result == 'Cancel':
        return ([], [])
    if result not in ['JointHalfLap', 'JointPolylineLap']:
        return ([], [])

    sequence = assembly.sequence
    affected_beams = set()
    affected_joints = set()
    for joint_id in joint_ids:
        beam_s_id, beam_m_id = joint_id
        if sequence.index(beam_s_id) < sequence.index(beam_m_id):
            beam_s_id, beam_m_id = beam_m_id, beam_s_id
        beam_stay = assembly.beam(beam_s_id)
        beam_move = assembly.beam(beam_m_id)
        joint_id_s_m = (beam_s_id, beam_m_id)
        joint_id_m_s = (beam_m_id, beam_s_id)

        joint_face_id_move = assembly.joint(joint_id_m_s).face_id
        joint_face_id_stay = assembly.joint(joint_id_s_m).face_id

        if result == 'JointPolylineLap':
            j_s, j_m, screw_line = JointPolylineLap.from_beam_beam_intersection(beam_stay, beam_move, joint_face_id_move=joint_face_id_move)
        if result == 'JointHalfLap':
            j_s, j_m, screw_line = JointHalfLap.from_beam_beam_intersection(beam_stay, beam_move, joint_face_id_move=joint_face_id_move)

        if j_m is not None and j_s is not None:
            print('- Joint (%s-%s) chagned to %s' % (beam_s_id, beam_m_id, j_m.__class__.__name__))
            assembly.add_joint_pair(j_s, j_m, beam_s_id, beam_m_id)
            affected_beams.add(beam_s_id)
            affected_beams.add(beam_m_id)

            for joint_id in [joint_id_s_m, joint_id_m_s]:
                affected_joints.add(joint_id)
                # Redraw new selectable joint feature
                delete_selectable_joint(process, joint_id)
                draw_selectable_joint(process, joint_id)

    for beam_id in affected_beams:
        artist.redraw_interactive_beam(beam_id, force_update=True, redraw=False)
        process.dependency.invalidate(beam_id, process.assign_tool_type_to_joints)

    return (affected_beams, affected_joints)


def _joint_half_lap_change_thickness(process, joint_ids):
    # type: (RobotClampAssemblyProcess, list[Tuple[str, str]]) -> Tuple[list[str], list[str, str]]
    artist = get_process_artist()

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
    affected_joints = set()
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
        affected_joints.add(joint_id)
        affected_joints.add(joint_id_nbr)

        # Redraw new selectable joint feature
        delete_selectable_joint(process, joint_id)
        delete_selectable_joint(process, joint_id_nbr)
        draw_selectable_joint(process, joint_id)
        draw_selectable_joint(process, joint_id_nbr)

    # Redraw affected beams with new joints
    for beam_id in affected_beams:
        artist.redraw_interactive_beam(beam_id, force_update=True, redraw=False)

    return (affected_beams, affected_joints)


def _change_joint_non_planar_lap_thickness(process, joint_ids):
    # type: (RobotClampAssemblyProcess, list[Tuple[str, str]]) -> Tuple[list[str], list[str, str]]
    """
    Returns (affected_beams, affected_joints)

    If user pressed cancel in the data entering process, return None
    """

    # * Flip joint id to the joint on earlier beam.
    for i in range(len(joint_ids)):
        p, q = joint_ids[i]
        if process.assembly.sequence.index(p) > process.assembly.sequence.index(q):
            joint_ids[i] = q, p

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
        return None

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


def _change_joint_non_planar_lap_beam_stay_face_id(process, joint_ids):
    """
    Returns (affected_beams, affected_joints)

    If user pressed cancel in the data entering process, return None
    """
    # * Print out current joint parameters
    existing_face_id = set()
    for joint_id in joint_ids:
        joint_on_move = process.assembly.joint(joint_id)  # type: (JointNonPlanarLap)
        current_face_id = joint_on_move.beam_stay_face_id
        existing_face_id.add(current_face_id)
        print("Joint (%s-%s) beam_stay_face_id = %s" % (joint_id[0], joint_id[1], current_face_id))

    # * Ask user for new paramter value
    new_face_id = rs.GetReal("New face_id on Staying Beam: (Existing face_id are: %s" % existing_face_id)
    if new_face_id is None:
        return None

    # * Make changes to selected joints
    affected_beams = set()
    affected_joints = set()
    for joint_id in joint_ids:
        # Update this joint and its neighbour
        beam_id_move, beam_id_stay = joint_id
        joint_id_nbr = (beam_id_stay, beam_id_move)
        joint_on_move = process.assembly.joint(joint_id)  # type:(JointNonPlanarLap)
        joint_on_stay = process.assembly.joint(joint_id_nbr)  # type:(JointNonPlanarLap)

        # Skip if there are no change
        current_face_id = joint_on_move.beam_stay_face_id
        if new_face_id == current_face_id:
            continue

        # Create a new joint with old parameters
        beam_stay = process.assembly.beam(beam_id_stay)
        beam_move = process.assembly.beam(beam_id_move)
        new_joint_on_stay, new_joint_on_move, _ = JointNonPlanarLap.from_beam_beam_intersection(
            beam_stay,
            beam_move,
            thickness=joint_on_move.thickness,
            joint_face_id_move=joint_on_move.beam_move_face_id,
            joint_face_id_stay=new_face_id,
        )
        process.assembly.add_joint_pair(new_joint_on_stay, new_joint_on_move, beam_id_stay, beam_id_move)
        for key, value in joint_on_stay.get_parameters_dict().items():
            new_joint_on_stay.set_parameter(key, value)
        for key, value in joint_on_move.get_parameters_dict().items():
            new_joint_on_move.set_parameter(key, value)

        # * Logic to update this joint and its neighbour
        print("beam_stay_face_id of joint pair (%s) changed to %s." % (joint_on_move.beam_stay_face_id, new_joint_on_move.beam_stay_face_id))
        affected_beams.add(beam_id_move)
        affected_beams.add(beam_id_stay)
        affected_joints.add(joint_id)
        affected_joints.add(joint_id_nbr)

    [process.dependency.invalidate(beam_id, process.assign_tool_type_to_joints) for beam_id in affected_beams]
    return (affected_beams, affected_joints)


def _change_joint_non_planar_lap_beam_move_face_id(process, joint_ids):
    """
    Returns (affected_beams, affected_joints)

    If user pressed cancel in the data entering process, return None
    """
    # * Print out current joint parameters
    existing_face_id = set()
    for joint_id in joint_ids:
        joint_on_move = process.assembly.joint(joint_id)  # type: (JointNonPlanarLap)
        current_face_id = joint_on_move.beam_move_face_id
        existing_face_id.add(current_face_id)
        print("Joint (%s-%s) beam_move_face_id = %s" % (joint_id[0], joint_id[1], current_face_id))

    # * Ask user for new paramter value
    new_face_id = rs.GetReal("New face_id on Moving Beam: (Existing face_id are: %s" % existing_face_id)
    if new_face_id is None:
        return None

    # * Make changes to selected joints
    affected_beams = set()
    affected_joints = set()
    for joint_id in joint_ids:
        # Update this joint and its neighbour
        beam_id_move, beam_id_stay = joint_id
        joint_id_nbr = (beam_id_stay, beam_id_move)
        joint_on_move = process.assembly.joint(joint_id)  # type:(JointNonPlanarLap)
        joint_on_stay = process.assembly.joint(joint_id_nbr)  # type:(JointNonPlanarLap)

        # Skip if there are no change
        current_face_id = joint_on_move.beam_move_face_id
        if new_face_id == current_face_id:
            continue

        # Create a new joint with old parameters
        beam_stay = process.assembly.beam(beam_id_stay)
        beam_move = process.assembly.beam(beam_id_move)
        new_joint_on_stay, new_joint_on_move, _ = JointNonPlanarLap.from_beam_beam_intersection(
            beam_stay,
            beam_move,
            thickness=joint_on_move.thickness,
            joint_face_id_move=new_face_id,
            joint_face_id_stay=joint_on_move.beam_stay_face_id,
        )
        process.assembly.add_joint_pair(new_joint_on_stay, new_joint_on_move, beam_id_stay, beam_id_move)
        for key, value in joint_on_stay.get_parameters_dict().items():
            new_joint_on_stay.set_parameter(key, value)
        for key, value in joint_on_move.get_parameters_dict().items():
            new_joint_on_move.set_parameter(key, value)

        # * Logic to update this joint and its neighbour
        print("beam_move_face_id of joint pair (%s) changed to %s." % (joint_on_move.beam_stay_face_id, new_joint_on_move.beam_stay_face_id))
        affected_beams.add(beam_id_move)
        affected_beams.add(beam_id_stay)
        affected_joints.add(joint_id)
        affected_joints.add(joint_id_nbr)

    [process.dependency.invalidate(beam_id, process.assign_tool_type_to_joints) for beam_id in affected_beams]
    return (affected_beams, affected_joints)


def change_joint_non_planar_lap_parameters(process):
    # type: (RobotClampAssemblyProcess) -> None
    artist = get_process_artist()
    selected_parameter = "thickness"
    while True:
        # * Hide joints that are not the right type:
        show_selectable_joints_by_types(process, joint_types=[JointNonPlanarLap], backward_joint=False, redraw=True)

        # * Option Menu for user
        para_change_function = {
            "thickness": _change_joint_non_planar_lap_thickness,
            "beam_stay_face_id": _change_joint_non_planar_lap_beam_stay_face_id,
            "beam_move_face_id": _change_joint_non_planar_lap_beam_move_face_id,
        }

        # * Ask user to pick joints
        go = Rhino.Input.Custom.GetObject()
        go.EnablePreSelect(False, True)
        go.SetCommandPrompt("Select NPJoints to change (%s). ENTER when done:" % (selected_parameter))
        [go.AddOption(key) for key in para_change_function.keys()]
        joint_feature_filter = _get_filter_of_selectable_joints(process, joint_types=[JointNonPlanarLap], backward_joint=False)
        go.SetCustomGeometryFilter(joint_feature_filter)
        para_change_result = go.GetMultiple(0, 0)

        # * If user press ESC, exit function.
        if para_change_result is None or para_change_result == Rhino.Input.GetResult.Cancel:
            show_all_selectable_joints(process, redraw=True)
            return None

        # * If user pressed an Option, it changes the selected_parameter
        joint_ids = _joint_id_from_rhino_guids([obj.ObjectId for obj in go.Objects()])
        if para_change_result == Rhino.Input.GetResult.Option:
            selected_parameter = go.Option().EnglishName
            print("joint_ids: ", len(joint_ids))
            # print ("go.Objects():", len(list(go.Objects())))

            # * If user pressed an Option while there are selected joints, activate the _change function.
            if len(joint_ids) > 0:
                pass
                # joint_ids = _joint_id_from_rhino_guids([obj.ObjectId for obj in go.Objects()])
            else:
                # * If user pressed an Option but no objects are selected, restart the selection process
                continue

        elif para_change_result == Rhino.Input.GetResult.Object:
            if len(joint_ids) > 0:  # * If user pressed Enter with selected joints, activate the _change function.
                # joint_ids = _joint_id_from_rhino_guids([obj.ObjectId for obj in go.Objects()])
                pass
            else:  # * If user pressed Enter with no selected joints, exit function
                show_all_selectable_joints(process, redraw=True)
                return None

        # Retrive joint_ids from object name
        # joint_ids = _joint_id_from_rhino_guids([obj.ObjectId for obj in go.Objects()])

        # Flip selected joint_ids because it is more intuitive to select the positive
        if joint_ids is None or len(joint_ids) == 0:
            show_all_selectable_joints(process, redraw=True)
            return
        joint_ids = [(i, j) for (j, i) in joint_ids]

        # * Activate sub function to deal with changing a specific type of joint and parameter
        para_change_result = para_change_function[selected_parameter](process, joint_ids)
        if para_change_result is None:
            show_all_selectable_joints(process, redraw=True)
            return
        affected_beams, affected_joints = para_change_result

        # * Redraw new selectable joint feature and affected beams with new joints
        for joint_id in affected_joints:
            delete_selectable_joint(process, joint_id, redraw=False)
            draw_selectable_joint(process, joint_id, redraw=False)
        for beam_id in affected_beams:
            artist.redraw_interactive_beam(beam_id, force_update=True, redraw=False)


def _joint_polyine_lap_change_thickness(process, joint_ids):
    # type: (RobotClampAssemblyProcess, list[Tuple[str, str]]) -> Tuple[list[str], list[str, str]]
    """
    Returns (affected_beams, affected_joints)

    If user pressed cancel in the data entering process, return None
    """
    # * Print out current joint parameters
    existing_thickness = set()
    for joint_id in joint_ids:
        joint = process.assembly.joint(joint_id)
        current_thickness = joint.get_parameter('thickness')
        existing_thickness.add(current_thickness)
        print("Joint (%s-%s) thickness = %s" % (joint_id[0], joint_id[1], current_thickness))

    # * Ask user for new paramter value
    new_thickness = rs.GetReal("New thickness of the lap joint: (Existing Thickness are: %s" % existing_thickness)
    if new_thickness is None:
        return None

    # * Make changes to selected joints
    affected_beams = set()
    affected_joints = set()
    for beam_id1, beam_id2 in joint_ids:
        # Update this joint and its neighbour
        joint_id = (beam_id2, beam_id1)
        joint_id_nbr = (beam_id1, beam_id2)
        joint = process.assembly.joint(joint_id)  # type:(JointNonPlanarLap)
        joint_nbr = process.assembly.joint(joint_id_nbr)  # type:(JointNonPlanarLap)

        # Skip if there are no change
        current_thickness = joint.get_parameter('thickness')
        if new_thickness == current_thickness:
            continue

        # Warn and Skip if thickness is => beam thickness
        beam = process.assembly.beam(beam_id1)
        beam_depth = beam.get_face_height(joint.face_id)
        if new_thickness >= beam_depth:
            print("Warning: Cannot set joint thickness >= beam depth (=%s)" % (beam_depth))
            print("Thickness of joint pair (%s) unchanged (=%s)." % (joint_id, joint_nbr.thickness))
            continue

        # * Logic to update this joint and its neighbour
        diff = new_thickness - joint.get_parameter('thickness')
        joint.set_parameter('thickness', new_thickness)
        joint_nbr.set_parameter('thickness', joint_nbr.get_parameter('thickness') - diff)
        print("Thickness of joint pair (%s) changed to %s." % (joint_id, joint_nbr.thickness))
        affected_beams.add(beam_id1)
        affected_beams.add(beam_id2)
        affected_joints.add(joint_id)
        affected_joints.add(joint_id_nbr)

    return (affected_beams, affected_joints)


def _joint_polyline_lap_change_polyline_string(process, joint_ids):
    # type: (RobotClampAssemblyProcess, list[Tuple[str, str]]) -> Tuple[list[str], list[str, str]]
    """
    Returns (affected_beams, affected_joints)

    If user pressed cancel in the data entering process, return None
    """
    # * Print out current joint parameters
    # existing_thickness = set()
    for joint_id in joint_ids:
        joint = process.assembly.joint(joint_id)
        print("Joint (%s-%s) string = %s" % (joint_id[0], joint_id[1], joint.param_string))

    # * Ask user for new paramter value
    new_string = rs.StringBox("Enter new string for the joints.")
    if new_string is None:
        return None

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
        current_string = joint.param_string
        if new_string == current_string:
            continue

        # Warn and Skip if thickness is => beam thickness
        try:
            # * Logic to update this joint and its neighbour
            joint.set_parameter('param_string', new_string)
            joint_nbr.set_parameter('param_string', new_string)
            # Check if successful
            assert isinstance(joint.thickness, float)
            assert len(joint.polylines) == 4
            print("String of Joint (%s-%s) changed." % (joint_id))
            affected_beams.add(beam_id1)
            affected_beams.add(beam_id2)
            affected_joints.add(joint_id)
            affected_joints.add(joint_id_nbr)
        except:
            joint.set_parameter('param_string', current_string)
            joint_nbr.set_parameter('param_string', current_string)
            print("Error changing string for Joint (%s-%s), change reverted." % (joint_id))

    return (affected_beams, affected_joints)


def _joint_polyline_lap_rotate(process, joint_ids, rotate_cw=True):
    # type: (RobotClampAssemblyProcess, list[Tuple[str, str]], bool) -> Tuple[list[str], list[str, str]]
    """
    Returns (affected_beams, affected_joints)

    If user pressed cancel in the data entering process, return None
    """

    # * Make changes to selected joints
    affected_beams = set()
    affected_joints = set()
    for joint_id in joint_ids:
        # Update this joint and its neighbour
        beam_id1, beam_id2 = joint_id
        joint_id_nbr = (beam_id2, beam_id1)
        joint = process.assembly.joint(joint_id)  # type:(JointNonPlanarLap)
        joint_nbr = process.assembly.joint(joint_id_nbr)  # type:(JointNonPlanarLap)

        # Rotate Polyline
        current_polylines = joint.polylines
        if rotate_cw:
            new_polylines = current_polylines[-1:] + current_polylines[:-1]
        else:
            new_polylines = current_polylines[1:] + current_polylines[:1]

        # Skip if there are no change
        if new_polylines == current_polylines:
            continue

        try:
            # * Logic to update this joint and its neighbour
            joint.polylines = new_polylines
            joint_nbr.polylines = new_polylines
            # Check if successful
            assert len(joint.polylines) == 4
            print("Joint (%s-%s) rotated." % (joint_id))
            affected_beams.add(beam_id1)
            affected_beams.add(beam_id2)
            affected_joints.add(joint_id)
            affected_joints.add(joint_id_nbr)
        except:
            joint.polylines = current_polylines
            joint_nbr.polylines = current_polylines
            print("Error changing Joint (%s-%s), change reverted." % (joint_id))

    return (affected_beams, affected_joints)


def _joint_polyline_lap_rotate_cw(process, joint_ids):
    # type: (RobotClampAssemblyProcess, list[Tuple[str, str]]) -> Tuple[list[str], list[str, str]]
    return _joint_polyline_lap_rotate(process, joint_ids, rotate_cw=True)


def _joint_polyline_lap_rotate_ccw(process, joint_ids):
    # type: (RobotClampAssemblyProcess, list[Tuple[str, str]]) -> Tuple[list[str], list[str, str]]
    return _joint_polyline_lap_rotate(process, joint_ids, rotate_cw=False)


def _joint_polyline_lap_mirror(process, joint_ids, direction_u=True):
    # type: (RobotClampAssemblyProcess, list[Tuple[str, str]], bool) -> Tuple[list[str], list[str, str]]
    """
    Returns (affected_beams, affected_joints)

    If user pressed cancel in the data entering process, return None
    """

    # * Make changes to selected joints
    affected_beams = set()
    affected_joints = set()
    for joint_id in joint_ids:
        # Update this joint and its neighbour
        beam_id1, beam_id2 = joint_id
        joint_id_nbr = (beam_id2, beam_id1)
        joint = process.assembly.joint(joint_id)  # type:(JointNonPlanarLap)
        joint_nbr = process.assembly.joint(joint_id_nbr)  # type:(JointNonPlanarLap)

        # Mirror polyline
        current_polylines = joint.polylines
        if direction_u:
            new_polylines = [current_polylines[i] for i in [2, 1, 0, 3]]
        else:
            new_polylines = [current_polylines[i] for i in [0, 3, 2, 1]]
        # Reverse order of each line
        for i in range(4):
            new_polylines[i] = [[1-u, v] for u, v in new_polylines[i]][::-1]

        # Skip if there are no change
        if new_polylines == current_polylines:
            continue

        try:
            # * Logic to update this joint and its neighbour
            joint.polylines = new_polylines
            joint_nbr.polylines = new_polylines
            # Check if successful
            assert len(joint.polylines) == 4
            print("Joint (%s-%s) rotated." % (joint_id))
            affected_beams.add(beam_id1)
            affected_beams.add(beam_id2)
            affected_joints.add(joint_id)
            affected_joints.add(joint_id_nbr)
        except:
            joint.polylines = current_polylines
            joint_nbr.polylines = current_polylines
            print("Error changing Joint (%s-%s), change reverted." % (joint_id))

    return (affected_beams, affected_joints)


def _joint_polyline_lap_mirror_u(process, joint_ids):
    # type: (RobotClampAssemblyProcess, list[Tuple[str, str]]) -> Tuple[list[str], list[str, str]]
    return _joint_polyline_lap_mirror(process, joint_ids, direction_u=True)


def _joint_polyline_lap_mirror_v(process, joint_ids):
    # type: (RobotClampAssemblyProcess, list[Tuple[str, str]]) -> Tuple[list[str], list[str, str]]
    return _joint_polyline_lap_mirror(process, joint_ids, direction_u=False)


# ############################
# Main menu / Top level menu
# ############################

def select_joint_parameter_to_change(process, joint_ids):
    # type: (RobotClampAssemblyProcess, list[Tuple[str, str]]) -> None
    artist = get_process_artist()
    assert len(set(process.assembly.joint(joint_id).__class__.__name__ for joint_id in joint_ids)) == 1

    all_options = {
        "JointHalfLap": {
            "Thickness": _joint_half_lap_change_thickness,
            "ChangeType": change_joint_type
        },
        "JointNonPlanarLap":
        {
            "Thickness": _change_joint_non_planar_lap_thickness,
            "MoveFaceID": _change_joint_non_planar_lap_beam_move_face_id,
            "StayFaceID": _change_joint_non_planar_lap_beam_stay_face_id,
            "ChangeType": change_joint_type
        },
        "JointPolylineLap":
        {
            "Polyline": _joint_polyline_lap_change_polyline_string,
            "MirrorU": _joint_polyline_lap_mirror_u,
            "MirrorV": _joint_polyline_lap_mirror_v,
            "RotateCW": _joint_polyline_lap_rotate_cw,
            "RotateCCW": _joint_polyline_lap_rotate_ccw,
            "Thickness": _joint_polyine_lap_change_thickness,
            "ChangeType": change_joint_type
        },
    }

    while True:
        joint_type = process.assembly.joint(joint_ids[0]).__class__
        options = list(all_options[joint_type.__name__].keys())
        print("Selected %i joints: %s" % (len(joint_ids), str(joint_ids)))
        result = rs.GetString("Options for %s(s) (ESC to return):" % (joint_type.__name__), strings=options)

        if result is None:
            return None
        if result == "Cancel":
            return None

        if result in all_options[joint_type.__name__]:
            function = all_options[joint_type.__name__][result]
            affected_beams, affected_joints = function(process, joint_ids)

            # * Redraw new selectable joint feature and affected beams with new joints
            for joint_id in affected_joints:
                delete_selectable_joint(process, joint_id, redraw=False)
                draw_selectable_joint(process, joint_id, redraw=False)

            for beam_id in affected_beams:
                artist.redraw_interactive_beam(beam_id, force_update=True, redraw=False)

            show_selectable_joints_by_id(process, joint_ids, True)
            print("Function Complete, Anything else?")


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

    while (True):

        # Create Menu
        # result = CommandMenu(construct_menu()).select_action()
        show_all_selectable_joints(process, redraw=True)
        selected_joint_ids = users_select_feature(process, [JointPolylineLap, JointHalfLap, JointNonPlanarLap], backward_joint=False, prompt="Select a Joint to Edit")

        # User cancel command by Escape
        if selected_joint_ids is None:
            return on_exit_ui()

        if len(selected_joint_ids) == 0:
            return on_exit_ui()
        show_selectable_joints_by_id(process, selected_joint_ids, redraw=True)
        select_joint_parameter_to_change(process, selected_joint_ids)


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
