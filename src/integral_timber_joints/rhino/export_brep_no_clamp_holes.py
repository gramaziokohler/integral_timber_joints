
import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext as sc
import Rhino.Geometry as rg

from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.geometry import JointHalfLap, JointNonPlanarLap
from integral_timber_joints.rhino.assembly_artist import AssemblyNurbsArtist
from integral_timber_joints.assembly import BeamAssemblyMethod
from integral_timber_joints.tools import Clamp, Screwdriver, Gripper

from compas_rhino.utilities import clear_layer, delete_objects, draw_breps, draw_cylinders, draw_mesh
from integral_timber_joints.rhino.artist import mesh_to_brep, vertices_and_faces_to_brep_struct, draw_shapes_as_brep_get_guids

import json

from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh
from compas.data import DataEncoder


from compas.geometry import Cylinder, Transformation

add_brep = sc.doc.Objects.AddBrep
find_object = sc.doc.Objects.Find

TOL = sc.doc.ModelAbsoluteTolerance

def get_tool_features_on_beam(self, beam_id, include_screwdriver = True, include_clamp = True, include_gripper = True):
    assembly = self.assembly
    # * Retrieve Clamps and Screwdrivers attached to this beam
    other_feature_shapes = []

    assembly_method = self.assembly.get_assembly_method(beam_id)

    def draw_drill_cylinders_of_tool_at_wcf(tool):
        cylinders = []
        t_world_tool_at_final = Transformation.from_frame(tool.current_frame)
        for line in tool.gripper_drill_lines:
            cylinder = Cylinder.from_line_radius(line, tool.gripper_drill_diameter/2.0)
            cylinders.append(cylinder.transformed(t_world_tool_at_final))
        return cylinders

    # * Gripper (except if it is manually assembled)
    if assembly_method != BeamAssemblyMethod.MANUAL_ASSEMBLY:
        if self.assembly.get_beam_attribute(beam_id, "gripper_tcp_in_ocf") is None:
            print("Warning: gripper_tcp_in_ocf is None while calling process.get_tool_features_on_beam(%s)" % (beam_id))
        else:
            gripper = self.get_gripper_of_beam(beam_id)
            print (type(gripper))
            if include_gripper and type(gripper) == Gripper:
                other_feature_shapes += draw_drill_cylinders_of_tool_at_wcf(gripper)
            if include_screwdriver and type(gripper) == Screwdriver:
                other_feature_shapes += draw_drill_cylinders_of_tool_at_wcf(gripper)

    # * Clamps Attached to this beam - Need to check neighbour's assembly method
    for neighbour_id in assembly.get_unbuilt_neighbors(beam_id):
        if assembly.get_assembly_method(neighbour_id) == BeamAssemblyMethod.CLAMPED:
            clamp = self.get_tool_of_joint((beam_id, neighbour_id))  # type: Clamp
            if clamp is not None:
                if include_clamp:
                    other_feature_shapes += draw_drill_cylinders_of_tool_at_wcf(clamp)

    # * Screwdrivers Attached to this beam
    if assembly_method in BeamAssemblyMethod.screw_methods:
        for neighbour_id in assembly.get_already_built_neighbors(beam_id):
            screwdriver = self.get_tool_of_joint((neighbour_id, beam_id))
            if screwdriver is not None:
                if include_screwdriver:
                    other_feature_shapes += draw_drill_cylinders_of_tool_at_wcf(screwdriver)
    return other_feature_shapes

def draw_brep_without_screwdriver_holes(beam_id):

    # Obtain tool features on Beam from Process
    other_feature_shapes = get_tool_features_on_beam(process, beam_id, include_screwdriver=False)

    rs.EnableRedraw(False)


    # Positive geometry is the uncut beam mesh
    mesh_box = assembly.beam(beam_id).draw_uncut_mesh()
    positive_brep_guids = draw_breps(mesh_to_brep(mesh_box), join=True, redraw=False)

    # Retrieve all the negative features
    negative_shapes = assembly.get_beam_negative_shapes(beam_id)
    negative_shapes += other_feature_shapes

    guids = []  # Hold the guids of the final boolean result
    if len(negative_shapes) > 0:
        # Get the negative meshes from the features and convert them to nurbs
        negative_brep_guids = draw_shapes_as_brep_get_guids(negative_shapes)

        # Perform Boolean Difference
        positive_breps = [rs.coercebrep(guid) for guid in positive_brep_guids]
        negative_breps = [rs.coercebrep(guid) for guid in negative_brep_guids]

        # Perform MergeCoplanarFaces before boolean to reduce chance of failure
        [brep.MergeCoplanarFaces(sc.doc.ModelAbsoluteTolerance) for brep in negative_breps]

        # print("negative_breps : %s" % negative_breps)
        # ! First attempt at boolean all objects together
        boolean_result = rg.Brep.CreateBooleanDifference(positive_breps, negative_breps, TOL)

        # ! Second attempt at boolean objects one by one
        if boolean_result is None:
            print("WARNING: AssemblyNurbArtist draw_beam(%s) Group Boolean Failure" % beam_id)
            pos = positive_breps
            for neg in negative_breps:
                pos = rg.Brep.CreateBooleanDifference(pos, [neg], TOL)
                if pos is None:
                    print("WARNING: AssemblyNurbArtist draw_beam(%s) Iterative Boolean Failure" % beam_id)
                    break
            boolean_result = pos

        if boolean_result is None:
            print("ERROR: AssemblyNurbArtist draw_beam(%s) Boolean All Failure" % beam_id)
            # delete_objects(positive_brep_guids + negative_brep_guids, purge=True, redraw=False)
            # delete_objects(negative_brep_guids, purge=True, redraw=False)
            # [sc.doc.Objects.AddBrep(brep) for brep in negative_breps]
        else:
            for brep in boolean_result:
                # Perform MergeCoplanarFaces after boolean to clean up
                brep.MergeCoplanarFaces(sc.doc.ModelAbsoluteTolerance)
                # New guids from boolean results
                guid = add_brep(brep)
                if guid:
                    guids.append(guid)

            # Delete the original boolean set geometries
            delete_objects(positive_brep_guids + negative_brep_guids, purge=True, redraw=False)
    else:
        guids = positive_brep_guids

    # Rename newly created object with beam_id
    for guid in guids:
        obj = find_object(guid)
        attr = obj.Attributes
        attr.Name = beam_id
        obj.CommitChanges()

process = get_process()
assembly = process.assembly
artist = get_process_artist()
# beam_id = assembly.sequence[19]

# Layer
layer = 'itj_export'
rs.CurrentLayer(layer)
clear_layer(layer)
for beam_id in assembly.sequence:
    print (beam_id)
    draw_brep_without_screwdriver_holes(beam_id)

# Enable redraw
rs.EnableRedraw(True)


