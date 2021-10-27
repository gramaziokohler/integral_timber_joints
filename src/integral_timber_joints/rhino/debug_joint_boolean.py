import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext as sc
import re
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.geometry import JointHalfLap, JointNonPlanarLap
from integral_timber_joints.rhino.assembly_artist import AssemblyNurbsArtist
from integral_timber_joints.rhino.artist import vertices_and_faces_to_brep_struct, draw_shapes_as_brep_get_guids
from integral_timber_joints.assembly import BeamAssemblyMethod
from integral_timber_joints.tools import Clamp, Screwdriver, Gripper

import json

from compas.data import DataEncoder
from compas_rhino.utilities import clear_layer, delete_objects, draw_breps, draw_cylinders, draw_mesh


from compas.geometry import Cylinder, Transformation, Polyhedron


if __name__ == '__main__':
    process = get_process()
    assembly = process.assembly
    artist = get_process_artist()
    beam_id = 'b15'
    shapes = assembly.get_beam_negative_shapes(beam_id)

    # draw_shapes_as_brep_get_guids(shapes)

    beam_stay = assembly.beam('b15')
    beam_move = assembly.beam('b21')
    j1, j2, line = JointNonPlanarLap.from_beam_beam_intersection(beam_stay, beam_move, joint_face_id_stay=1, joint_face_id_move=1)
    shapes = j1.get_feature_shapes(None)
    draw_shapes_as_brep_get_guids(shapes)