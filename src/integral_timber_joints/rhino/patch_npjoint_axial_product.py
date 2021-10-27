import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext
import re

from compas.geometry import Line
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.geometry import JointHalfLap, Screw_SL, JointNonPlanarLap
from integral_timber_joints.assembly import BeamAssemblyMethod
from integral_timber_joints.rhino.assembly_artist import AssemblyNurbsArtist

from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh

if __name__ == '__main__':
    process = get_process()
    assembly = process.assembly
    # artist = get_process_artist()
    # artist = AssemblyNurbsArtist(process.assembly)

    for beam_id in assembly.sequence:
        for nbr_id in assembly.get_already_built_neighbors(beam_id):
            joint_id = (nbr_id, beam_id)
            joint = assembly.joint(joint_id)
            if type(joint) == JointNonPlanarLap:
                print ("Fixing Joint: " , joint_id)
                beam_stay = assembly.beam(nbr_id)
                beam_move = assembly.beam(beam_id)
                j1, j2, line = JointNonPlanarLap.from_beam_beam_intersection(beam_stay, beam_move, joint_face_id_stay=joint.beam_stay_face_id, joint_face_id_move=joint.beam_move_face_id)
                new_value = j1.axial_dot_product
                print("axial_dot_product changed from: %f to %f "% (joint.axial_dot_product, new_value))
                assembly.joint((nbr_id, beam_id)).axial_dot_product = new_value
                assembly.joint((beam_id, nbr_id)).axial_dot_product = new_value
