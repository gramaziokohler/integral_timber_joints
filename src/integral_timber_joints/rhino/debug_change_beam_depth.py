import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext as sc
import re
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.geometry import JointHalfLap, JointNonPlanarLap
from integral_timber_joints.rhino.assembly_artist import AssemblyNurbsArtist
from integral_timber_joints.assembly import BeamAssemblyMethod
from integral_timber_joints.tools import Clamp, Screwdriver, Gripper

import json

from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh
from compas.data import DataEncoder


from compas.geometry import Cylinder, Transformation




if __name__ == '__main__':
    process = get_process()
    assembly = process.assembly
    artist = get_process_artist()
    beam_id = 'b15'

    # Adjust Beam Dimension
    # -----------------------------

    # beam = assembly.beam(beam_id)
    # print (beam.height)
    # beam.height = 140

    # artist.delete_interactive_beam_visualization(beam_id)
    # artist.draw_beam_brep(beam_id)

    # Deal with neighbouring joint
    # -----------------------------

    # nbr_beam_id = 'b9'

    # joint = assembly.joint((nbr_beam_id, beam_id))
    # joint.length = 140
    # print (joint.data)

    # artist.delete_interactive_beam_visualization(nbr_beam_id)
    # artist.draw_beam_brep(nbr_beam_id)


    # Recreate Planar Joints
    # ----------------------

    affected_beams = set()
    for joint_id in assembly.get_joint_ids_of_beam(beam_id):
        joint = assembly.joint(joint_id)
        if joint.__class__.__name__ == 'JointHalfLap':
            print (joint_id, joint.__class__.__name__)

            x, y = joint_id
            beam_stay_id = assembly.sequence[min(assembly.sequence.index(x), assembly.sequence.index(y))]
            beam_move_id = assembly.sequence[max(assembly.sequence.index(x), assembly.sequence.index(y))]
            beam_stay = assembly.beam(beam_stay_id)
            beam_move = assembly.beam(beam_move_id)

            j_s, j_m, screw_line = JointHalfLap.from_beam_beam_intersection(beam_stay, beam_move)
            assembly.add_joint_pair(j_s, j_m, beam_stay.name, beam_move.name)
            affected_beams.add(beam_stay_id)
            affected_beams.add(beam_move_id)

    for beam_id in affected_beams:
        artist.delete_interactive_beam_visualization(beam_id)
        artist.draw_beam_brep(beam_id)