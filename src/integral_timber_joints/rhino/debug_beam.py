import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext as sc
import re
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.geometry import JointHalfLap
from integral_timber_joints.rhino.assembly_artist import AssemblyNurbsArtist
from integral_timber_joints.assembly import BeamAssemblyMethod
from integral_timber_joints.tools import Clamp, Screwdriver, Gripper

import json

from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh
from compas.data import DataEncoder


from compas.geometry import Cylinder, Transformation




if __name__ == '__main__':
    process = get_process()

    artist = get_process_artist()
    beam_id = 'b3'
    artist.draw_beam_brep(beam_id, True)
    # beam = process.assembly.beam(beam_id)

    s1 = process.screwdriver('s1')
    print(s1.gripper_drill_lines)

    g1 = process.gripper('g1')
    print(g1.gripper_drill_lines)

    # artist.draw_beam_at_position(beam_id, 'assembly_wcf_pickup', delete_old=True)
    # artist.draw_beam_at_position(beam_id, 'assembly_wcf_screwdriver_attachment_pose', delete_old=True)
