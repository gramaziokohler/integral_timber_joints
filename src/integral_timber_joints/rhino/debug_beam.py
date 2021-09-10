import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext as sc
import re
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.geometry import JointHalfLap
from integral_timber_joints.rhino.assembly_artist import AssemblyNurbsArtist

import json

from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh
from compas.data import DataEncoder


from compas_rhino.geometry.transformations import xform_from_transformation


if __name__ == '__main__':
    process = get_process()
    artist = AssemblyNurbsArtist(process.assembly)
    beam_id = 'b3'

    beam = process.assembly.beam(beam_id)


    artist.draw_beam_at_position(beam_id, 'assembly_wcf_pickup', delete_old=True)
    artist.draw_beam_at_position(beam_id, 'assembly_wcf_screwdriver_attachment_pose', delete_old=True)
