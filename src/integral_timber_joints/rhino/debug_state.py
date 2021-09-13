import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext
import re
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.geometry import JointHalfLap
from integral_timber_joints.rhino.assembly_artist import AssemblyNurbsArtist
import json

from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh
from compas.data import DataEncoder

if __name__ == '__main__':
    process = get_process()
    # artist = get_process_artist()
    artist = AssemblyNurbsArtist(process.assembly)
    beam_id = 'b3'
    beam = process.assembly.beam(beam_id)

    scene = process.get_movement_start_scene(process.movements[0])
    print (scene[beam_id,'f'])
    print (scene['b1','f'])