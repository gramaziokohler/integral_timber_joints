import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext
import re
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.geometry import Joint_halflap
from integral_timber_joints.rhino.assembly_artist import AssemblyNurbsArtist

from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh

if __name__ == '__main__':
    process = get_process()
    # artist = get_process_artist()
    artist = AssemblyNurbsArtist(process.assembly)
    beam_id = 'b11'
    beam = process.assembly.beam(beam_id)
    # Good one
    # # print (process.assembly.joint(('b8', 'b11')).data)
    # print (process.assembly.joint(('b11', 'b8')).data)

    # # print (process.assembly.joint(('b10', 'b11')).data)
    # print (process.assembly.joint(('b11', 'b10')).data)

    artist.draw_beam(beam_id, delete_old=True, verbose=True)


    for joint in process.assembly.get_joints_of_beam(beam_id):
        if isinstance(joint, Joint_halflap):
            print (joint.data)
            pass
        else:
            # print (joint.data)
            print (joint.thickness)
    #         mesh = joint.get_feature_meshes(beam)[0]
    #         v, f = mesh.to_vertices_and_faces()
    #         # Redraw for individual call supressed here.
    #         guid = draw_mesh(v, f)