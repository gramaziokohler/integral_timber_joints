import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext
import re

from compas.geometry import Line
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.geometry import JointHalfLap, Screw_SL
from integral_timber_joints.assembly import BeamAssemblyMethod
from integral_timber_joints.rhino.assembly_artist import AssemblyNurbsArtist

from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh

if __name__ == '__main__':
    process = get_process()
    assembly = process.assembly
    # artist = get_process_artist()
    # artist = AssemblyNurbsArtist(process.assembly)
    beam_id = 'b25'
    beam = process.assembly.beam(beam_id)

    joint_id = ('b25','b15')
    joint = assembly.joint(joint_id)
    print (joint_id)
    # print (joint.angle)
    print (joint.data)
    print (joint.assembly_tool_types(BeamAssemblyMethod.CLAMPED))
    print (joint.get_clamp_frames(process.assembly.beam(joint_id[0])))

    joint_id = ('b26','b15')
    joint = assembly.joint(joint_id)
    print (joint_id)
    # print (joint.angle)
    print (joint.data)
    print (joint.assembly_tool_types(BeamAssemblyMethod.CLAMPED))
    print (joint.get_clamp_frames(process.assembly.beam(joint_id[0])))


    # artist.draw_beam(beam_id, delete_old=True, verbose=True)


    # from integral_timber_joints.process.compute_process_action_movement import assign_tool_type_to_joints
    # assign_tool_type_to_joints(process, beam_id, verbose=True)
    # assembly_method = assembly.get_assembly_method(beam_id)
    # for joint_id in assembly.get_joint_ids_of_beam(beam_id):
    #     print (joint_id)
    #     print (process.assembly.get_joint_attribute(joint_id, 'tool_type'))
    #     print (process.assembly.joint(joint_id).assembly_tool_types(assembly_method))


    # print(process.available_assembly_tool_types)