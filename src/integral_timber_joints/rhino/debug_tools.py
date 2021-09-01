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
    beam_id = 'b13'
    beam = process.assembly.beam(beam_id)

    for joint_id in process.assembly.get_joint_ids_with_tools_for_beam(beam_id):
        print("joint_id", joint_id)
        print("- tool_id", process.assembly.get_joint_attribute(joint_id, "tool_id"))
        print("- tool_type", process.assembly.get_joint_attribute(joint_id, "tool_type"))

    print(process.assembly.get_beam_attribute(beam_id, "gripper_id"))
    print(process.assembly.get_beam_attribute(beam_id, "gripper_type"))
