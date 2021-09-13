import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext
import re
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.rhino.assembly_artist import AssemblyNurbsArtist
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.process import ComputationalResult
from integral_timber_joints.assembly import BeamAssemblyMethod

from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh

try:
    from typing import Dict, List, Optional, Tuple

    from integral_timber_joints.process import RFLPathPlanner
except:
    pass



if __name__ == '__main__':
    process = get_process()
    # artist = get_process_artist()
    # artist = AssemblyNurbsArtist(process.assembly)
    beam_id = 'b3'
    beam = process.assembly.beam(beam_id)

    # process.assembly.set_joint_attribute(('b1', 'b3'), "tool_type_preference", 'SL1_G200')
    # process.assembly.set_joint_attribute(('b0', 'b3'), "tool_type_preference", 'SL1_G200')
    # assign_tool_type_to_joints(process, beam_id, True)

    # print (process.get_available_tool_type_dict())
    for beam_id in process.assembly.sequence:
        if beam_id != 'b4' : continue
        process.assign_tool_type_to_joints(beam_id, True)
        print ("-------------------------")
        print (beam_id)
        # process.assign_tool_type_to_joints(beam_id, True)

        for joint_id in process.assembly.get_joint_ids_with_tools_for_beam(beam_id):
            print("joint_id", joint_id)
            print("- tool_type_preference", process.assembly.get_joint_attribute(joint_id, "tool_type_preference"))
            print("- tool_type", process.assembly.get_joint_attribute(joint_id, "tool_type"))
            print("- tool_id", process.assembly.get_joint_attribute(joint_id, "tool_id"))

    # print(process.assembly.get_beam_attribute(beam_id, "gripper_id"))
    # print(process.assembly.get_beam_attribute(beam_id, "gripper_type"))
