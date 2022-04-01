from compas.geometry import Translation, Transformation
from integral_timber_joints.assembly import BeamAssemblyMethod
from integral_timber_joints.process.dependency import ComputationalResult

try:
    from typing import Dict, List, Optional, Tuple
    from integral_timber_joints.process import RobotClampAssemblyProcess
    from integral_timber_joints.tools.screwdriver import Screwdriver
except:
    pass
# ---------------------------------------------------------------
# This file contains functions to be imported into Process class.
# They are separated here to keep individual files smaller.
# ---------------------------------------------------------------

# Computing Screwdriver Related Attributes
# ----------------------------------------

# Automatically Invoked Functions
# -------------------------------------


def compute_screwdriver_positions(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, bool) -> ComputationalResult
    """Compute /pre-compute Screwdriver attached frames,
    saving the results in process.assembly.joint_attribute.

    The results are used for key-position preview and also for generating Movements.
    """
    assembly = process.assembly
    beam = assembly.beam(beam_id)

    # Check if beam is of type SCREWED, otherwise, return
    if assembly.get_beam_attribute(beam_id, 'assembly_method') not in BeamAssemblyMethod.screw_methods:
        return ComputationalResult.ValidNoChange
    # For every joint we compute the screwdriver position
    for joint_id in assembly.get_joint_ids_with_tools_for_beam(beam_id):
        tool = process.get_tool_of_joint(joint_id, None)  # type: Screwdriver
        if tool is None:
            return ComputationalResult.ValidCannotContinue

        # * screwdriver_assembled_attached
        # Get the tool attachment frame from the joint on beams
        neighbout_joint_id = (joint_id[1], joint_id[0])
        joint = assembly.joint(neighbout_joint_id)
        clamp_attachment_frames = joint.get_clamp_frames(assembly.beam(beam_id))
        tool_orientation_frame_index = process.assembly.get_joint_attribute(joint_id, 'tool_orientation_frame_index')
        tool_tcp_frame_in_wcf = clamp_attachment_frames[tool_orientation_frame_index]

        # * Compute Tool Grasp and set it back to Joint Attribute
        t_world_from_tool_tcf = Transformation.from_frame(tool_tcp_frame_in_wcf)
        t_beam_from_world = Transformation.from_frame(beam.frame).inverse()
        t_beam_ocf_from_tool_tcp = t_beam_from_world * t_world_from_tool_tcf
        process.assembly.set_joint_attribute(joint_id, 't_beam_ocf_from_tool_tcp', t_beam_ocf_from_tool_tcp)

        # Set clamp tcp to selected_frame and compute tool vectors in wcf
        tool.set_current_frame_from_tcp(tool_tcp_frame_in_wcf)
        approach_vector_wcf = tool.current_frame.to_world_coordinates(tool.approach_vector)
        detachretract_vector_wcf = tool.current_frame.to_world_coordinates(tool.detachretract_vector)

        # Begin assemble distance
        assemblebegin_length = tool.approach_vector.length - process.screwdriver_begin_assemble_offset
        assembled_retracted_length = tool.detachretract_vector.length - process.screwdriver_begin_assemble_offset

        # Calculation of Beam Positions
        # -----------------------------

        # * assembly_wcf_final
        assembly.set_beam_attribute(beam_id, 'assembly_wcf_final', beam.frame.copy())

        # * assembly_wcf_assemblebegin
        # Moving beam frame backward along reversed tool.approach_vector.
        f_assemblebegin = beam.frame.transformed(Translation.from_vector(approach_vector_wcf.unitized().scaled(-assemblebegin_length)))
        assembly.set_beam_attribute(beam_id, 'assembly_wcf_assemblebegin', f_assemblebegin)

        # * assembly_wcf_assembleapproach
        # Moving beam frame backward along reversed tool.approach_vector.
        screwdriver_approach_distance = process.screwdriver_approach_distance
        v_assembleapproach = approach_vector_wcf.unitized().scaled(screwdriver_approach_distance + approach_vector_wcf.length)
        f_assembleapproach = beam.frame.transformed(Translation.from_vector(v_assembleapproach.scaled(-1)))
        assembly.set_beam_attribute(beam_id, 'assembly_wcf_assembleapproach', f_assembleapproach)

        # Calculation of Tool Positions in assembled area
        # -------------------------------------------------------------

        # * screwdriver_assembled_attached
        # The tool was set at its assembled state, its base frame is the basis of the following calculation
        f_assembled_attached = tool.current_frame
        assembly.set_joint_attribute(joint_id, 'screwdriver_assembled_attached', f_assembled_attached)

        # * screwdriver_assembled_retracted
        # Moving tool backwards along tool.detachretract_vector.
        f_assembled_detached = f_assembled_attached.transformed(Translation.from_vector(detachretract_vector_wcf.unitized().scaled(assembled_retracted_length)))
        assembly.set_joint_attribute(joint_id, 'screwdriver_assembled_retracted', f_assembled_detached)

        # * screwdriver_assembled_retractedfurther
        # Moving tool backwards along tool.detachretract_vector + a little more distance
        detachretracted_vector_wcf = detachretract_vector_wcf.unitized().scaled(screwdriver_approach_distance + detachretract_vector_wcf.length)
        f_assembled_retracted = f_assembled_attached.transformed(Translation.from_vector(detachretracted_vector_wcf))
        assembly.set_joint_attribute(joint_id, 'screwdriver_assembled_retractedfurther', f_assembled_retracted)

        # * screwdriver_assemblebegin_attached
        # Moving tool backwards along reversed tool.approach_vector.
        f_assemblebegin_attached = f_assembled_attached.transformed(Translation.from_vector(approach_vector_wcf.scaled(-1)))
        assembly.set_joint_attribute(joint_id, 'screwdriver_assemblebegin_attached', f_assemblebegin_attached)

        # * screwdriver_assembleapproach_attached
        # Moving tool backwards along a lengthened  tool.approach_vector + a little more distance
        longer_approach_vector_wcf = approach_vector_wcf.unitized().scaled(screwdriver_approach_distance + approach_vector_wcf.length)
        frame = f_assembled_attached.transformed(Translation.from_vector(longer_approach_vector_wcf.scaled(-1)))
        assembly.set_joint_attribute(joint_id, 'screwdriver_assembleapproach_attached', frame)

        # Calculation of Tool Positions in pickup area
        # -------------------------------------------------------------

        # * screwdriver_assembled_retracted
        # Moving tool backwards along tool.detachretract_vector.
        f_pickup_attached = f_assembled_attached.transformed(process.assembly.get_beam_transformaion_to(beam_id, 'assembly_wcf_pickup'))
        assembly.set_joint_attribute(joint_id, 'screwdriver_pickup_attached', f_pickup_attached)

        f_pickup_attached = f_assembled_attached.transformed(process.assembly.get_beam_transformaion_to(beam_id, 'assembly_wcf_screwdriver_attachment_pose'))
        assembly.set_joint_attribute(joint_id, 'screwdriver_attachment_pose_attached', f_pickup_attached)

        # TODO assembly_wcf_pickupretract

    return ComputationalResult.ValidCanContinue
