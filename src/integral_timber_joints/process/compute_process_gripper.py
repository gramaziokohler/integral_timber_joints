from compas.geometry import Translation, Vector, Transformation
from integral_timber_joints.assembly import BeamAssemblyMethod
from integral_timber_joints.process.dependency import ComputationalResult

try:
    from typing import Dict, List, Optional, Tuple
    from integral_timber_joints.process import RobotClampAssemblyProcess
    from integral_timber_joints.tools import Gripper, Tool
except:
    pass

# ---------------------------------------------------------------
# This file contains functions to be imported into Process class.
# They are separated here to keep individual files smaller.
# ---------------------------------------------------------------

# Computing Gripper Related Attributes
# ------------------------------------

# Automatically Invoked Functions
# -------------------------------------


def assign_gripper_to_beam(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, bool) -> ComputationalResult
    """Assign a gripper type using available grippers based on the beam's length.
    Beam must fit within gripper `beam_length_limits`, if multiple options allow,
    the gripper with the closest `target_beam_length` will be chosen.

    If the attribute `gripper_type` is already assigned, this function will not chage it.

    State Change
    ------------
    This functions sets the following beam_attribute
    - 'gripper_type'
    - 'gripper_id'

    Return
    ------
    `ComputationalResult.ValidCannotContinue` if no suitable gripper can be found
    `ComputationalResult.ValidCanContinue` if a suitable gripper can be found
    """
    beam_length = process.assembly.beam(beam_id).length
    chosen_gripper_type = None
    chosen_gripper_ideal = None

    # Do not change anything if gripper_type is already set
    already_set = False
    if process.assembly.get_beam_attribute(beam_id, "gripper_type") is not None:
        if verbose:
            print("assign_gripper_to_beam: gripper_type set")
        if process.assembly.get_beam_attribute(beam_id, "gripper_id") is not None:
            if verbose:
                print("assign_gripper_to_beam: gripper_id set")

            # Check that the gripper_id is sensible
            if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
                if verbose:
                    print("assign_gripper_to_beam: assembly method = %s " % process.assembly.get_assembly_method(beam_id))

                if process.assembly.get_beam_attribute(beam_id, "gripper_id") in [tool.name for tool in process.screwdrivers]:
                    if verbose:
                        print("assign_gripper_to_beam: gripper_id %s is valid and will not be changed." % process.assembly.get_beam_attribute(beam_id, "gripper_id"))
                    already_set = True
            else:
                if process.assembly.get_beam_attribute(beam_id, "gripper_id") in [tool.name for tool in process.grippers]:
                    if verbose:
                        print("assign_gripper_to_beam: gripper_id %s is valid and will not be changed." % process.assembly.get_beam_attribute(beam_id, "gripper_id"))
                    already_set = True
    if already_set:
        if verbose:
            print("Beam (%s) gripper_type (%s) has already been set. No change made by assign_gripper_to_beam()." %
                (beam_id, process.assembly.get_beam_attribute(beam_id, "gripper_type")))
        return ComputationalResult.ValidNoChange

    if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
        joint_ids = process.assembly.get_joint_ids_with_tools_for_beam(beam_id)
        first_screwdriver = process.get_tool_of_joint(joint_ids[0])
        chosen_gripper_type = first_screwdriver.type_name
        gripper_id = first_screwdriver.name
        if verbose:
            print("chosen_gripper_type = %s" % chosen_gripper_type)
            print("gripper_id = %s" % gripper_id)
    else:
        # Compute Gripper Type
        for gripper_type in process.available_gripper_types:
            gripper = process.get_one_gripper_by_type(gripper_type)
            # Check if beam length is within limits
            if beam_length >= gripper.beam_length_limits[0] and beam_length <= gripper.beam_length_limits[1]:
                # Compute beam length vs ideal length and make decision
                length_to_ideal = abs(beam_length - gripper.target_beam_length)
                if chosen_gripper_type is None or length_to_ideal < chosen_gripper_ideal:
                    chosen_gripper_type = gripper_type
                    chosen_gripper_ideal = length_to_ideal

        # In cases no suitable gripper is available
        if chosen_gripper_type is None:
            if verbose:
                print("No suitable gripper assigned to %s" % (beam_id))
            return ComputationalResult.ValidCannotContinue
        gripper_id = process.get_one_tool_by_type(chosen_gripper_type).name

    # Set gripper_type and gripper_id and return
    process.assembly.set_beam_attribute(beam_id, "gripper_type", chosen_gripper_type)
    process.assembly.set_beam_attribute(beam_id, "gripper_id", gripper_id)
    if verbose:
        print("Gripper Type: %s assigned to %s" % (chosen_gripper_type, beam_id))

    return ComputationalResult.ValidCanContinue


def compute_gripper_grasp_pose(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, bool) -> ComputationalResult
    """ Compute grasp pose for the beam and gripper.
    Default values will be applied if 'gripper_grasp_dist_from_start' and 'gripper_grasp_face'
    are not set. Otherwise previous values will be preserved to calculate 'gripper_tcp_in_ocf'.

    Gripper should be assigned before.

    State Change
    ------------
    This functions sets the following beam_attribute
    - 'gripper_grasp_dist_from_start' (if default)
    - 'gripper_grasp_face' (if default)
    - 'gripper_tcp_in_ocf'

    Return
    ------
    `ComputationalResult.ValidCannotContinue` if prerequisite not satisfied
    `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)

    """
    # Check to ensure prerequisite
    if process.assembly.get_beam_attribute(beam_id, 'gripper_type') is None:
        return ComputationalResult.ValidCannotContinue

    beam = process.assembly.beam(beam_id)

    # * Computing `gripper_grasp_face` if it is None
    def grasp_face(beam_id):
        return process.assembly.get_beam_attribute(beam_id, "gripper_grasp_face")

    # Apply default values if None
    if grasp_face(beam_id) not in [1, 2, 3, 4]:  # Default method
        gripper_grasp_face = process.set_grasp_face_following_assembly_direction(beam_id)

    if grasp_face(beam_id) not in [1, 2, 3, 4]:  # Backup plan
        gripper_grasp_face = process.set_grasp_face_following_guide_vector(beam_id)

    if grasp_face(beam_id) not in [1, 2, 3, 4]:  # Picking face 1 and deal with it
        process.assembly.set_beam_attribute(beam_id, "gripper_grasp_face", 1)
        print("Someting wrong, gripper_grasp_face is not in [1,2,3,4] after search. Grasp face defaulted to ", 1)

    # * Computing `gripper_grasp_dist_from_start` if it is None
    gripper_grasp_dist_from_start = process.assembly.get_beam_attribute(beam_id, "gripper_grasp_dist_from_start")
    if gripper_grasp_dist_from_start is None:
        gripper_grasp_dist_from_start = beam.length / 2.0
        process.assembly.set_beam_attribute(beam_id, "gripper_grasp_dist_from_start", gripper_grasp_dist_from_start)

    # * Compute Gripper Grasp Pose, aka. gripper_tcp_in_ocf
    gripper_tcp_in_ocf = beam.grasp_frame_ocf(grasp_face(beam_id), gripper_grasp_dist_from_start)
    process.assembly.set_beam_attribute(beam_id, "gripper_tcp_in_ocf", gripper_tcp_in_ocf)
    return ComputationalResult.ValidCanContinue


def set_grasp_face_following_assembly_direction(process, beam_id):
    # type: (RobotClampAssemblyProcess, str) -> int
    """Return the best face number (1-4) for creating `gripper_tcp_in_ocf`
    where grasp face normal is the opposite direction of the beam's assembly direction.

    State Change
    ------------
    This functions sets the following beam_attribute
    - 'gripper_grasp_face'

    Dependency Trigger
    ------------------
    Invalidate: 'compute_gripper_grasp_pose' and downstream
    """
    for joint_id in process.assembly.get_joints_of_beam_connected_to_already_built(beam_id):
        joint = process.assembly.joint(joint_id)
        selected_face = (joint.face_id + 1) % 4 + 1
        process.assembly.set_beam_attribute(beam_id, 'gripper_grasp_face', selected_face)
        # Dependency Trigger
        process.dependency.invalidate(beam_id, process.compute_gripper_grasp_pose)
        # Only the first joint is considered
        return selected_face


def set_grasp_face_following_guide_vector(process, beam_id):
    # type: (RobotClampAssemblyProcess, str) -> int
    """Return the best face number (1-4) for creating `gripper_tcp_in_ocf`
    where the Z-Axis of the tcp_in_WCF, when beam is at 'assembly_wcf_final',
    follows the direction of guide vector `design_guide_vector_grasp`

    Side Effect
    -----------
    beam_attribute 'gripper_grasp_face' will be set.
    """
    # Get the guide Vector from beam_attribute
    design_guide_vector_grasp = process.assembly.get_beam_attribute(beam_id, 'design_guide_vector_grasp').unitized()
    assert design_guide_vector_grasp is not None

    # Try different grasp face and choose the one that aligns best.
    beam = process.assembly.beam(beam_id)
    best_face = 0
    best_score = -1
    for gripper_grasp_face in range(1, 5):
        gripper_tcp_in_ocf = beam.grasp_frame_ocf(gripper_grasp_face, 0)
        gripper_tcp_in_wcf = gripper_tcp_in_ocf.transformed(Transformation.from_frame(beam.frame))
        # Compute the alignment score using dot product
        alignment_score = gripper_tcp_in_wcf.zaxis.dot(design_guide_vector_grasp)
        if alignment_score > best_score:
            best_score = alignment_score
            best_face = gripper_grasp_face

    process.assembly.set_beam_attribute(beam_id, 'gripper_grasp_face', best_face)
    return best_face

# ------------------------------------
# Manually invoked Functions
# -------------------------------------


def adjust_gripper_pos(process, beam_id, amount):
    # type: (RobotClampAssemblyProcess, str, float) -> bool
    """ Modify the grasp pose 'gripper_grasp_dist_from_start'

    'gripper_tcp_in_ocf'

    Gripper should be assigned before.

    State Change
    ------------
    This functions updates the following beam_attribute
    - 'gripper_grasp_dist_from_start'
    - 'gripper_tcp_in_ocf'

    Return
    ------
    False if prerequisite not satisfied
    True, if setting is successdul otherwise (this function should not fail)

    Dependency Trigger
    ------------------
    Invalidate: 'compute_gripper_grasp_pose' and downstream
    """
    # Check to ensure prerequisite
    assembly = process.assembly
    beam = assembly.beam(beam_id)

    if assembly.get_beam_attribute(beam_id, 'gripper_type') is None:
        return False

    gripper_grasp_face = assembly.get_beam_attribute(beam_id, "gripper_grasp_face")
    gripper_grasp_dist_from_start = assembly.get_beam_attribute(beam_id, "gripper_grasp_dist_from_start")
    gripper_grasp_dist_from_start += amount
    assembly.set_beam_attribute(beam_id, "gripper_grasp_dist_from_start", gripper_grasp_dist_from_start)

    # Recompute beam grasp_frame
    gripper_tcp_in_ocf = beam.grasp_frame_ocf(gripper_grasp_face, gripper_grasp_dist_from_start)
    assembly.set_beam_attribute(beam_id, "gripper_tcp_in_ocf", gripper_tcp_in_ocf)
    # Dependency Trigger
    process.dependency.invalidate(beam_id, process.compute_gripper_grasp_pose)
    return True


def override_grasp_face(process, beam_id, grasp_face):
    # type: (RobotClampAssemblyProcess, str, float) -> bool
    """Manually override `gripper_grasp_face` for a specified beam
    `grasp_face` can only be within 1 - 4, overrange value will be wrapped

    State Change
    ------------
    This functions sets the following beam_attribute
    - 'gripper_grasp_face'

    Dependency Trigger
    ------------------
    Invalidate: 'compute_gripper_grasp_pose' and downstream

    """
    grasp_face = (grasp_face - 1) % 4 + 1
    process.assembly.set_beam_attribute(beam_id, 'gripper_grasp_face', grasp_face)
    # Dependency Trigger
    process.dependency.invalidate(beam_id, process.compute_gripper_grasp_pose)
    return True
