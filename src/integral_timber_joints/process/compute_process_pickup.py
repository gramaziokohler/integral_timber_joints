from compas.geometry import Translation, Vector, Transformation, Frame, identity_matrix
from integral_timber_joints.assembly import BeamAssemblyMethod
from integral_timber_joints.process.dependency import ComputationalResult
from integral_timber_joints.tools import StackedPickupStation, GripperAlignedPickupStation, PickupStation

try:
    from typing import Dict, List, Optional, Tuple
    from integral_timber_joints.process import RobotClampAssemblyProcess
except:
    pass

# ---------------------------------------------------------------
# This file contains functions to be imported into Process class.
# They are separated here to keep individual files smaller.
# ---------------------------------------------------------------

# Computing Beam Pickup Related Attributes
# ------------------------------------

# Automatically Invoked Functions
# -------------------------------------


def compute_pickup_frame(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, bool) -> ComputationalResult
    """ Compute the pickup frame of a beam
    Beam assembly direcion must be valid. Grasp face and PickupStation and must be assigned before.

    - For GripperAlignedPickupStation, the pickup frame is pretty much fixed.
    - For (CornerAligned)PickupStation, `compute_pickup_location_by_aligning_corner` is called

    State Change
    ------------
    This functions sets the following beam_attribute
    - 'assembly_wcf_pickup'

    Return
    ------
    `ComputationalResult.ValidCannotContinue` if prerequisite not satisfied
    `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)

    """
    # * Skip if this is manually Assembled
    assembly_method = process.assembly.get_assembly_method(beam_id)
    if assembly_method == BeamAssemblyMethod.MANUAL_ASSEMBLY:
        return ComputationalResult.ValidCanContinue

    # Check to ensure prerequisite
    if process.pickup_station is None:
        return ComputationalResult.ValidCannotContinue

    # Switching computation function depdns on the Pickup station type
    if isinstance(process.pickup_station, StackedPickupStation):
        print("StackedPickupStation not implemented at compute_pickup_frame")
        return ComputationalResult.ValidCannotContinue

    elif isinstance(process.pickup_station, GripperAlignedPickupStation):
        if process.pickup_station.compute_pickup_frame(process, beam_id):
            return ComputationalResult.ValidCanContinue
        else:
            return ComputationalResult.ValidCannotContinue

    elif isinstance(process.pickup_station, PickupStation):
        return process.compute_pickup_location_by_aligning_corner(beam_id)

    else:
        print("Unknown Pickupstation type for compute_pickup_frame")
        return ComputationalResult.ValidCannotContinue


def compute_pickup_location_by_aligning_corner(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, bool) -> ComputationalResult
    """ Compute 'assembly_wcf_pickup' alignment frame
    by aligning a choosen corner relative to the 'gripper_grasp_face'
    to the given pickup_station_frame.

    Note
    ----
    This function cannot be used for beam center alignment.

    Side Effect
    -----------
    beam_attribute 'assembly_wcf_pickup' will be set.

    Return
    ------
    `ComputationalResult.ValidCannotContinue` if pickup_station is None
    `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)

    Example
    -------
    For a beam-start alignment: align_face_X0 = True, align_face_Y0 = True, align_face_Z0 = True
    For a beam-end alignment: align_face_X0 = False, align_face_Y0 = False, align_face_Z0 = True
    e.g
    """
    if process.pickup_station is None:
        return ComputationalResult.ValidCannotContinue

    pickup_station_frame = process.pickup_station.alignment_frame
    align_face_X0 = process.pickup_station.align_face_X0
    align_face_Y0 = process.pickup_station.align_face_Y0
    align_face_Z0 = process.pickup_station.align_face_Z0

    # Compute alignment frame origin
    beam = process.assembly.beam(beam_id)
    alignment_corner = _compute_alignment_corner_from_grasp_face(process,
                                                                 beam_id,
                                                                 process.pickup_station.align_face_X0,
                                                                 process.pickup_station.align_face_Y0,
                                                                 process.pickup_station.align_face_Z0)
    #print ('alignment_corner = %s' % alignment_corner)
    alignment_corner_ocf = beam.corner_ocf(alignment_corner)

    # Compute alignment frame X and Y axis - derived from 'gripper_grasp_face' frame
    gripper_grasp_face = process.assembly.get_beam_attribute(beam_id, 'gripper_grasp_face')
    gripper_grasp_face_frame_ocf = beam.reference_side_ocf(gripper_grasp_face)
    alignment_vector_X = gripper_grasp_face_frame_ocf.xaxis
    alignment_vector_Y = gripper_grasp_face_frame_ocf.yaxis
    if not align_face_X0:
        alignment_vector_X.scale(-1.0)
    if not align_face_Y0:
        alignment_vector_Y.scale(-1.0)

    # Alignment frame
    alignment_frame_ocf = Frame(alignment_corner_ocf, alignment_vector_X, alignment_vector_Y)

    # Compute the Transformation needed to bring beam OCF to meet with storage
    T = Transformation.from_frame(beam.frame)
    alignment_frame_wcf = alignment_frame_ocf.transformed(T)
    T = Transformation.from_frame_to_frame(alignment_frame_wcf, pickup_station_frame)

    # Compute the beam ocf in the storage position, save result 'assembly_wcf_pickup'
    assembly_wcf_pickup = beam.frame.transformed(T)
    process.assembly.set_beam_attribute(beam_id, 'assembly_wcf_pickup', assembly_wcf_pickup)

    print("compute_pickup_location_by_aligning_corner computed")
    return ComputationalResult.ValidCanContinue


def _compute_alignment_corner_from_grasp_face(process, beam_id, align_face_X0=True, align_face_Y0=True, align_face_Z0=True):
    # type: (RobotClampAssemblyProcess, str, bool, bool, bool) -> int
    """Returns one corner (int 1 - 8) of the chosen beam
    in relation to the picking face set in 'gripper_grasp_face'.
    There are 8 possible alignment relationship described by the three bool values.

    The retrived int can be used in beam.corner_ocf(alignment_corner)
    to retrive the corner coordinates in ocf of beam.

    Example
    -------
    Beam-start, Y-Neg, bottom-slignment:  align_face_X0 = True, align_face_Y0 = True, align_face_Z0 = True (typical)
    Beam-end, Y-Neg, bottom-slignment:  align_face_X0 = False, align_face_Y0 = False, align_face_Z0 = True
    Beam-start, Y-Pos, bottom-slignment:  align_face_X0 = True, align_face_Y0 = False, align_face_Z0 = True
    """
    # Compute storage alignment corner based on 'gripper_grasp_face'
    gripper_grasp_face = process.assembly.get_beam_attribute(beam_id, 'gripper_grasp_face')  # type: int
    assert gripper_grasp_face is not None
    print('gripper_grasp_face = %s' % gripper_grasp_face)

    corner = 0
    if (not align_face_Z0) and align_face_Y0:
        corner = 1
    elif align_face_Z0 and align_face_Y0:
        corner = 2
    elif align_face_Z0 and (not align_face_Y0):
        corner = 3
    elif (not align_face_Z0) and (not align_face_Y0):
        corner = 4
    else:
        raise Exception("Something is really wrong is aligning corners")

    # For corners 1 - 4, adding the corner number will suffice because corner 1 to 4 are corresponding to face 1 - 4
    corner = (gripper_grasp_face + corner - 2) % 4 + 1

    # Corner 5-8 is only related to whether align_face_X0
    if not align_face_X0:
        corner = corner + 4

    return corner


def compute_beam_pickupapproach(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, bool) -> ComputationalResult
    """ Compute gripper retract positions from 'assembly_wcf_pickup'.
    Approach vector is taken from gripper's approach vector (tcf) -> (beam ocf)

    Gripper, Pickupstation and beam_attribute'assembly_wcf_pickup' should be set before hand

    If assembly_method of beam is `SCREWED_WITHOUT_GRIPPER` , the function will not compute anything
    and will return `ComputationalResult.ValidCanContinue` immediately

    State Change
    ------------
    This functions sets the following beam_attribute
    - 'assembly_wcf_pickupapproach'

    Return
    ------
    `ComputationalResult.ValidCannotContinue` if prerequisite not satisfied
    `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)

    """
    assembly_method = process.assembly.get_assembly_method(beam_id)

    # * Skip MANUAL_ASSEMBLY
    if assembly_method == BeamAssemblyMethod.MANUAL_ASSEMBLY:
        if verbose:
            print("Skipping compute_beam_pickupapproach for MANUAL_ASSEMBLY")
        return ComputationalResult.ValidCanContinue

    # Check for Assembly Type
    if assembly_method == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
        return ComputationalResult.ValidCanContinue

    # Check to ensure prerequisite
    if process.pickup_station is None:
        print("pickup_station is not set")
        return ComputationalResult.ValidCannotContinue
    if process.assembly.get_beam_attribute(beam_id, 'assembly_wcf_pickup') is None:
        print("assembly_wcf_pickup is not set")
        return ComputationalResult.ValidCannotContinue
    if process.assembly.get_beam_attribute(beam_id, 'gripper_type') is None:
        print("gripper_type is not set")
        return ComputationalResult.ValidCannotContinue

    approach_vector_wcf_final = _compute_gripper_approach_vector_wcf_final(process, beam_id)

    # Express approach vector in World (wcf) when beam is in 'assembly_wcf_pickup'
    T = process.assembly.get_beam_transformaion_to(beam_id, 'assembly_wcf_pickup')
    approach_vector_wcf_storage = approach_vector_wcf_final.transformed(T)

    # Compute assembly_wcf_pickupapproach (wcf)
    T = Translation.from_vector(approach_vector_wcf_storage.scaled(-1))
    assembly_wcf_pickup = process.assembly.get_beam_attribute(beam_id, 'assembly_wcf_pickup')
    assert assembly_wcf_pickup is not None
    assembly_wcf_pickupapproach = assembly_wcf_pickup.transformed(T)
    process.assembly.set_beam_attribute(beam_id, 'assembly_wcf_pickupapproach', assembly_wcf_pickupapproach)

    return ComputationalResult.ValidCanContinue


def compute_beam_finalretract(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, bool) -> ComputationalResult
    """ Compute gripper retract positions from 'assembly_wcf_final'.
    Retraction direction and amount wcf) is taken from gripper attribute 'approach_vector', reversed.

    Gripper should be assigned before hand.

    If assembly_method of beam is `SCREWED_WITHOUT_GRIPPER` , the function will not compute anything
    and will return `ComputationalResult.ValidCanContinue` immediately

    State Change
    ------------
    This functions sets the following beam_attribute
    - 'assembly_wcf_finalretract'

    Return
    ------
    `ComputationalResult.ValidCannotContinue` if prerequisite not satisfied
    `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)

    """
    assembly_method = process.assembly.get_assembly_method(beam_id)

    # * Skip MANUAL_ASSEMBLY
    if assembly_method == BeamAssemblyMethod.MANUAL_ASSEMBLY:
        if verbose:
            print("Skipping compute_beam_finalretract for MANUAL_ASSEMBLY")
        return ComputationalResult.ValidCanContinue

    # Check for Assembly Type
    if assembly_method == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
        return ComputationalResult.ValidCanContinue

    # Check to ensure prerequisite
    if process.assembly.get_beam_attribute(beam_id, 'gripper_type') is None:
        print("gripper_type is not set")
        return ComputationalResult.ValidCannotContinue

    approach_vector_wcf_final = _compute_gripper_approach_vector_wcf_final(process, beam_id)

    # Compute assembly_wcf_finalretract (wcf)
    T = Translation.from_vector(approach_vector_wcf_final.scaled(-1))
    assembly_wcf_final = process.assembly.get_beam_attribute(beam_id, 'assembly_wcf_final')
    assembly_wcf_finalretract = assembly_wcf_final.transformed(T)
    process.assembly.set_beam_attribute(beam_id, 'assembly_wcf_finalretract', assembly_wcf_finalretract)

    return ComputationalResult.ValidCanContinue


def compute_beam_pickupretract(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, bool) -> ComputationalResult
    """Compute 'assembly_wcf_pickupretract' from beam attributes:
    by transforming 'assembly_wcf_pickup' along 'PickupStation.pickup_retract_vector'.

    For Beams in BeamAssemblyMethod.screw_methods, compute 'assembly_wcf_screwdriver_attachment_pose'
    The Beam assembly direction (representing the screwdriver pointy end direction)
    is used to guide the rotation of the beam above the retract frame.

    State Change
    ------------
    This functions sets the following beam_attribute
    - 'assembly_wcf_pickupretract'
    - 'assembly_wcf_screwdriver_attachment_pose' (BeamAssemblyMethod in screw_methods)

    Return
    ------
    `ComputationalResult.ValidCannotContinue` if prerequisite not satisfied
    `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)

    """
    assembly_method = process.assembly.get_assembly_method(beam_id)

    # * Skip MANUAL_ASSEMBLY
    if assembly_method == BeamAssemblyMethod.MANUAL_ASSEMBLY:
        if verbose:
            print("Skipping compute_beam_pickupretract for MANUAL_ASSEMBLY")
        return ComputationalResult.ValidCanContinue

    # Check to ensure prerequisite
    if process.pickup_station is None:
        print("pickup_station is not set")

    def vprint(str):
        if verbose:
            print(str)

    # * Compute assembly_wcf_pickupretract
    assembly_wcf_pickup = process.assembly.get_beam_attribute(beam_id, 'assembly_wcf_pickup')
    vprint("assembly_wcf_pickup = %s" % (assembly_wcf_pickup))

    retract_vector = process.pickup_station.pickup_retract_vector
    vprint("retract_vector = %s" % (retract_vector))
    t_beam_final_from_beam_at_pickup = Translation.from_vector(retract_vector)
    assembly_wcf_pickupretract = assembly_wcf_pickup.transformed(t_beam_final_from_beam_at_pickup)
    vprint("assembly_wcf_pickupretract = %s" % (assembly_wcf_pickupretract))

    process.assembly.set_beam_attribute(beam_id, 'assembly_wcf_pickupretract', assembly_wcf_pickupretract)
    vprint("process.assembly.set_beam_attribute(%s, 'assembly_wcf_pickupretract', %s)" % (beam_id, assembly_wcf_pickupretract))

    # Check if beam is of type SCREWED, otherwise stop here
    if assembly_method not in BeamAssemblyMethod.screw_methods:
        vprint("return ComputationalResult.ValidCanContinue")

    # * Compute assembly_wcf_screwdriver_attachment_pose
    beam = process.assembly.beam(beam_id)
    assembly_vector_final_in_wcf = process.assembly.get_beam_attribute(beam_id, 'assembly_vector_final')
    t_beam_final_from_beam_at_pickup = process.assembly.get_beam_transformaion_to(beam_id, 'assembly_wcf_pickup')
    assembly_vector_at_pickup_in_wcf = assembly_vector_final_in_wcf.transformed(t_beam_final_from_beam_at_pickup)
    vprint("assembly_vector_at_pickup_in_wcf = %s" % (assembly_vector_at_pickup_in_wcf))

    assembly_vector_up_amount = Vector(0, 0, 1).dot(assembly_vector_at_pickup_in_wcf)
    vprint("assembly_vector_up_amount = %s" % (assembly_vector_up_amount))

    def four_possible_rotations():
        rotations = []
        for i in range(1, 5):
            # ! This is the same code spelled out in matrix transformation:
            # t_world_from_beam_final = Transformation.from_frame(beam.get_face_frame(1))
            # t_world_from_beam_final_newpose = Transformation.from_frame(beam.get_face_frame(i))
            # t_beam_at_pickupretract_from_beam_newpose = t_world_from_beam_final.inverse() * t_world_from_beam_final_newpose
            # rotations.append(t_beam_at_pickupretract_from_beam_newpose)
            rotations.append(Transformation.from_change_of_basis(beam.get_face_frame(i), beam.get_face_frame(1)))
        return rotations

    t_world_from_beam_at_final = Transformation.from_frame(beam.frame)
    assembly_vector_in_ocf = assembly_vector_final_in_wcf.transformed(t_world_from_beam_at_final.inverse())
    vprint('assembly_vector_final_in_wcf = %s' % assembly_vector_final_in_wcf)
    vprint('assembly_vector_in_ocf = %s' % assembly_vector_in_ocf)

    possible_rotation_vectors = []
    possible_rotation_vector_dowm_amount = []
    # Trying all four possible rotations to get a new assembly and compare it so see which one points downwards
    for t_change_of_basis_rotation_at_final in four_possible_rotations():
        vprint("t_change_of_basis_rotation_at_final = %s" % t_change_of_basis_rotation_at_final)
        new_assembly_vector_in_ocf = assembly_vector_in_ocf.transformed(t_change_of_basis_rotation_at_final)
        vprint('new_assembly_vector_in_ocf = %s' % new_assembly_vector_in_ocf)
        new_assembly_vector_at_pickup_in_wcf = new_assembly_vector_in_ocf.transformed(t_world_from_beam_at_final).transformed(t_beam_final_from_beam_at_pickup)
        vprint('new_assembly_vector_at_pickup_in_wcf = %s' % new_assembly_vector_at_pickup_in_wcf)
        new_assembly_vector_down_amount = Vector(0, 0, -1).dot(new_assembly_vector_at_pickup_in_wcf)
        vprint('new_assembly_vector_down_amount = %s' % new_assembly_vector_down_amount)
        possible_rotation_vectors.append(t_change_of_basis_rotation_at_final)
        possible_rotation_vector_dowm_amount.append(new_assembly_vector_down_amount)
        vprint("--------------")

    # After identifying the best rotation. Apply it to find the new pose of the beam after pickup
    x = max(possible_rotation_vector_dowm_amount)

    best_rotation_index = possible_rotation_vector_dowm_amount.index(x)
    t_world_from_beam_at_pickupretract = Transformation.from_frame(assembly_wcf_pickupretract)
    t_world_from_beam_at_newpose = t_world_from_beam_at_pickupretract * possible_rotation_vectors[best_rotation_index]

    f_world_from_beam_at_newpose = Frame.from_transformation(t_world_from_beam_at_newpose)

    # Add some offset to world up direction
    f_world_from_beam_at_newpose.point = f_world_from_beam_at_newpose.point.transformed(Translation.from_vector([0,0,50]))

    process.assembly.set_beam_attribute(beam_id, 'assembly_wcf_screwdriver_attachment_pose', f_world_from_beam_at_newpose)
    vprint("process.assembly.set_beam_attribute(%s, 'assembly_wcf_screwdriver_attachment_pose', %s)" % (beam_id, f_world_from_beam_at_newpose))

    return ComputationalResult.ValidCanContinue


def _compute_gripper_approach_vector_wcf_final(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, bool) -> Vector
    """Compute gripper approach_vector (wcf)
    when beam is at final location (beam.frame)

    Return
    ------
    'approach_vector_wcf_final'
    """
    beam = process.assembly.beam(beam_id)
    if verbose:
        print("beam_id = %s " % beam_id)

    # Get approach vector from gripper
    gripper_type = process.assembly.get_beam_attribute(beam_id, 'gripper_type')
    assert gripper_type is not None
    gripper = process.get_one_gripper_by_type(gripper_type)
    approach_vector_tcf = gripper.approach_vector.transformed(gripper.t_t0cf_from_tcf)
    if verbose:
        print("approach_vector_tcf = %s " % approach_vector_tcf)

    # Express the approach_vector in ocf of beam (beam.frame coordinate frame)
    gripper_tcp_in_ocf = process.assembly.get_beam_attribute(beam_id, 'gripper_tcp_in_ocf')
    T = Transformation.from_frame_to_frame(Frame.worldXY(), gripper_tcp_in_ocf)
    approach_vector_ocf = approach_vector_tcf.transformed(T)
    if verbose:
        print("approach_vector_ocf = %s " % approach_vector_ocf)

    # Express approach vector in World (wcf) for beam in 'assembly_wcf_final'
    T = Transformation.from_frame_to_frame(Frame.worldXY(), beam.frame)
    approach_vector_wcf_final = approach_vector_ocf.transformed(T)
    if verbose:
        print("approach_vector_wcf_final = %s " % approach_vector_wcf_final)

    return approach_vector_wcf_final


def compute_storeage_frame(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, bool) -> ComputationalResult
    """Compute the storage frame of a beam in the stack.

    Note
    -----
    Changing assembly sequence or storage location should require a recomputation.


    State Change
    ------------
    This functions sets the following beam_attribute
    - 'assembly_wcf_storage'

    Return
    ------
    `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)

    """
    # Use the origin as storage frame if there is no beam_storage object
    if process.beam_storage is None:
        process.assembly.set_beam_attribute(beam_id, 'assembly_wcf_storage', Frame.worldXY())

    beam = process.assembly.beam(beam_id)
    storage_frame = process.beam_storage.get_storage_frame(process.assembly.sequence.index(beam_id), len(process.assembly.sequence))
    process.assembly.set_beam_attribute(beam_id, 'assembly_wcf_storage', storage_frame)
    return ComputationalResult.ValidCanContinue
