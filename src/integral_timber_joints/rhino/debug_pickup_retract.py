import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext as sc
import re
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.geometry import JointHalfLap, JointNonPlanarLap
from integral_timber_joints.rhino.assembly_artist import AssemblyNurbsArtist
from integral_timber_joints.assembly import BeamAssemblyMethod
from integral_timber_joints.tools import Clamp, Screwdriver, Gripper
from integral_timber_joints.process import RobotClampAssemblyProcess

import json

from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh
from compas.data import DataEncoder


from compas.geometry import Cylinder, Transformation, Translation, Vector, Frame



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
    # Check to ensure prerequisite
    if process.pickup_station is None:
        print("pickup_station is not set")

    def vprint(str):
        if verbose:
            print(str)

    # * Compute assembly_wcf_pickupretract
    assembly_wcf_pickup = process.assembly.get_beam_attribute(beam_id, 'assembly_wcf_pickup')
    vprint("assembly_wcf_pickup = %s" % (assembly_vector_at_pickup_in_wcf))

    retract_vector = process.pickup_station.pickup_retract_vector
    vprint("retract_vector = %s" % (retract_vector))
    t_beam_final_from_beam_at_pickup = Translation.from_vector(retract_vector)
    assembly_wcf_pickupretract = assembly_wcf_pickup.transformed(t_beam_final_from_beam_at_pickup)
    vprint("assembly_wcf_pickupretract = %s" % (assembly_wcf_pickupretract))

    process.assembly.set_beam_attribute(beam_id, 'assembly_wcf_pickupretract', assembly_wcf_pickupretract)
    vprint("process.assembly.set_beam_attribute(%s, 'assembly_wcf_pickupretract', %s)" % (beam_id, assembly_wcf_pickupretract))

    # Check if beam is of type SCREWED, otherwise stop here
    if process.assembly.get_beam_attribute(beam_id, 'assembly_method') not in BeamAssemblyMethod.screw_methods:
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
    process.assembly.set_beam_attribute(beam_id, 'assembly_wcf_screwdriver_attachment_pose', f_world_from_beam_at_newpose)
    vprint("process.assembly.set_beam_attribute(%s, 'assembly_wcf_screwdriver_attachment_pose', %s)" % (beam_id, f_world_from_beam_at_newpose))

    return ComputationalResult.ValidCanContinue


if __name__ == '__main__':
    process = get_process()
    assembly = process.assembly
    artist = get_process_artist()
    beam_id = 'b12'

    compute_beam_pickupretract(process, beam_id, verbose=True)
    # artist.draw_beam_brep(beam_id)
