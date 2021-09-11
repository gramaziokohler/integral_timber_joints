try:
    from typing import Dict, List, Optional, Tuple

    from integral_timber_joints.process import RFLPathPlanner, RobotClampAssemblyProcess
except:
    pass
import itertools
from copy import deepcopy

from compas.geometry import Frame

from integral_timber_joints.assembly import Assembly, BeamAssemblyMethod
from integral_timber_joints.process.action import *
from integral_timber_joints.process.dependency import ComputationalResult
from integral_timber_joints.process.movement import *
from integral_timber_joints.process.state import SceneState
from integral_timber_joints.tools import Clamp, Gripper, Tool

######################################################
# Beam level functions - Assigning tool_type / tool_id
######################################################


def assign_tool_type_to_joints(process, beam_id, verbose=False):
    # type: (RobotClampAssemblyProcess, str, Optional[bool]) -> ComputationalResult
    """Assign tool_types to joints based on the
    (priority 1) user proference or
    (priority 2) joint's preference.

    If request tools are not available or not enough, return None.

    State Change
    ------------
    This functions sets the joint attribute `tool_type`

    Return
    ------
    `ComputationalResult.ValidCannotContinue` if no suitable clamp is found not satisfied
    `ComputationalResult.ValidCanContinue` otherwise (this function should not fail)
    """
    # Loop through all the beams and look at their previous_built neighbour.
    assembly = process.assembly
    something_failed = False
    something_changed = False

    assembly_method = assembly.get_assembly_method(beam_id)
    joint_ids = assembly.get_joint_ids_with_tools_for_beam(beam_id)

    if len(joint_ids) == 0:
        return ComputationalResult.ValidNoChange

    def vprint(str):
        if verbose:
            print(str)

    vprint("Beam %s (%s). Joints that need assignment: %s" % (beam_id, BeamAssemblyMethod.value_to_names_dict[assembly_method], joint_ids))

    # * Function to set the assignment results back to Joint Attributes
    def _set_assignments_to_joints(joint_ids, assignment):
        # type: (List[Tuple[str, str]], List[Tuple[str, str]]) -> None
        for (joint_id, (tool_type, tool_id)) in zip(joint_ids, assignment):
            assembly.set_joint_attribute(joint_id, 'tool_type', tool_type)
            assembly.set_joint_attribute(joint_id, 'tool_id', tool_id)

    # * For SCREWED_WITHOUT_GRIPPER, put the grasping joint first
    if assembly_method == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
        grasping_joint_id = process.assembly.get_grasping_joint_id(beam_id)
        joint_ids.remove(grasping_joint_id)
        joint_ids.insert(0, grasping_joint_id)

    # * (Priority 1) Try assign according to user preference + joint request
    type_requests = []
    for joint_id in joint_ids:
        preference = assembly.get_joint_attribute(joint_id, 'tool_type_preference')
        tool_type_request_from_joint = process.assembly.joint(joint_id).assembly_tool_types(assembly_method)
        if preference is not None:
            if preference in tool_type_request_from_joint:
                type_requests.append([preference])
            else:
                vprint("Warning: tool_type_preference: %s for Joint %s is no longer valid. Valid options are %s" % (preference, joint_id, tool_type_request_from_joint))
                type_requests.append(tool_type_request_from_joint)
        else:
            type_requests.append(tool_type_request_from_joint)
    assignment = assign_tool_types_by_formulated_request(process, type_requests)
    vprint("Type Request with user preference: %s" % type_requests)
    vprint("Assignment with user preference: %s" % assignment)
    if assignment is not None:
        _set_assignments_to_joints(joint_ids, assignment)
        return ComputationalResult.ValidCanContinue

    # (Priority 2) Ignore User Preference use joint.assembly_tool_types only
    type_requests = [process.assembly.joint(joint_id).assembly_tool_types(assembly_method) for joint_id in joint_ids]
    assignment = assign_tool_types_by_formulated_request(process, type_requests)
    vprint("Assignment with only joint reequest: %s" % assignment)
    if assignment is not None:
        _set_assignments_to_joints(joint_ids, assignment)
        return ComputationalResult.ValidCanContinue
    else:
        [assembly.set_joint_attribute(joint_id, 'tool_type', None) for joint_id in joint_ids]
        return ComputationalResult.ValidCannotContinue


def assign_tool_types_by_formulated_request(process, joints_tool_types_request):
    # type: (RobotClampAssemblyProcess, List[List[str]]) -> List[Tuple[str, str]]
    """Assign the tool_types to the request according to a given list of possible tool_types.

    `type_request is a list (per joint) of list (of tool_type).

    Returns a list of tuple containing assigned `tool_types` and `tool_id` in the same order of the original `joints_tool_types_request`
    If request tools are not available or not enough, return None.
    """
    if any([len(request) == 0 for request in joints_tool_types_request]):
        return None
    available_tools = process.get_available_tool_type_dict()
    assigned_tool_types = []

    def assign_one_joint(possible_types):
        """Returns True of success."""
        for tool_type in possible_types:
            if tool_type in available_tools and len(available_tools[tool_type]) > 0:
                assigned_tool_types.append((tool_type, available_tools[tool_type].pop(0)))
                return True
        return False  # None of the possible type are available

    # * Assigning to all the joints
    for tool_types_request in joints_tool_types_request:
        if not assign_one_joint(tool_types_request):
            return None

    # If passing all assignemnts
    return assigned_tool_types


if __name__ == "__main__":
    pass
