from functools import partial
from termcolor import cprint, colored
from copy import copy

from compas.geometry import Transformation, Frame
from compas.robots.configuration import Configuration

from integral_timber_joints.process.state import ObjectState, SceneState, copy_state_dict
from integral_timber_joints.planning.visualization import color_from_object_id
from integral_timber_joints.process.action import LoadBeamAction, PickGripperFromStorageAction, \
    PickBeamWithGripperAction, PickClampFromStorageAction, PlaceClampToStructureAction, BeamPlacementWithClampsAction, \
    PlaceGripperToStorageAction, PlaceClampToStorageAction, PickClampFromStructureAction, BeamPlacementWithoutClampsAction
from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticMovement, RoboticClampSyncLinearMovement, RobotScrewdriverSyncLinearMovement
from integral_timber_joints.process.dependency import ComputationalResult

##############################################

def assign_ik_conf_to_action(process, action, conf):
    # * trigger movement computation
    action.create_movements(process)
    action.assign_movement_ids()
    for movement in action.movements:
        movement.create_state_diff(process)
    for mov_n, movement in enumerate(action.movements):
        # if verbose:
        #     print("Processing Seq %i Action %i Movement %i: %s" % (action.seq_n, action.act_n, mov_n, movement.tag))
        if (isinstance(movement, RoboticLinearMovement) and 'Advance' in movement.tag) or \
            isinstance(movement, RoboticClampSyncLinearMovement):
            process.set_movement_end_robot_config(movement, conf)
    return action

def actions_from_pddlstream_plan(process, plan, verbose=False):
    place_actions = [action for action in plan if action.name == 'place_element_on_structure']
    beam_sequence = [ac.args[0] for ac in place_actions]
    assert beam_sequence == process.assembly.sequence, 'Don\'t support auto-plan assembly sequence for now!'

    color_print_fn = partial(colored_str_from_object, show_details=False)
    beam_id = last_beam_id = ''
    seq_n = 0
    acts = []
    for pddl_action in plan:
        print('='*10)
        name, args = pddl_action
        print('{} {}'.format(colored(name, 'green'), ' '.join(map(color_print_fn, args))))

        itj_act = None
        conf = None
        if pddl_action.name == 'operator_load_element_on_rack':
            beam_id = pddl_action.args[0]
            itj_act = LoadBeamAction(seq_n, 0, beam_id)

        elif pddl_action.name == 'pick_element_from_rack':
            # TODO screwdriver needs special care here
            beam_id = pddl_action.args[0]
            gripper_id = pddl_action.args[3]
            gripper = process.gripper(gripper_id)
            # * double-check tool type consistency
            gt_gripper_type = process.assembly.get_beam_attribute(beam_id, "gripper_type")
            # assert gt_gripper_type == gripper.type_name, '{} should use gripper with type {} but {} with type {} assigned.'.format(beam_id, gt_gripper_type, gripper.name, gripper.type_name)

            if gt_gripper_type == gripper.type_name:
                '{} should use gripper with type {} but {} with type {} assigned.'.format(beam_id, gt_gripper_type, gripper.name, gripper.type_name)

            conf = pddl_action.args[-1]
            itj_act = PickBeamWithGripperAction(seq_n, 0, beam_id, gripper_id)

        elif pddl_action.name == 'place_element_on_structure':
            beam_id = pddl_action.args[0]
            gripper_id = pddl_action.args[-3]
            gripper = process.gripper(gripper_id)
            # * double-check tool type consistency
            gt_gripper_type = process.assembly.get_beam_attribute(beam_id, "gripper_type")
            assert gt_gripper_type == gripper.type_name, '{} should use gripper with type {} but {} with type {} assigned.'.format(beam_id, gt_gripper_type, gripper.name, gripper.type_name)
            # if gt_gripper_type != gripper.type_name:
            #     print('{} should use gripper with type {} but {} with type {} assigned.'.format(beam_id, gt_gripper_type, gripper.name, gripper.type_name))

            joint_id_of_clamps = list(process.assembly.get_joint_ids_with_tools_for_beam(beam_id))
            clamp_ids = [process.assembly.get_joint_attribute(joint_id, 'tool_id') for joint_id in joint_id_of_clamps]
            if len(clamp_ids) == 0:
                # ! beams without clamp
                itj_act = BeamPlacementWithoutClampsAction(seq_n, 0, beam_id, gripper_id)
            else:
                # ! beams with clamp
                itj_act = BeamPlacementWithClampsAction(seq_n, 0, beam_id, joint_id_of_clamps, gripper_id, clamp_ids)
            conf = pddl_action.args[-1]

        elif pddl_action.name == 'pick_tool_from_rack':
            tool_id = pddl_action.args[0]
            if tool_id.startswith('g'):
                gripper = process.gripper(tool_id)
                itj_act = PickGripperFromStorageAction(seq_n, 0, gripper.type_name, tool_id)
            elif tool_id.startswith('c'):
                clamp = process.clamp(tool_id)
                itj_act = PickClampFromStorageAction(seq_n, 0, clamp.type_name, tool_id)
            else:
                raise ValueError('Weird tool id {}'.format(tool_id))
            conf = pddl_action.args[-1]

        elif pddl_action.name == 'place_tool_at_rack':
            tool_id = pddl_action.args[0]
            if tool_id.startswith('g'):
                gripper = process.gripper(tool_id)
                itj_act = PlaceGripperToStorageAction(seq_n, 0, gripper.type_name, tool_id)
            elif tool_id.startswith('c'):
                clamp = process.clamp(tool_id)
                itj_act = PlaceClampToStorageAction(tool_type=clamp.type_name, tool_id=tool_id)
            else:
                raise ValueError('Weird tool id {}'.format(tool_id))
            conf = pddl_action.args[-1]


        elif pddl_action.name == 'pick_clamp_from_joint':
            clamp_id = pddl_action.args[0]
            clamp = process.clamp(clamp_id)
            joint_id = (pddl_action.args[-3], pddl_action.args[-2])
            # ! convention: sequence id smaller first
            if process.assembly.sequence.index(joint_id[0]) > process.assembly.sequence.index(joint_id[1]):
                joint_id = joint_id[::-1]
            itj_act = PickClampFromStructureAction(joint_id=joint_id, tool_type=clamp.type_name, tool_id=clamp_id)
            conf = pddl_action.args[-1]

        elif pddl_action.name == 'place_clamp_at_joint':
            clamp_id = pddl_action.args[0]
            clamp = process.clamp(clamp_id)
            joint_id = (pddl_action.args[-3], pddl_action.args[-2])
            # ! convention: sequence id smaller first
            if process.assembly.sequence.index(joint_id[0]) > process.assembly.sequence.index(joint_id[1]):
                joint_id = joint_id[::-1]
            # * double-check clamp type consistency
            gt_clamp_type = process.assembly.get_joint_attribute(joint_id, 'tool_type')
            assert gt_clamp_type == clamp.type_name, 'Joint {} should use clamp with type {} but {} with type {} assigned.'.format(joint_id, gt_clamp_type, clamp.name, clamp.type_name)
            # if gt_clamp_type != clamp.type_name:
            #     print('Joint {} should use clamp with type {} but {} with type {} assigned.'.format(joint_id, gt_clamp_type, clamp.name, clamp.type_name))

            # * clamp assignment to beam
            process.assembly.set_joint_attribute(joint_id, 'tool_id', clamp_id)
            itj_act = PlaceClampToStructureAction(seq_n, 0, joint_id, clamp.type_name, clamp_id)
            conf = pddl_action.args[-1]

        elif pddl_action.name == 'move':
            continue

        assert itj_act is not None
        # * compute diff_state and assign conf
        itj_act = assign_ik_conf_to_action(process, itj_act, conf)
        acts.append(itj_act)

        if pddl_action.name == 'place_element_on_structure':
            assert len(acts) > 0
            if beam_id != last_beam_id and beam_id != beam_sequence[-1]:
                if verbose:
                    print('='*10)
                    print('Beam id: {}'.format(beam_id))
                    for act in acts:
                        print('|- ' + act.__str__())
                process.assembly.set_beam_attribute(beam_id, 'actions', acts)
                last_beam_id = beam_id
                seq_n += 1
                acts = []

    # * last beam and tool collection
    assert beam_id == beam_sequence[-1] and len(acts) > 0
    if verbose:
        print('='*10)
        print('Beam id: {}'.format(beam_id))
        for act in acts:
            print('|- ' + act.__str__())
    process.assembly.set_beam_attribute(beam_id, 'actions', acts)

    return ComputationalResult.ValidCanContinue

##########################################

from compas.geometry import Frame, Transformation
from pddlstream.utils import str_from_object
from pddlstream.language.constants import is_plan, DurativeAction, Action, StreamAction, FunctionAction

def contains_number(value):
    for character in value:
        if character.isdigit():
            return True
    return False

def colored_str_from_object(obj, show_details=False):
    if not show_details:
        if isinstance(obj, Frame):
            return '(frm)'
        elif isinstance(obj, Transformation):
            return '(tf)'
        elif isinstance(obj, Configuration):
            return colored('(conf)', 'yellow')

    str_rep = str_from_object(obj)
    if contains_number(str_rep):
        return colored(str_rep, 'blue')
    else:
        return colored(str_rep, 'red')

def print_itj_pddl_plan(plan, show_details=False):
    if not is_plan(plan):
        return
    step = 1
    color_print_fn = partial(colored_str_from_object, show_details=show_details)
    for action in plan:
        if isinstance(action, DurativeAction):
            name, args, start, duration = action
            print('{:.2f} - {:.2f}) {} {}'.format(start, start+duration, name,
                                                  ' '.join(map(str_from_object, args))))
        elif isinstance(action, Action):
            name, args = action
            print('{:2}) {} {}'.format(step, colored(name, 'green'), ' '.join(map(color_print_fn, args))))
            step += 1
        elif isinstance(action, StreamAction):
            name, inputs, outputs = action
            print('    {}({})->({})'.format(name, ', '.join(map(str_from_object, inputs)),
                                            ', '.join(map(str_from_object, outputs))))
        elif isinstance(action, FunctionAction):
            name, inputs = action
            print('    {}({})'.format(name, ', '.join(map(str_from_object, inputs))))
        else:
            raise NotImplementedError(action)

