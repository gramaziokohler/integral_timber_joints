from termcolor import cprint
from copy import copy
from compas.geometry import Transformation, Frame
from compas.robots.model import joint, tool

from integral_timber_joints.process.state import ObjectState, SceneState, copy_state_dict
from integral_timber_joints.planning.visualization import color_from_object_id
from integral_timber_joints.process.action import LoadBeamAction, PickGripperFromStorageAction, \
    PickBeamWithGripperAction, PickClampFromStorageAction, PlaceClampToStructureAction, BeamPlacementWithClampsAction, \
    PlaceGripperToStorageAction, PlaceClampToStorageAction, PickClampFromStructureAction, BeamPlacementWithoutClampsAction
from integral_timber_joints.process.dependency import ComputationalResult

def actions_from_pddlstream_plan(process, plan, verbose=False):
    place_actions = [action for action in plan if action.name == 'place_element_on_structure']
    beam_sequence = [ac.args[0] for ac in place_actions]
    assert beam_sequence == process.assembly.sequence, 'Don\'t support auto-plan assembly sequence for now!'

    beam_id = last_beam_id = ''
    seq_n = 0
    acts = []
    for action in plan:
        print('='*10)
        cprint(action, 'cyan')
        if action.name == 'pick_element_from_rack':
            # TODO screwdriver needs special care here
            beam_id = action.args[0]
            gripper_id = action.args[-1]
            gripper = process.gripper(gripper_id)
            # * double-check tool type consistency
            gt_gripper_type = process.assembly.get_beam_attribute(beam_id, "gripper_type")
            # assert gt_gripper_type == gripper.type_name, '{} should use gripper with type {} but {} with type {} assigned.'.format(beam_id, gt_gripper_type, gripper.name, gripper.type_name)

            if gt_gripper_type == gripper.type_name:
                '{} should use gripper with type {} but {} with type {} assigned.'.format(beam_id, gt_gripper_type, gripper.name, gripper.type_name)

            acts.append(LoadBeamAction(seq_n, 0, beam_id))
            acts.append(PickBeamWithGripperAction(seq_n, 0, beam_id, gripper_id))

        elif action.name == 'place_element_on_structure':
            beam_id = action.args[0]
            gripper_id = action.args[-1]
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
                act = BeamPlacementWithoutClampsAction(seq_n, 0, beam_id, gripper_id)
            else:
                # ! beams with clamp
                act = BeamPlacementWithClampsAction(seq_n, 0, beam_id, joint_id_of_clamps, gripper_id, clamp_ids)
            acts.append(act)

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

        elif action.name == 'pick_gripper_from_rack':
            gripper_id = action.args[0]
            gripper = process.gripper(gripper_id)
            acts.append(PickGripperFromStorageAction(seq_n, 0, gripper.type_name, gripper_id))

        elif action.name == 'pick_clamp_from_rack':
            clamp_id = action.args[0]
            clamp = process.clamp(clamp_id)
            acts.append(PickClampFromStorageAction(seq_n, 0, clamp.type_name, clamp_id))

        elif action.name == 'place_tool_at_rack':
            tool_id = action.args[0]
            if tool_id.startswith('g'):
                gripper = process.gripper(tool_id)
                act = PlaceGripperToStorageAction(seq_n, 0, gripper.type_name, tool_id)
            elif tool_id.startswith('c'):
                clamp = process.clamp(tool_id)
                act = PlaceClampToStorageAction(tool_type=clamp.type_name, tool_id=tool_id)
            else:
                raise ValueError('Weird tool id {}'.format(tool_id))
            acts.append(act)

        elif action.name == 'pick_clamp_from_joint':
            clamp_id = action.args[0]
            clamp = process.clamp(clamp_id)
            joint_id = (action.args[1], action.args[2])
            # ! convention: sequence id smaller first
            if process.assembly.sequence.index(joint_id[0]) > process.assembly.sequence.index(joint_id[1]):
                joint_id = joint_id[::-1]
            acts.append(PickClampFromStructureAction(joint_id=joint_id, tool_type=clamp.type_name, tool_id=clamp_id))

        elif action.name == 'place_clamp_at_joint':
            clamp_id = action.args[0]
            clamp = process.clamp(clamp_id)
            joint_id = (action.args[1], action.args[2])
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
            acts.append(PlaceClampToStructureAction(seq_n, 0, joint_id, clamp.type_name, clamp_id))

        elif action.name == 'move':
            pass

    # * last beam and tool collection
    assert beam_id == beam_sequence[-1] and len(acts) > 0
    if verbose:
        print('='*10)
        print('Beam id: {}'.format(beam_id))
        for act in acts:
            print('|- ' + act.__str__())
    process.assembly.set_beam_attribute(beam_id, 'actions', acts)

    # * trigger movement computation
    for action in process.actions:
        action.create_movements(process)
        action.assign_movement_ids()
        for mov_n, movement in enumerate(action.movements):
            # if verbose:
            #     print("Processing Seq %i Action %i Movement %i: %s" % (action.seq_n, action.act_n, mov_n, movement.tag))
            movement.create_state_diff(process)
    return ComputationalResult.ValidCanContinue