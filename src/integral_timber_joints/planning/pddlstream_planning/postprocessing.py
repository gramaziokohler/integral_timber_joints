from termcolor import cprint, colored
from copy import copy

from compas.geometry import Transformation, Frame
from compas_fab.robots import Trajectory
from compas.robots.configuration import Configuration

from integral_timber_joints.process import RobotClampAssemblyProcess, RobotAction

from integral_timber_joints.process.state import ObjectState, SceneState, copy_state_dict
from integral_timber_joints.planning.parsing import parse_process, save_process, get_process_path
from integral_timber_joints.process.action import LoadBeamAction, PickGripperFromStorageAction, PickBeamWithGripperAction, PickClampFromStorageAction, PlaceClampToStructureAction, BeamPlacementWithClampsAction, PlaceGripperToStorageAction, PlaceClampToStorageAction, PickClampFromStructureAction, BeamPlacementWithoutClampsAction, AssembleBeamWithScrewdriversAction,  RetractGripperFromBeamAction, PickScrewdriverFromStorageAction, PlaceScrewdriverToStorageAction, ManaulAssemblyAction, OperatorAttachScrewdriverAction, DockWithScrewdriverAction, RetractScrewdriverFromBeamAction, PickAndRotateBeamForAttachingScrewdriverAction

from integral_timber_joints.assembly.beam_assembly_method import BeamAssemblyMethod

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticMovement, RoboticClampSyncLinearMovement, RobotScrewdriverSyncLinearMovement

from integral_timber_joints.process.dependency import ComputationalResult

##############################################

def _create_bundled_actions_for_screwed(process, beam_id, gripper_id, verbose=False):
    assembly = process.assembly
    actions = []
    assembly_method = process.assembly.get_assembly_method(beam_id)

    # * Lift Beam and Rotate , Operator Attach Screwdriver
    gripper = process.gripper(gripper_id)
    gripper_type = gripper.type_name
    joint_ids = list(assembly.get_joint_ids_with_tools_for_beam(beam_id))
    tool_ids = [assembly.get_joint_attribute(joint_id, 'tool_id') for joint_id in joint_ids]

    act = PickAndRotateBeamForAttachingScrewdriverAction(beam_id=beam_id, gripper_id=gripper_id)
    actions.append(act)

    for joint_id, tool_id in zip(joint_ids, tool_ids):
        if tool_id == gripper_id:
            continue  # Skipping the screwdriver that is acting as gripper
        tool_type = assembly.get_joint_attribute(joint_id, 'tool_type')
        actions.append(OperatorAttachScrewdriverAction(beam_id=beam_id, joint_id=joint_id, tool_type=tool_type, tool_id=tool_id,
            beam_position='assembly_wcf_screwdriver_attachment_pose'))

    # * Actions to Assemble
    if assembly_method == BeamAssemblyMethod.SCREWED_WITH_GRIPPER:

        # * Action to Place Beam and Screw it with Screwdriver, not including retract
        act = AssembleBeamWithScrewdriversAction(beam_id=beam_id, joint_ids=joint_ids, gripper_id=gripper_id, screwdriver_ids=tool_ids)
        actions.append(act)

        # * Action to Open Gripper and Retract Gripper
        act = RetractGripperFromBeamAction(beam_id=beam_id, gripper_id=gripper_id, additional_attached_objects=tool_ids)
        actions.append(act)

        # * Action to Place Gripper back to Storage
        act = PlaceGripperToStorageAction(tool_type=gripper_type, tool_id=gripper_id)
        actions.append(act)
    else:
        raise NotImplementedError(assembly_method)

    # * Actions to Detach Remaining Screwdriver from the Structure.
    for joint_id, tool_id in reversed(list(zip(joint_ids, tool_ids))):
        tool_type = assembly.get_joint_attribute(joint_id, 'tool_type')

        # * Action to Dock with Screwdriver at Storage
        actions.append(DockWithScrewdriverAction(joint_id=joint_id, tool_position='screwdriver_assembled_attached', tool_type=tool_type, tool_id=tool_id))

        # * Action to retract Screwdriver and place it to storage
        actions.append(RetractScrewdriverFromBeamAction(beam_id=beam_id, joint_id=joint_id, tool_id=tool_id))

        actions.append(PlaceScrewdriverToStorageAction(tool_type=tool_type, tool_id=tool_id))

    # Print out newly added actions and return
    if verbose:
        for act in actions:
            print('|- ' + act.__str__())

    return actions

# def assign_ik_conf_to_action(process, action, conf):
#     # * trigger movement computation
#     action.create_movements(process)
#     action.assign_movement_ids()
#     for movement in action.movements:
#         movement.create_state_diff(process)
#     for mov_n, movement in enumerate(action.movements):
#         # if verbose:
#         #     print("Processing Seq %i Action %i Movement %i: %s" % (action.seq_n, action.act_n, mov_n, movement.tag))
#         if (isinstance(movement, RoboticLinearMovement) and 'Advance' in movement.tag) or \
#             isinstance(movement, RoboticClampSyncLinearMovement):
#             if isinstance(conf, Configuration):
#                 process.set_movement_end_robot_config(movement, conf)
#     return action

# def assign_trajs_to_action(process, action, command):
#     # * trigger movement computation
#     action.create_movements(process)
#     action.assign_movement_ids()
#     for movement in action.movements:
#         movement.create_state_diff(process)
#     assert len(action.movements) == len(command.state_diffs)
#     assert len(action.movements) == len(command.trajs)
#     for mov_n, (movement, state_diff, traj) in enumerate(zip(action.movements, command.state_diffs, command.trajs)):
#         if mov_n == 0:
#             for key in command.start_state.object_keys:
#                 if command.start_state[key] is not None:
#                     action.movements[0].state_diff[key] = command.start_state[key]
#         movement.state_diff.update(state_diff)
#         if isinstance(movement, RoboticMovement) and traj is not None:
#             movement.trajectory = traj
#             process.set_movement_end_robot_config(movement, traj.points[-1])
#     return action

def action_compute_movements(process: RobotClampAssemblyProcess, action: RobotAction):
    action.create_movements(process)
    for movement in action.movements:
        movement.create_state_diff(process)
    return action

##############################################

def save_pddlstream_plan_to_itj_process(process: RobotClampAssemblyProcess, plan, design_dir, problem_name, symbolic=False, save_subdir='results', verbose=False):
    """
        design_dir = '211010_CantiBox'
        problem_name = 'CantiBoxLeft_process.json'
    """
    # TODO if symbolic create action here otherwise just parse action from plan action arguments

    beam_id = last_beam_id = ''
    seq_n = 0
    act_n = 0
    new_seq = []
    acts = []

    for pddl_action in plan:
        itj_act = None
        if pddl_action.name == 'pick_beam_with_gripper':
            beam_id = pddl_action.args[0]
            gripper_id = pddl_action.args[3]
            gripper = process.gripper(gripper_id)
            # * double-check tool type consistency
            gt_gripper_type = process.assembly.get_beam_attribute(beam_id, "gripper_type")
            assert gt_gripper_type == gripper.type_name, '{} should use gripper with type {} but {} with type {} assigned.'.format(beam_id, gt_gripper_type, gripper.name, gripper.type_name)

            itj_act = PickBeamWithGripperAction(seq_n, 0, beam_id, gripper_id)

        elif pddl_action.name == 'beam_placement_with_clamps' or \
            pddl_action.name == 'beam_placement_without_clamp':
            # ! :parameters (?element ?e_grasp ?tool ?tool_grasp)
            beam_id = pddl_action.args[0]
            gripper_id = pddl_action.args[2]
            gripper = process.gripper(gripper_id)
            # * double-check tool type consistency
            gt_gripper_type = process.assembly.get_beam_attribute(beam_id, "gripper_type")
            assert gt_gripper_type == gripper.type_name, '{} should use gripper with type {} but {} with type {} assigned.'.format(beam_id, gt_gripper_type, gripper.name, gripper.type_name)

            involved_joint_ids = list(process.assembly.get_joint_ids_with_tools_for_beam(beam_id))
            # ! these assigned tools should have been updated by previous place clamp actions already
            joint_tool_ids = [process.assembly.get_joint_attribute(joint_id, 'tool_id') for joint_id in involved_joint_ids]

            if pddl_action.name == 'beam_placement_without_clamp':
                itj_act = BeamPlacementWithoutClampsAction(seq_n, 0, beam_id, gripper_id)
            elif pddl_action.name == 'beam_placement_with_clamps':
                itj_act = BeamPlacementWithClampsAction(seq_n, 0, beam_id, involved_joint_ids, gripper_id, joint_tool_ids)

        elif pddl_action.name == 'assemble_beam_with_screwdrivers_and_gripper_at_rack':
            beam_id = pddl_action.args[0]
            gripper_id = pddl_action.args[-3]
            # ! screwdriver actions bundled to enforce immediate returnning of screwdrivers to racks
            itj_act = _create_bundled_actions_for_screwed(process, beam_id, gripper_id, verbose=verbose)

        elif pddl_action.name == 'retract_gripper_from_beam':
            # ! :parameters (?element ?e_grasp ?e_pose ?tool ?tool_grasp)
            beam_id = pddl_action.args[0]
            gripper_id = pddl_action.args[3]
            itj_act = RetractGripperFromBeamAction(beam_id=beam_id, gripper_id=gripper_id)

        elif pddl_action.name == 'operator_load_beam':
            beam_id = pddl_action.args[0]
            itj_act = LoadBeamAction(seq_n, 0, beam_id)

        elif pddl_action.name == 'manaul_assemble_scaffold':
            beam_id = pddl_action.args[0]
            itj_act = ManaulAssemblyAction(seq_n, 0, beam_id)

        elif pddl_action.name == 'pick_tool_from_rack':
            tool_id = pddl_action.args[0]
            if tool_id.startswith('g'):
                gripper = process.gripper(tool_id)
                itj_act = PickGripperFromStorageAction(seq_n, 0, gripper.type_name, tool_id)
            elif tool_id.startswith('c'):
                clamp = process.clamp(tool_id)
                itj_act = PickClampFromStorageAction(seq_n, 0, clamp.type_name, tool_id)
            elif tool_id.startswith('s'):
                screwdriver = process.screwdriver(tool_id)
                itj_act = PickScrewdriverFromStorageAction(seq_n, 0, screwdriver.type_name, tool_id)
            else:
                raise ValueError('Weird tool id {}'.format(tool_id))

        elif pddl_action.name == 'place_tool_at_rack':
            tool_id = pddl_action.args[0]
            if tool_id.startswith('g'):
                gripper = process.gripper(tool_id)
                itj_act = PlaceGripperToStorageAction(seq_n, 0, gripper.type_name, tool_id)
            elif tool_id.startswith('c'):
                clamp = process.clamp(tool_id)
                itj_act = PlaceClampToStorageAction(tool_type=clamp.type_name, tool_id=tool_id)
            elif tool_id.startswith('s'):
                screwdriver = process.screwdriver(tool_id)
                itj_act = PlaceScrewdriverToStorageAction(seq_n, 0, screwdriver.type_name, tool_id)
            else:
                raise ValueError('Weird tool id {}'.format(tool_id))

        elif pddl_action.name == 'place_clamp_to_structure':
            # ! :parameters (?tool ?pose ?grasp ?element1 ?element2 ?action)
            clamp_id = pddl_action.args[0]
            clamp = process.clamp(clamp_id)
            joint_id = (pddl_action.args[3], pddl_action.args[4])
            # ! convention: sequence id smaller first
            assert process.assembly.sequence.index(joint_id[0]) < process.assembly.sequence.index(joint_id[1])
            # * double-check clamp type consistency
            gt_clamp_type = process.assembly.get_joint_attribute(joint_id, 'tool_type')
            assert gt_clamp_type == clamp.type_name, 'Joint {} should use clamp with type {} but {} with type {} assigned.'.format(joint_id, gt_clamp_type, clamp.name, clamp.type_name)

            # * clamp assignment to beam
            process.assembly.set_joint_attribute(joint_id, 'tool_id', clamp_id)
            itj_act = PlaceClampToStructureAction(seq_n, 0, joint_id, clamp.type_name, clamp_id)

        elif pddl_action.name == 'pick_clamp_from_structure':
            # ! :parameters (?tool ?pose ?grasp ?element1 ?element2 ?action)
            clamp_id = pddl_action.args[0]
            clamp = process.clamp(clamp_id)
            joint_id = (pddl_action.args[3], pddl_action.args[4])
            assert process.assembly.sequence.index(joint_id[0]) < process.assembly.sequence.index(joint_id[1])
            itj_act = PickClampFromStructureAction(joint_id=joint_id, tool_type=clamp.type_name, tool_id=clamp_id)

        else:
            raise ValueError(pddl_action.name)
            # pass

        assert itj_act is not None, 'Action creation failed for {}'.format(pddl_action.name)
        if not isinstance(itj_act, list):
            itj_act = [itj_act]
        # * Action seq_n, act_n assignment
        for ac in itj_act:
            ac = action_compute_movements(process, ac)
            ac.act_n = act_n
            ac.seq_n = seq_n
            ac.assign_movement_ids()
            acts.append(ac)
            # ! bump act_n
            act_n += 1

        # separate actions and assigned to beam (grouping for visualization purposes)
        if 'beam_placement' in pddl_action.name or 'assemble_beam' in pddl_action.name or \
            'manaul_assemble_scaffold' in pddl_action.name:
            assert len(acts) > 0
            # if beam is not the same as last one, or is not the last beam
            # assign actions to beam and move on to the next one
            if beam_id != last_beam_id and beam_id != process.assembly.sequence[-1]:
                if verbose:
                    print('='*10)
                    print('Beam id: {}'.format(beam_id))
                    for act in acts:
                        print('|- ' + act.__str__())
                process.assembly.set_beam_attribute(beam_id, 'actions', acts)
                new_seq.append(beam_id)
                last_beam_id = beam_id
                # ! bump seq_n
                seq_n += 1
                acts = []

    # * last beam and tool collection
    assert new_seq == process.assembly.sequence[:-1]
    assert len(new_seq) == len(process.assembly.sequence)-1 and len(acts) > 0
    if verbose:
        print('='*10)
        print('Beam id: {}'.format(beam_id))
        for act in acts:
            print('|- ' + act.__str__())
    process.assembly.set_beam_attribute(beam_id, 'actions', acts)

    # * write to file
    save_process(design_dir, problem_name, process, save_dir=save_subdir)
