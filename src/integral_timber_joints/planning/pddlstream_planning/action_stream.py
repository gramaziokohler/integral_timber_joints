from typing import List
from matplotlib.pyplot import isinteractive
from termcolor import colored
from collections import namedtuple
import time
from itertools import product, chain
from termcolor import cprint
import pybullet_planning as pp
from copy import copy

from compas_fab.robots import Configuration, Robot
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose, pose_from_transformation
from compas_fab_pychoreo.utils import values_as_list
from compas.robots import Configuration
from compas_fab.robots import Trajectory
from compas.geometry import Transformation, Frame

from integral_timber_joints.planning.robot_setup import GANTRY_ARM_GROUP
from integral_timber_joints.planning.state import set_state, gantry_base_generator
from integral_timber_joints.planning.stream import _get_sample_bare_arm_ik_fn
from integral_timber_joints.planning.utils import LOGGER

from integral_timber_joints.process import RobotClampAssemblyProcess, RoboticMovement, RoboticFreeMovement, SceneState, Action
from integral_timber_joints.process.movement import Movement
from .utils import ITJ_ACTION_CLASS_FROM_PDDL_ACTION_NAME, print_fluents

from integral_timber_joints.process.action import OperatorAttachScrewdriverAction

######################################

JOINT_KEY = lambda j: j[0]+','+j[1]

def draw_state_frames(scene: SceneState):
    handles = []
    with pp.LockRenderer():
        for object_key in scene.object_keys:
            if scene[object_key] is not None and isinstance(scene[object_key], Frame):
                handles.extend(pp.draw_pose(pose_from_frame(scene[object_key], scale=1e-3)))
    # pp.wait_if_gui('Viz state object frames')
    with pp.LockRenderer():
        pp.remove_handles(handles)
    return handles

def assign_fluent_state(client: PyChoreoClient, robot: Robot,
        process: RobotClampAssemblyProcess, fluents: List):
    # * manually create a state according to the fluent facts and set_state
    state = SceneState(process)
    #.from_data(process.initial_state.data)

    # ! the initial_state's key got converted to list for some reasons
    for k in process.initial_state.object_keys:
        state[tuple(k)] = copy(process.initial_state[tuple(k)])
    state[process.robot_config_key] = None

    # # by default all the attachment status is None (False)
    # for oid in chain(process.assembly.sequence, process.tool_ids):
    #     state[(oid, 'a')] = False
    # # * Tool changer
    # state[('tool_changer', 'a')] = True
    # state[('tool_changer', 'g')] = Transformation.from_frame(Frame.worldXY())

    # assigned_tool_from_joints = {}
    attached_object_grasps = {}
    for fluent in fluents:
        name, args = fluent[0], fluent[1:]
        # print(fluent)
        if name == 'atpose':
            # (AtPose ?object ?pose)
            object_name, frame = args
            state[(object_name, 'f')] = frame
        elif name == 'attached':
            # (Attached ?object ?grasp)
            object_name, grasp_transform = args
            state[(object_name, 'a')] = True
            state[(object_name, 'g')] = grasp_transform
            attached_object_grasps[object_name] = grasp_transform
        elif name == 'toolatjoint':
            pass
        elif name == 'assembled':
            # ! force at goal pose, but this shouldn't be needed
            # ? for some reasons the beam's AtPose is not in fluents once assembled
            element = args[0]
            f_world_from_beam_final = process.assembly.get_beam_attribute(element, 'assembly_wcf_final')
            state[(element, 'f')] = f_world_from_beam_final
            # pass
        else:
            raise ValueError(name)

    if not set_state(client, robot, process, state):
        LOGGER.debug('assign_fluent_state: set state error.')
        raise RuntimeError()

    # LOGGER.debug('Set fluent state')
    # draw_state_frames(state)

    return state # , assigned_tool_from_joints

def apply_movement_state_diff(scene: SceneState, movements: List[Movement], debug=False):
    for key in scene.object_keys:
        for m in movements[::-1]:
            if key in m.state_diff:
                scene[key] = m.state_diff[key]
                # if debug:
                #     LOGGER.debug('{}-{}'.format(colored(key, 'blue'), m.state_diff[key]))
                break # ! break movement loop
    return scene

###########################################

def sample_ik_for_action(client: PyChoreoClient, robot: Robot,
        process: RobotClampAssemblyProcess,
        action: Action, fluents=[], prev_actions=[], options=None):

    debug = options.get('debug', False)
    verbose = options.get('verbose', False)
    gantry_attempts = options.get('gantry_attempts', 50)
    reachable_range = options.get('reachable_range', (0.2, 2.4))

    # * compute movements from action
    action.create_movements(process)

    # * create start state scene for the action based on fluents
    action_initial_scene = assign_fluent_state(client, robot, process, fluents)
    end_scene = action_initial_scene.copy()

    # * extra previous action state propagation for some bundled actions
    for prev_action in prev_actions:
        prev_action.create_movements(process)
        for m in prev_action.movements:
            m.create_state_diff(process)
        end_scene = apply_movement_state_diff(end_scene, prev_action.movements, debug=debug)

    with pp.WorldSaver():
        for i, movement in enumerate(action.movements):
            if debug:
                LOGGER.debug('='*5)
                LOGGER.debug('{} - {}'.format(i, movement.short_summary))

            # * trigger state diff computation
            movement.create_state_diff(process)

            # * updating end_scene for the next movement
            # this happens before set scene because we want to perform check_attachment_collisions
            # on the end scene
            end_scene = apply_movement_state_diff(end_scene, [movement], debug=debug)

            # * apply movement state_diff on the end_scene from the current movement
            # * set start state
            if not set_state(client, robot, process, end_scene, options=options):
                LOGGER.debug('sample_ik_for_action: set state error.')
                raise RuntimeError()
                # return None

            if debug:
                LOGGER.debug(f'start state of {movement.short_summary}')
                client._print_object_summary()
                draw_state_frames(end_scene)

            # * skip non-robotic movements for IK computation
            if not isinstance(movement, RoboticMovement):
               continue

            # * skip if there exists a taught conf
            if movement.target_configuration is not None:
                continue

            from integral_timber_joints.process.action import BeamPlacementWithClampsAction, AssembleBeamWithScrewdriversAction
            if i == 0: # and isinstance(action, AssembleBeamWithScrewdriversAction):
                pp.wait_if_gui()

            # * compute IK
            if movement.target_frame is None:
                LOGGER.warning(f'Target frame is None in {movement.short_summary}')
                # raise some serious error
            sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
            gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
            gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)

            # * ACM setup
            temp_name = '_tmp'
            for o1_name, o2_name in movement.allowed_collision_matrix:
                o1_bodies = client._get_bodies('^{}$'.format(o1_name))
                o2_bodies = client._get_bodies('^{}$'.format(o2_name))
                for parent_body, child_body in product(o1_bodies, o2_bodies):
                    client.extra_disabled_collision_links[temp_name].add(
                        ((parent_body, None), (child_body, None))
                    )

            end_t0cf_frame = movement.target_frame.copy()
            end_t0cf_frame.point *= 1e-3
            sample_found = False

            # * check collisions among the list of attached objects and obstacles in the scene.
            # This includes collisions between:
            #     - each pair of (attached object, obstacle)
            #     - each pair of (attached object 1, attached object 2)
            # attach_options = options.copy()
            # attach_options['diagnosis'] = False
            if client.check_attachment_collisions(options):
               LOGGER.debug('Stream sample fails: tool attachments collision')
            #    pp.wait_if_gui()
               return None

            gantry_base_gen_fn = gantry_base_generator(client, robot, end_t0cf_frame, reachable_range=reachable_range, scale=1.0, options=options)

            for gantry_iter, base_conf in zip(range(gantry_attempts), gantry_base_gen_fn):
                # * bare-arm IK sampler
                arm_conf_vals = sample_ik_fn(pose_from_frame(end_t0cf_frame, scale=1))
                # * iterate through all 6-axis IK solution
                for arm_conf_val in arm_conf_vals:
                    if arm_conf_val is None:
                        continue
                    full_conf = Configuration(list(base_conf.joint_values) + list(arm_conf_val),
                        gantry_arm_joint_types, gantry_arm_joint_names)
                    if not client.check_collisions(robot, full_conf, options=options):
                        sample_found = True
                        LOGGER.debug('IK sample found after {} gantry iters.'.format(gantry_iter))
                        # if debug:
                            # pp.wait_if_gui('IK Conf found.')
                        break
                if sample_found:
                    break

            # ! reset ACM
            if temp_name in client.extra_disabled_collision_links:
                del client.extra_disabled_collision_links[temp_name]

            # ! return None if one of the movement cannot find an IK solution
            if not sample_found:
                LOGGER.debug('No robot IK conf can be found for {} after {} attempts.'.format(
                    movement.short_summary, gantry_attempts))
                return None

    return (action,)

##########################################

def get_action_ik_fn(client: PyChoreoClient, robot: Robot, process: RobotClampAssemblyProcess, action_name: str, options=None):
    debug = options.get('debug', False)
    action_class = ITJ_ACTION_CLASS_FROM_PDDL_ACTION_NAME[action_name]
    beam_seq = process.assembly.sequence

    if action_name == 'place_clamp_to_structure':
        def ik_fn(tool_id: str, element1: str, element2: str, fluents=[]):
            if debug:
                print_fluents(fluents)
            if ('assembled', element1) not in fluents:
                return None
            tool = process.clamp(tool_id)
            action = action_class(joint_id=(element1, element2), tool_type=tool.type_name, tool_id=tool_id)
            return sample_ik_for_action(client, robot, process, action, fluents, options=options)

    elif action_name == 'pick_clamp_from_structure':
        def ik_fn(tool_id: str, element1: str, element2: str, fluents=[]):
            if debug:
                print_fluents(fluents)
            if ('assembled', element1) not in fluents or ('assembled', element2) not in fluents:
                return None
            tool = process.clamp(tool_id)
            action = action_class(joint_id=(element1, element2), tool_type=tool.type_name, tool_id=tool_id)
            return sample_ik_for_action(client, robot, process, action, fluents, options=options)

    elif action_name == 'beam_placement_without_clamp':
        def ik_fn(tool_id: str, element: str, fluents=[]):
            if debug:
                print_fluents(fluents)

            seq_id = beam_seq.index(element)
            if seq_id > 0:
                prev_element = beam_seq[seq_id - 1]
                if ('assembled', prev_element) not in fluents:
                    return None

            action = action_class(beam_id=element, gripper_id=tool_id)
            return sample_ik_for_action(client, robot, process, action, fluents, options=options)

    elif action_name == 'beam_placement_with_clamps':
        def ik_fn(tool_id: str, element: str, fluents=[]):
            if debug:
                print_fluents(fluents)

            prev_element = beam_seq[beam_seq.index(element) - 1]
            if ('assembled', prev_element) not in fluents:
                return None

            joint_ids = process.assembly.get_joint_ids_with_tools_for_beam(element)
            clamp_ids = []
            for joint in joint_ids:
                assert joint[1] == element
                if ('assembled', joint[0]) in fluents:
                    for fluent in fluents:
                        if fluent[0] == 'toolatjoint' and fluent[2] == joint[0] and fluent[3] == joint[1] and \
                            ('assembled', fluent[4]) in fluents:
                            clamp_ids.append(fluent[1])
                            break
                    else:
                        # A joint is not occupied by a clamp
                        return None
            if len(clamp_ids) == 0:
                return None

            action = action_class(beam_id=element, joint_ids=joint_ids, gripper_id=tool_id, clamp_ids=clamp_ids)
            return sample_ik_for_action(client, robot, process, action, fluents, options=options)

    elif action_name == 'assemble_beam_with_screwdrivers':
        # * with_gripper and without_gripper share the same IK fn
        def ik_fn(gripper_id: str, element: str, fluents=[]):
            if debug:
                print_fluents(fluents)

            prev_element = beam_seq[beam_seq.index(element) - 1]
            if ('assembled', prev_element) not in fluents:
                return None

            joint_ids = process.assembly.get_joint_ids_with_tools_for_beam(element)
            # PDDL doesn't mess with screwdriver assignment
            tool_ids = [process.assembly.get_joint_attribute(joint_id, 'tool_id') for joint_id in joint_ids]

            # * extra previous action to set the screwdriver (not-gripper ones) attached state
            prev_actions = []
            for joint_id, tool_id in zip(joint_ids, tool_ids):
                if tool_id == gripper_id:
                    continue  # Skipping the screwdriver that is acting as gripper
                tool_type = process.assembly.get_joint_attribute(joint_id, 'tool_type')
                prev_actions.append(OperatorAttachScrewdriverAction(beam_id=element, joint_id=joint_id, 
                    tool_type=tool_type, tool_id=tool_id, beam_position='assembly_wcf_screwdriver_attachment_pose'))

            action = action_class(beam_id=element, joint_ids=joint_ids, gripper_id=gripper_id, screwdriver_ids=tool_ids)
            return sample_ik_for_action(client, robot, process, action, fluents, prev_actions=prev_actions, options=options)

    else:
        raise NotImplementedError('stream sample function {} not implemented!'.format(action_name))

    return ik_fn

