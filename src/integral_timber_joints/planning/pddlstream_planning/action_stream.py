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

    handles = []
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
        elif name == 'toolassignedtojoint':
            pass
        #     #  (ToolAssignedToJoint ?element1 ?element2 ?tool)
        #     e1, e2, tool = args
        #     assigned_tool_from_joints[JOINT_KEY((e1,e2))] = tool
        elif name == 'assembled':
            pass
        else:
            raise ValueError(name)

    # ! update attached object frame according to robot flange frame and grasp
    # flange_frame = state[('robot', 'f')]
    # for object_name, grasp_transform in attached_object_grasps.items():
    #     robot_flange_from_attached_obj = grasp_transform
    #     t_world_robot = Transformation.from_frame(flange_frame)
    #     t_world_from_object = t_world_robot * robot_flange_from_attached_obj
    #     object_frame = Frame.from_transformation(t_world_from_object)
    #     state[(object_name, 'f')] = object_frame
        # client.set_object_frame('^{}$'.format(object_name), object_frame)

    if not set_state(client, robot, process, state):
        LOGGER.debug('assign_fluent_state: set state error.')
        raise RuntimeError()

    # # ! update attachment pose since set_state woulnd't do that if
    # # the current object pose and robot flange pose are not close
    # robot_flange_pose = pose_from_frame(state[('robot', 'f')], scale=1e-3)
    # attachments = values_as_list(client.pychoreo_attachments)
    # for attachment in attachments:
    #     # attachment.assign()
    #     child_pose = pp.body_from_end_effector(robot_flange_pose, attachment.grasp_pose)
    #     pp.set_pose(attachment.child, child_pose)

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
        action: Action, fluents=[], options=None):

    debug = options.get('debug', False)
    verbose = options.get('verbose', False)
    gantry_attempts = options.get('gantry_attempts', 50)
    reachable_range = options.get('reachable_range', (0.2, 2.4))

    # * compute movements from action
    action.create_movements(process)

    with pp.WorldSaver():
        # * create start state scene for the action based on fluents
        action_initial_scene = assign_fluent_state(client, robot, process, fluents)
        end_scene = action_initial_scene.copy()

        for i, movement in enumerate(action.movements):
            if debug:
                LOGGER.debug('='*5)
                LOGGER.debug('{} - {}'.format(i, movement.short_summary))

            # * trigger state diff computation
            movement.create_state_diff(process)

            # * updating end_scene for the next movement
            end_scene = apply_movement_state_diff(end_scene, [action.movements[i]], debug=debug)

            # * apply movement state_diff on the end_scene from the last movement
            # * set start state
            if not set_state(client, robot, process, end_scene, options=options):
                LOGGER.debug('sample_ik_for_action: set state error.')
                raise RuntimeError()
                # return None

            # * skip non-robotic movements for IK computation
            if not isinstance(movement, RoboticMovement):
               continue

            # * skip if there exists a taught conf
            if movement.target_configuration is not None:
                continue

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

            # if debug:
                # LOGGER.debug(f'start state of {movement.short_summary}')
                # client._print_object_summary()
                # draw_state_frames(end_scene)

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

    # cprint('sample pick element succeeds for {}|{}.'.format(obj_name, tool_name), 'green')
    return (action,)

##########################################

def get_action_ik_fn(client: PyChoreoClient, robot: Robot, process: RobotClampAssemblyProcess, action_name: str, options=None):
    debug = options.get('debug', False)
    action_class = ITJ_ACTION_CLASS_FROM_PDDL_ACTION_NAME[action_name]

    if action_name == 'place_clamp_to_structure':
        def ik_fn(tool_id: str, element1: str, element2: str,
            fluents=[]):
            if debug:
                print_fluents(fluents)
            if ('assembled', element1) not in fluents:
                return None
            tool = process.clamp(tool_id)
            action = action_class(joint_id=(element1, element2), tool_type=tool.type_name, tool_id=tool_id)
            return sample_ik_for_action(client, robot, process, action, fluents, options=options)
    if action_name == 'pick_clamp_from_structure':
        def ik_fn(tool_id: str, element1: str, element2: str,
            fluents=[]):
            if debug:
                print_fluents(fluents)
            if ('assembled', element1) not in fluents or ('assembled', element2) not in fluents:
                return None
            tool = process.clamp(tool_id)
            action = action_class(joint_id=(element1, element2), tool_type=tool.type_name, tool_id=tool_id)
            return sample_ik_for_action(client, robot, process, action, fluents, options=options)

    return ik_fn

