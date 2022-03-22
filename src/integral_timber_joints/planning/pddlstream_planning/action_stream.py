from termcolor import colored
from collections import namedtuple
import time
from itertools import product, chain
from termcolor import cprint
import pybullet_planning as pp

from compas_fab.robots import Configuration, Robot
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose, pose_from_transformation
from compas.robots import Configuration
from compas_fab.robots import Trajectory

from integral_timber_joints.planning.robot_setup import GANTRY_ARM_GROUP
from integral_timber_joints.planning.state import set_state, gantry_base_generator
from integral_timber_joints.planning.stream import _get_sample_bare_arm_ik_fn
from integral_timber_joints.planning.utils import LOGGER

from integral_timber_joints.process import RobotClampAssemblyProcess, RoboticMovement, RoboticFreeMovement, SceneState, Action

######################################

JOINT_KEY = lambda j: j[0]+','+j[1]

def assign_fluent_state(client: PyChoreoClient, robot: Robot,
        process: RobotClampAssemblyProcess, fluents):
    # * manually create a state according to the fluent facts and set_state
    state = SceneState(process)
    # by default all the attachment status is None (False)
    for oid in chain(process.assembly.sequence, process.tool_ids):
        state[(oid, 'a')] = False

    assigned_tool_from_joints = {}
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
        elif name == 'toolassignedtojoint':
            #  (ToolAssignedToJoint ?element1 ?element2 ?tool)
            e1, e2, tool = args
            assigned_tool_from_joints[JOINT_KEY((e1,e2))] = tool
        else:
            raise ValueError(name)
    set_state(client, robot, process, state)
    return state, assigned_tool_from_joints

def apply_movement_state_diff(scene: SceneState, movements, debug=False):
    for key in scene.object_keys:
        for m in movements[::-1]:
            if key in m.state_diff:
                scene[key] = m.state_diff[key]
                if debug:
                    print(colored(key, 'blue' if key[0]=='robot' else 'grey'), m.state_diff[key])
                break # ! break movement loop
    return scene

###########################################

def sample_ik_for_action(client: PyChoreoClient, robot: Robot,
        process: RobotClampAssemblyProcess,
        itj_action: Action, fluents, options=None):

    debug = options.get('debug', False)
    verbose = options.get('verbose', False)
    gantry_attempts = options.get('gantry_attempts', 50)
    reachable_range = options.get('reachable_range', (0.2, 2.4))

    # * create start state scene for the action based on fluents
    _action_scene, assigned_tool_from_joints = assign_fluent_state(client, robot, process, fluents)

    itj_action.create_movements(process)
    end_scene = _action_scene.copy()
    for i, movement in enumerate(itj_action.movements):
        if debug:
            LOGGER.debug('='*5)
            LOGGER.debug('{} - {}'.format(i, movement.short_summary))

        movement.create_state_diff(process)
        # * apply movement state_diff on the end_scene from the last movement
        start_scene = end_scene.copy()
        end_scene = apply_movement_state_diff(end_scene, [itj_action.movements[i]], debug=debug)

        # * set start state
        if not set_state(client, robot, process, start_scene, options=options):
            LOGGER.error('Compute linear movement: set start state error.')
            return None

        # * skip free and non-robotic movements
        if not isinstance(movement, RoboticMovement) or \
           isinstance(movement, RoboticFreeMovement):
           continue
        if end_scene[('robot', 'f')] is None:
            LOGGER.warning(f'Target frame is None in {movement.short_summary}')

        # * compute IK
        sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
        gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
        gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)

        end_t0cf_frame = end_scene[('robot', 'f')].copy()
        end_t0cf_frame.point *= 1e-3
        sample_found = False

        gantry_base_gen_fn = gantry_base_generator(client, robot, end_t0cf_frame, reachable_range=reachable_range, scale=1.0)

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
                    if verbose:
                        LOGGER.debug('IK sample found after {} gantry iters.'.format(gantry_iter))
                    break
            if sample_found:
                break
        else:
            LOGGER.error(colored('No robot IK conf can be found for {} after {} attempts.'.format(
                movement.short_summary, gantry_attempts), 'red'))
            return None

    # cprint('sample pick element succeeds for {}|{}.'.format(obj_name, tool_name), 'green')
    # return (itj_action,)
