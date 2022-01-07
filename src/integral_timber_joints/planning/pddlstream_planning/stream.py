from collections import namedtuple
import time
from itertools import product, chain
from compas.geometry.primitives import frame
from pybullet_planning.interfaces.env_manager.user_io import wait_if_gui
from termcolor import cprint
import pybullet_planning as pp

from compas_fab.robots import Configuration
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose, pose_from_transformation

from integral_timber_joints.planning.robot_setup import GANTRY_ARM_GROUP
from integral_timber_joints.process.state import SceneState
from integral_timber_joints.planning.state import set_state, gantry_base_generator
from integral_timber_joints.planning.stream import _get_sample_bare_arm_ik_fn

from compas.robots import Configuration
from compas_fab.robots import Trajectory

######################################

def assign_fluent_state(client, robot, process, fluents):
    # * manually create a state according to the fluent facts and set_state
    state = SceneState(process)
    # by default all the attachment status is None (False)
    for oid in chain(process.assembly.sequence, process.tool_ids):
        state[(oid, 'a')] = False
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
            # state[(object_name, 'g')] = grasp_transform
        else:
            raise ValueError(name)
    set_state(client, robot, process, state)
    return state

######################################

def _archived_get_action_ik_fn(client, process, robot, options=None):
    options = options or {}
    debug = options.get('debug', False)
    verbose = options.get('verbose', False)
    reachable_range = options.get('reachable_range', (0.2, 2.5))
    ik_gantry_attempts = options.get('ik_gantry_attempts', 10)

    def ik_fn(object_name, object_pose, grasp, fluents=[]):

        # * set state
        # pp.wait_if_gui('IK: Before assign fluent state')
        assign_fluent_state(client, robot, process, fluents)
        # print(client._print_object_summary())
        # pp.wait_if_gui('IK: After assign fluent state')

        state = SceneState(process)
        object_frame = object_pose.copy()
        object_frame.point *= 1e-3
        state[(object_name, 'f')] = object_frame
        state[(object_name, 'g')] = grasp
        world_from_object = pose_from_frame(object_frame)
        robot_flange_from_attached_obj = pose_from_transformation(grasp, scale=1e-3)
        flange_frame = frame_from_pose(pp.multiply(world_from_object, pp.invert(robot_flange_from_attached_obj)))

        # TODO use trac-ik here
        sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
        gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
        gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)

        # print(flange_frame)
        # pp.draw_pose(pose_from_frame(flange_frame, scale=1))

        # * ACM setup
        temp_name = '_tmp'
        allowed_collision_matrix = [('tool_changer', object_name)]
        if object_name == 'g1':
            # TODO fix this hand-coded index...
            allowed_collision_matrix.extend([
                (object_name, 'e47'),
                (object_name, 'e48'),
                (object_name, 'e53'),
                (object_name, 'e54'),
                ])

        for o1_name, o2_name in allowed_collision_matrix:
            o1_bodies = client._get_bodies('^{}$'.format(o1_name))
            o2_bodies = client._get_bodies('^{}$'.format(o2_name))
            for parent_body, child_body in product(o1_bodies, o2_bodies):
                client.extra_disabled_collision_links[temp_name].add(
                    ((parent_body, None), (child_body, None))
                )

        start_time = time.time()
        gantry_base_gen_fn = gantry_base_generator(client, robot, flange_frame, reachable_range=reachable_range, scale=1.0)
        with pp.LockRenderer(not debug):
            # * sample from a ball near the pose
            for attempt, base_conf in zip(range(ik_gantry_attempts), gantry_base_gen_fn):
                # * pybullet gradient-based IK
                conf = client.inverse_kinematics(robot, flange_frame, group=GANTRY_ARM_GROUP,
                                                 options=options)
                if not conf:
                    # * bare-arm IKfast sampler
                    arm_conf_vals = sample_ik_fn(pose_from_frame(flange_frame, scale=1))
                    # in total, 8 ik solutions for the 6-axis arm
                    for ik_iter, arm_conf_val in enumerate(arm_conf_vals):
                        if arm_conf_val is None:
                            continue
                        conf = Configuration(list(base_conf.joint_values) + list(arm_conf_val),
                            gantry_arm_joint_types, gantry_arm_joint_names)
                        if not client.check_collisions(robot, conf, options=options):
                            if verbose:
                                msg = 'IK found with IKFast after {}/{} gantry attempts - {} ik attempts / total {} IKFast solutions | total time {:.3f}'.format(
                                    attempt+1, ik_gantry_attempts, ik_iter, len(arm_conf_vals), pp.elapsed_time(start_time))
                                cprint(msg, 'green')

                            if temp_name in client.extra_disabled_collision_links:
                                del client.extra_disabled_collision_links[temp_name]
                            return (conf,)
                else:
                    if verbose:
                        msg = 'IK found with pb-IK after {}/{} gantry attempts | total time {:.3f}'.format(
                            attempt+1, ik_gantry_attempts, pp.elapsed_time(start_time))
                        cprint(msg, 'green')

                    if temp_name in client.extra_disabled_collision_links:
                        del client.extra_disabled_collision_links[temp_name]
                    return (conf,)
        if verbose:
            msg = 'no IK solotion found after {} gantry attempts | total time {:.3f}'.format(ik_gantry_attempts, pp.elapsed_time(start_time))
            cprint(msg, 'yellow')

        if temp_name in client.extra_disabled_collision_links:
            del client.extra_disabled_collision_links[temp_name]
        return None

    return ik_fn

###########################################

from termcolor import colored
from integral_timber_joints.planning.solve import compute_movement
from integral_timber_joints.process.action import PickBeamWithGripperAction
from integral_timber_joints.process.movement import RoboticMovement, RoboticFreeMovement
from integral_timber_joints.planning.visualization import visualize_movement_trajectory

def apply_movement_state_diff(scene, movements, debug=False):
    for key in scene.object_keys:
        for m in movements[::-1]:
            if key in m.state_diff:
                scene[key] = m.state_diff[key]
                if debug:
                    print(colored(key, 'blue' if key[0]=='robot' else 'grey'), m.state_diff[key])
                break # ! break movement loop
    return scene

def get_action_ik_fn(client, process, robot, action_name, options=None):
    debug = options.get('debug', False)
    verbose = options.get('verbose', False)
    def sample_fn(obj_name, tool_name, fluents=[]):
        # ! :inputs (?object ?tool)
        # ! :outputs (?conf1 ?conf2 ?traj)
        # * assign `AtPose` and `Attached` predicates to state
        # start_state of the action
        # pp.wait_if_gui('PickBeam: Before assign fluent state')
        _action_scene = assign_fluent_state(client, robot, process, fluents)
        # print(client._print_object_summary())
        # pp.wait_if_gui('PickBeam: After assign fluent state')

        # if pddl_action.name == 'pick_beam_with_gripper':
        itj_action = PickBeamWithGripperAction(seq_n=0, act_n=0, beam_id=obj_name, gripper_id=tool_name)

        itj_action.create_movements(process)
        end_scene = _action_scene.copy()
        for i, movement in enumerate(itj_action.movements):
            if debug:
                print('='*5)
                print('{} - {}'.format(i, movement.short_summary))
            movement.create_state_diff(process)
            start_scene = end_scene.copy()
            start_scene = apply_movement_state_diff(start_scene, itj_action.movements[:i], debug=debug)
            # if debug:
                # print('~~~')
            end_scene = start_scene.copy()
            end_scene = apply_movement_state_diff(end_scene, [itj_action.movements[i]], debug=debug)
            # * Robot state is not the same as previous.
            end_scene[process.robot_config_key] = None
            # itj_action.movements[i].state_diff[process.robot_config_key]

            # * skip free and non-robotic movements
            if not isinstance(movement, RoboticMovement) or \
               isinstance(movement, RoboticFreeMovement):
               continue

            if compute_movement(client, robot, process, movement,
                options=options, diagnosis=options['diagnosis'], given_state_pair=(start_scene, end_scene)):
                # * conf backward-propagation
                m_id = itj_action.movements.index(movement)
                back_id = m_id-1
                while back_id >= 0:
                    back_m = itj_action.movements[back_id]
                    if not isinstance(back_m, RoboticMovement) or isinstance(back_m, RoboticFreeMovement):
                        back_m.state_diff[process.robot_config_key] = movement.trajectory.points[0]
                    else:
                        break
                    back_id -= 1
                if debug:
                    wait_if_gui('Start sim')
                    with pp.WorldSaver():
                        visualize_movement_trajectory(client, robot, process, movement, step_sim=False,
                            given_state_pair=(start_scene, end_scene), draw_polylines=True)
            else:
                cprint('sample pick element fails for {}.'.format(movement.short_summary), 'red')
                return None
        cprint('sample pick element succeeds for {}|{}.'.format(obj_name, tool_name), 'green')
        return (itj_action,)
    return sample_fn
