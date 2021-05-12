import os
import sys
import pybullet
from termcolor import cprint
from copy import copy, deepcopy
from itertools import product
from collections import defaultdict

from compas.geometry import Frame, distance_point_point, Transformation
from compas_fab.robots import Configuration, Robot

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotClampAssemblyProcess, Movement
from integral_timber_joints.process.state import get_object_from_flange

import ikfast_abb_irb4600_40_255
from pybullet_planning import GREY
from pybullet_planning import link_from_name, get_link_pose, draw_pose, multiply, \
    joints_from_names, LockRenderer, WorldSaver, wait_for_user, joint_from_name, wait_if_gui, has_gui
from pybullet_planning import link_from_name, sample_tool_ik, is_pose_close
from pybullet_planning import compute_inverse_kinematics

from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo_examples.ik_solver import InverseKinematicsSolver, get_ik_fn_from_ikfast
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.utils import compare_configurations

from integral_timber_joints.planning.robot_setup import MAIN_ROBOT_ID, BARE_ARM_GROUP, GANTRY_ARM_GROUP, GANTRY_GROUP
from integral_timber_joints.planning.robot_setup import get_gantry_control_joint_names, get_gantry_robot_custom_limits
from integral_timber_joints.planning.utils import reverse_trajectory, merge_trajectories, FRAME_TOL
from integral_timber_joints.planning.state import set_state, gantry_base_generator
from integral_timber_joints.planning.utils import notify

##############################


def fill_in_tool_path(client: PyChoreoClient, robot: Robot, traj, group=GANTRY_ARM_GROUP):
    """Fill FK frames into `path_from_link` attribute to the given trajectory
    Note that the `path_from_link` attribute will not be serialized if exported as json.

    Returns
    -------
    Trajectory
        updated trajectory
    """
    if traj:
        tool_link_name = robot.get_end_effector_link_name(group=group)
        traj.path_from_link = defaultdict(list)
        with WorldSaver():
            for tj_pt in traj.points:
                client.set_robot_configuration(robot, tj_pt)
                traj.path_from_link[tool_link_name].append(client.get_link_frame_from_name(robot, tool_link_name))
    return traj

##############################


def _get_sample_bare_arm_ik_fn(client: PyChoreoClient, robot: Robot):
    """get the IKFast ik function for the 6-axis ABB robot.

    Returns
    -------
    IK function handle: pybullet pose -> list(float) of six values
    """
    robot_uid = client.get_robot_pybullet_uid(robot)
    ik_base_link_name = robot.get_base_link_name(group=BARE_ARM_GROUP)
    ik_joint_names = robot.get_configurable_joint_names(group=BARE_ARM_GROUP)
    # * joint indices in pybullet
    ik_base_link = link_from_name(robot_uid, ik_base_link_name)
    ik_joints = joints_from_names(robot_uid, ik_joint_names)
    ikfast_fn = ikfast_abb_irb4600_40_255.get_ik

    def get_sample_ik_fn(robot, ik_fn, robot_base_link, ik_joints, tool_from_root=None):
        def sample_ik_fn(world_from_tcp):
            if tool_from_root:
                world_from_tcp = multiply(world_from_tcp, tool_from_root)
            return sample_tool_ik(ik_fn, robot, ik_joints, world_from_tcp, robot_base_link, get_all=True)
        return sample_ik_fn
    sample_ik_fn = get_sample_ik_fn(robot_uid, ikfast_fn, ik_base_link, ik_joints)
    return sample_ik_fn

##############################

def check_cartesian_conf_agreement(client, robot, conf1, conf2, conf1_tag='', conf2_tag='', options=None, verbose=True):
    options = options or {}
    robot_uid = client.get_robot_pybullet_uid(robot)
    tool_link_name = robot.get_end_effector_link_name(group=BARE_ARM_GROUP)
    ik_tool_link = link_from_name(robot_uid, tool_link_name)
    jump_threshold = options.get('jump_threshold', {})

    if compare_configurations(conf1, conf2, jump_threshold, fallback_tol=1e-3, verbose=verbose):
        with WorldSaver():
            client.set_robot_configuration(robot, conf1)
            p1 = get_link_pose(robot_uid, ik_tool_link)
            client.set_robot_configuration(robot, conf2)
            p2 = get_link_pose(robot_uid, ik_tool_link)
            pose_close = is_pose_close(p1, p2)
        if verbose:
            cprint('LinearMovement: {} not coincided with {} - max diff {:.5f}'.format(
            conf1_tag, conf2_tag, conf1.max_difference(conf2)), 'red')
            if not pose_close:
                if verbose:
                    cprint('FK frame close: {} | ({},{}) vs. ({},{})'.format(pose_close,
                        ['{:.3f}'.format(v) for v in p1[0]], ['{:.3f}'.format(v) for v in p1[1]],
                        ['{:.3f}'.format(v) for v in p2[0]], ['{:.3f}'.format(v) for v in p2[1]])
                        )
            notify('Warning! Go back to the command line now!')
            # wait_for_user()
        return False
    else:
        return True

##############################

def compute_linear_movement(client: PyChoreoClient, robot: Robot, process: RobotClampAssemblyProcess, movement: Movement, options=None, diagnosis=False):
    assert isinstance(movement, RoboticLinearMovement) or \
        isinstance(movement, RoboticClampSyncLinearMovement)
    robot_uid = client.get_robot_pybullet_uid(robot)

    # * options
    # sampling attempts
    options = options or {}
    gantry_attempts = options.get('gantry_attempts') or 10
    cartesian_attempts = options.get('cartesian_attempts') or 5
    reachable_range = options.get('reachable_range') or (0.2, 2.8) # (0.68, 2.83)

    cartesian_move_group = options.get('cartesian_move_group') or GANTRY_ARM_GROUP
    gantry_group = GANTRY_GROUP
    debug = options.get('debug', False)
    verbose = options.get('verbose', True)

    # * custom limits
    ik_joint_names = robot.get_configurable_joint_names(group=BARE_ARM_GROUP)
    tool_link_name = robot.get_end_effector_link_name(group=BARE_ARM_GROUP)
    ik_tool_link = link_from_name(robot_uid, tool_link_name)

    gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)

    # * construct IK function
    sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
    # TODO switch to client IK
    # ikfast_fn = get_ik_fn_from_ikfast(ikfast_abb_irb4600_40_255.get_ik)
    # ik_solver = InverseKinematicsSolver(robot, move_group, ikfast_fn, base_frame, robotA_tool.frame)
    # client.planner.inverse_kinematics = ik_solver.inverse_kinematics_function()

    # * get target T0CF pose
    start_state = process.get_movement_start_state(movement)
    end_state = process.get_movement_end_state(movement)
    start_conf = start_state['robot'].kinematic_config
    end_conf = end_state['robot'].kinematic_config

    # * set start state
    try:
        set_state(client, robot, process, start_state)
    except RuntimeError:
        return None

    start_t0cf_frame = copy(start_state['robot'].current_frame)
    start_t0cf_frame.point *= 1e-3
    end_t0cf_frame = copy(end_state['robot'].current_frame)
    end_t0cf_frame.point *= 1e-3

    # TODO: ignore beam / env collision in the first pickup pose
    temp_name = '_tmp'
    for o1_name, o2_name in movement.allowed_collision_matrix:
        o1_bodies = client._get_bodies('^{}$'.format(o1_name))
        o2_bodies = client._get_bodies('^{}$'.format(o2_name))
        for parent_body, child_body in product(o1_bodies, o2_bodies):
            client.extra_disabled_collision_links[temp_name].add(
                ((parent_body, None), (child_body, None))
            )
    # TODO special check for CLampSyncMove:
    # you can disable (clamps - gripper) and (clamps - toolchanger) during the move,
    # but double check the end state (with jaw set to closed state) if they collide.

    with WorldSaver():
        if start_conf is not None:
            if not start_conf.joint_names:
                start_conf.joint_names = gantry_arm_joint_names
            client.set_robot_configuration(robot, start_conf)
            start_tool_pose = get_link_pose(robot_uid, ik_tool_link)
            start_t0cf_frame_temp = frame_from_pose(start_tool_pose, scale=1)
            if not start_t0cf_frame_temp.__eq__(start_t0cf_frame, tol=FRAME_TOL):
                if verbose:
                    cprint('start conf FK inconsistent ({:.5f} m) with given current frame in start state.'.format(
                        distance_point_point(start_t0cf_frame_temp.point, start_t0cf_frame.point)), 'yellow')
                # TODO this might impact feasibility, investigate further
                # cprint('Overwriting with the FK-one.', 'yellow')
                # start_t0cf_frame = start_t0cf_frame_temp
                # overwrite_start_frame = copy(start_t0cf_frame_temp)
                # overwrite_start_frame.point *= 1e3
                # start_state['robot'].current_frame = overwrite_start_frame
        if end_conf is not None:
            if not end_conf.joint_names:
                end_conf.joint_names = gantry_arm_joint_names
            client.set_robot_configuration(robot, end_conf)
            end_tool_pose = get_link_pose(robot_uid, ik_tool_link)
            end_t0cf_frame_temp = frame_from_pose(end_tool_pose, scale=1)
            if not end_t0cf_frame_temp.__eq__(end_t0cf_frame, tol=FRAME_TOL):
                if verbose:
                    cprint('end conf FK inconsistent ({:.5f} m) with given current frame in end state.'.format(
                        distance_point_point(end_t0cf_frame_temp.point, end_t0cf_frame.point)), 'yellow')
                # TODO this might impact feasibility, investigate further
                # cprint('Overwriting with the FK-one.', 'yellow')
                # end_t0cf_frame = end_t0cf_frame_temp
                # overwrite_end_frame = copy(end_t0cf_frame_temp)
                # overwrite_end_frame.point *= 1e3
                # start_state['robot'].current_frame = overwrite_end_frame

    interp_frames = [start_t0cf_frame, end_t0cf_frame]
    solution_found = False
    samples_cnt = ik_failures = path_failures = 0
    planner_id = options.get('planner_id', 'IterativeIK')

    # TODO custom limits
    # TODO try fixing IK and only use gantry for Cartesian movements
    if start_conf is None and end_conf is None:
        # * sample from a ball near the pose
        gantry_base_gen_fn = gantry_base_generator(client, robot, interp_frames[0], reachable_range=reachable_range, scale=1.0)
        for gi, base_conf in zip(range(gantry_attempts), gantry_base_gen_fn):
            if verbose:
                cprint('-- gantry sampling iter {}'.format(gi), 'magenta')
            samples_cnt += 1
            arm_conf_vals = sample_ik_fn(pose_from_frame(interp_frames[0], scale=1))
            # * iterate through all 6-axis IK solution
            for ik_iter, arm_conf_val in enumerate(arm_conf_vals):
                if verbose:
                    cprint('   -- IK iter {}'.format(ik_iter), 'magenta')
                if arm_conf_val is None:
                    continue
                gantry_arm_conf = Configuration(list(base_conf.values) + list(arm_conf_val),
                    gantry_arm_joint_types, gantry_arm_joint_names)
                if not client.check_collisions(robot, gantry_arm_conf, options=options):
                    # * Cartesian planning, only for the six-axis arm (aka sub_conf)
                    for ci in range(cartesian_attempts):
                        # TODO pybullet IK struggles for dual arm setting
                        # see if we can create a x-y-z+6 axis subrobot clone
                        if verbose:
                            print('\tcartesian trial #{}'.format(ci))
                        cart_conf = client.plan_cartesian_motion(robot, interp_frames, start_configuration=gantry_arm_conf,
                            group=gantry_group, options=options)
                        if cart_conf is not None:
                            solution_found = True
                            if verbose:
                                cprint('Plan found by {}! After {} ik, {} path failure over {} samples.'.format(
                                planner_id, ik_failures, path_failures, samples_cnt), 'green')
                            break
                        else:
                            path_failures += 1
                if solution_found:
                    break
            else:
                ik_failures += 1
            if solution_found:
                break
        else:
            if verbose:
                cprint('Cartesian Path planning failure ({}) after {} attempts | {} due to IK, {} due to Cart.'.format(
                    planner_id, samples_cnt, ik_failures, path_failures), 'yellow')
    else:
        # TODO make sure start/end conf coincides if provided
        if start_conf is not None and end_conf is not None:
            if verbose:
                cprint('Both start/end confs are pre-specified, problem might be too stiff to be solved.', 'yellow')
        if start_conf:
            if verbose:
                cprint('One-sided Cartesian planning : start conf set, forward mode', 'blue')
            forward = True
            gantry_arm_conf = start_conf
        else:
            if verbose:
                cprint('One-sided Cartesian planning : end conf set, backward mode', 'blue')
            forward = False
            gantry_arm_conf = end_conf
            interp_frames = interp_frames[::-1]

        samples_cnt = 0
        for ci in range(cartesian_attempts):
            if verbose:
                print('\tcartesian trial #{}'.format(ci))
            cart_conf = client.plan_cartesian_motion(robot, interp_frames, start_configuration=gantry_arm_conf,
                group=cartesian_move_group, options=options)
            samples_cnt += 1
            if cart_conf is not None:
                success_planner = planner_id
                # cprint('Plan found by {}! After {} path failure over {} samples.'.format(
                #     planner_id, path_failures, samples_cnt), 'green')
                break
            else:
                path_failures += 1
        else:
            # * fallback to ladder graph
            lm_options = options.copy()
            lm_options['planner_id'] = 'LadderGraph'
            lm_options['ik_function'] = _get_sample_bare_arm_ik_fn(client, robot)
            # pose -> list(conf values)
            cart_conf = client.plan_cartesian_motion(robot, interp_frames, start_configuration=gantry_arm_conf,
                group=BARE_ARM_GROUP, options=lm_options)
            if cart_conf:
                success_planner = 'LadderGraph'

        if not cart_conf:
            if verbose:
                cprint('Cartesian Path planning (w/ prespecified st or end conf) failure after\n{} attempts of {} + 1 LadderGraph attempt.'.format(
                    planner_id, samples_cnt), 'yellow')
        else:
            if not forward:
                cart_conf = reverse_trajectory(cart_conf)
            if verbose:
                cprint('Plan found by {}! After {} path failure (by {}) over {} samples.'.format(
                    success_planner, path_failures, planner_id, samples_cnt), 'green')

    if cart_conf is None and diagnosis:
        lockrenderer = options.get('lockrenderer', None)
        if lockrenderer is not None:
            lockrenderer.restore()
        print('Start diagnosis.')
        print('movement.allowed_collision_matrix: ', movement.allowed_collision_matrix)
        print('extra_disabled_collision_links: ', client.extra_disabled_collision_links)
        client._print_object_summary()
        d_options = options.copy()
        d_options['diagnosis'] = True
        in_collision = client.check_collisions(robot, gantry_arm_conf, options=d_options)

        # d_options['planner_id'] = 'LadderGraph'
        # d_options['ik_function'] = _get_sample_bare_arm_ik_fn(client, robot)
        temp_cart_conf = client.plan_cartesian_motion(robot, interp_frames, start_configuration=gantry_arm_conf,
            group=cartesian_move_group, options=d_options)

        # print('in collision {} | cart conf {}'.format(in_collision, temp_cart_conf))
        wait_if_gui('Diagnosis done.')
        if lockrenderer is not None:
            lockrenderer = LockRenderer()

    if temp_name in client.extra_disabled_collision_links:
        del client.extra_disabled_collision_links[temp_name]

    traj = None
    if cart_conf:
        traj = cart_conf
        if start_conf is not None:
            # TODO if return None?
            check_cartesian_conf_agreement(client, robot, start_conf, traj.points[0],
                conf1_tag='given start conf', conf2_tag='traj[0]', options=options, verbose=verbose)
        if end_conf is not None:
            check_cartesian_conf_agreement(client, robot, end_conf, traj.points[-1],
                conf1_tag='given end conf', conf2_tag='traj[-1]', options=options, verbose=verbose)
    else:
        if verbose:
            cprint('No linear movement found for {}.'.format(movement.short_summary), 'red')
    return fill_in_tool_path(client, robot, traj, group=GANTRY_ARM_GROUP)

##############################

def compute_free_movement(client: PyChoreoClient, robot: Robot, process: RobotClampAssemblyProcess, movement: Movement,
        options=None, diagnosis=False):
    assert isinstance(movement, RoboticFreeMovement)
    options = options or {}
    # * options
    debug = options.get('debug', False)
    verbose = options.get('verbose', True)
    # * sampling attempts, needed only if start/end conf not specified
    gantry_attempts = options.get('gantry_attempts') or 500
    reachable_range = options.get('reachable_range') or (0.2, 2.8)

    start_state = process.get_movement_start_state(movement)
    end_state = process.get_movement_end_state(movement)
    orig_start_conf = start_state['robot'].kinematic_config
    orig_end_conf = end_state['robot'].kinematic_config

    assert orig_end_conf is not None, 'End conf not provided in {}, which should never happen'.format(movement.short_summary)

    # fill in joint names if needed
    if orig_start_conf and len(orig_start_conf.joint_names) == 0:
        orig_start_conf.joint_names = orig_end_conf.joint_names

    # * set start state
    try:
        set_state(client, robot, process, start_state)
    except RuntimeError:
        return None

    if orig_start_conf is None:
        cprint('FreeMovement: Robot start conf is NOT specified in {}, we will sample an IK conf based on the given t0cp frame.'.format(movement.short_summary), 'yellow')
        if verbose:
            notify('Warning! Go back to the command line now!')
            # wait_for_user('Please press Enter to confirm.')
        # * sample from t0cp if no conf is provided for the robot
        start_t0cf_frame = copy(start_state['robot'].current_frame)
        start_t0cf_frame.point *= 1e-3
        if start_t0cf_frame is not None:
            gantry_base_gen_fn = gantry_base_generator(client, robot, start_t0cf_frame, reachable_range=reachable_range, scale=1.0)
            for _, base_conf in zip(range(gantry_attempts), gantry_base_gen_fn):
                orig_start_conf = client.inverse_kinematics(robot, start_t0cf_frame, group=GANTRY_ARM_GROUP, options=options)
                if orig_start_conf:
                    break
            else:
                # if verbose:
                cprint('No robot IK conf can be found for {} after {} attempts, Underspecified problem, solve fails.'.format(
                    movement.short_summary, gantry_attempts), 'red')
                # raise RuntimeError()
                return None
        else:
            # if verbose:
            cprint('No robot start frame is specified in {}, Underspecified problem, solve fails.'.format(movement.short_summary), 'red')
            # raise RuntimeError()
            return None

    start_conf = orig_start_conf
    end_conf = orig_end_conf

    # * custom limits
    custom_limits = get_gantry_robot_custom_limits(MAIN_ROBOT_ID)
    if 'custom_limits' not in options:
        options.update({'custom_limits': custom_limits})

    goal_constraints = robot.constraints_from_configuration(end_conf, [0.01], [0.01], group=GANTRY_ARM_GROUP)
    with LockRenderer():
        traj = client.plan_motion(robot, goal_constraints, start_configuration=start_conf, group=GANTRY_ARM_GROUP,
                                  options=options)
    if verbose:
        if traj is None:
            cprint('No free movement found for {}.'.format(movement.short_summary), 'red')
        else:
            cprint('Free movement found for {}!'.format(movement.short_summary), 'green')

    if traj is None and diagnosis:
        client._print_object_summary()
        lockrenderer = options.get('lockrenderer', None)
        if lockrenderer:
            lockrenderer.restore()
        print('Start diagnosis.')
        d_options = options.copy()
        d_options['diagnosis'] = True
        traj = client.plan_motion(robot, goal_constraints, start_configuration=start_conf, group=GANTRY_ARM_GROUP,
                                  options=d_options)
        if lockrenderer:
            lockrenderer = LockRenderer()

    return fill_in_tool_path(client, robot, traj, group=GANTRY_ARM_GROUP)
