import os, sys
import pybullet
from termcolor import cprint
from copy import copy, deepcopy
from itertools import product
from collections import defaultdict

from compas.geometry import Frame, distance_point_point, Transformation
from compas_fab.robots import Configuration

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticClampSyncLinearMovement
from integral_timber_joints.process.state import get_object_from_flange

import ikfast_abb_irb4600_40_255
from pybullet_planning import GREY
from pybullet_planning import link_from_name, get_link_pose, draw_pose, multiply, \
    joints_from_names, LockRenderer, WorldSaver, wait_for_user, joint_from_name, wait_if_gui
from pybullet_planning import get_sample_fn, link_from_name, sample_tool_ik
from pybullet_planning import uniform_pose_generator

from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo_examples.ik_solver import InverseKinematicsSolver, get_ik_fn_from_ikfast

from .robot_setup import R11_INTER_CONF_VALS, MAIN_ROBOT_ID, BARE_ARM_GROUP, GANTRY_ARM_GROUP, GANTRY_Z_LIMIT
from .robot_setup import get_gantry_control_joint_names, get_gantry_robot_custom_limits
from .utils import reverse_trajectory, merge_trajectories, FRAME_TOL
from .state import set_state

##############################

def fill_in_tool_path(client, robot, traj, group=GANTRY_ARM_GROUP):
    if traj:
        tool_link_name = robot.get_end_effector_link_name(group=group)
        traj.path_from_link = defaultdict(list)
        with WorldSaver():
            for tj_pt in traj.points:
                client.set_robot_configuration(robot, tj_pt)
                traj.path_from_link[tool_link_name].append(client.get_link_frame_from_name(robot, tool_link_name))
    return traj

##############################

def _get_sample_bare_arm_ik_fn(client, robot):
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

def compute_linear_movement(client, robot, process, movement, options=None):
    assert isinstance(movement, RoboticLinearMovement) or \
           isinstance(movement, RoboticClampSyncLinearMovement)
    robot_uid = client.get_robot_pybullet_uid(robot)

    # * options
    # sampling attempts
    options = options or {}
    gantry_attempts = options.get('gantry_attempts') or 10
    reachable_range = options.get('reachable_range') or (0.2, 2.8) # (0.68, 2.83)
    debug = options.get('debug', False)

    # * custom limits
    ik_joint_names = robot.get_configurable_joint_names(group=BARE_ARM_GROUP)
    tool_link_name = robot.get_end_effector_link_name(group=BARE_ARM_GROUP)
    # pb ids
    ik_joints = joints_from_names(robot_uid, ik_joint_names)
    ik_tool_link = link_from_name(robot_uid, tool_link_name)

    gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)
    # gantry_joint_names = list(set(gantry_arm_joint_names) - set(ik_joint_names))

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
    set_state(client, robot, process, start_state)

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
    if debug:
        print('movement.allowed_collision_matrix: ', movement.allowed_collision_matrix)
        print('extra_disabled_collision_links: ', client.extra_disabled_collision_links)
        client._print_object_summary()

    # TODO special check for CLampSyncMove:
    # you can disable (clamps - gripper) and (clamps - toolchanger) during the move,
    # but double check the end state (with jaw set to closed state) if they collide.

    with WorldSaver():
        if start_conf is not None:
            client.set_robot_configuration(robot, start_conf)
            start_tool_pose = get_link_pose(robot_uid, ik_tool_link)
            start_t0cf_frame_temp = frame_from_pose(start_tool_pose, scale=1)
            if not start_t0cf_frame_temp.__eq__(start_t0cf_frame, tol=FRAME_TOL):
                cprint('start conf FK inconsistent ({:.5f} m) with given current frame in start state.'.format(
                    distance_point_point(start_t0cf_frame_temp.point, start_t0cf_frame.point)), 'yellow')
            # start_t0cf_frame = start_t0cf_frame_temp
            # , overwriting with the FK one.
            # start_state['robot'].current_frame = start_t0cf_frame_temp
        if end_conf is not None:
            client.set_robot_configuration(robot, end_conf)
            end_tool_pose = get_link_pose(robot_uid, ik_tool_link)
            end_t0cf_frame_temp = frame_from_pose(end_tool_pose, scale=1)
            if not end_t0cf_frame_temp.__eq__(end_t0cf_frame, tol=FRAME_TOL):
                cprint('end conf FK inconsistent ({:.5f} m) with given current frame in end state.'.format(
                    distance_point_point(end_t0cf_frame_temp.point, end_t0cf_frame.point)), 'yellow')
            # end_t0cf_frame = end_t0cf_frame_temp
            # , overwriting with the FK one
            # end_state['robot'].current_frame = end_t0cf_frame_temp

    interp_frames = [start_t0cf_frame, end_t0cf_frame]

    sorted_gantry_joint_names = get_gantry_control_joint_names(MAIN_ROBOT_ID)
    gantry_joints = joints_from_names(robot_uid, sorted_gantry_joint_names)
    gantry_z_joint = joint_from_name(robot_uid, sorted_gantry_joint_names[2])
    gantry_z_sample_fn = get_sample_fn(robot_uid, [gantry_z_joint], custom_limits={gantry_z_joint : GANTRY_Z_LIMIT})

    solution_found = False
    samples_cnt = ik_failures = path_failures = 0

    # TODO custom limits

    if start_conf is None and end_conf is None:
        # * sample from a ball near the pose
        base_gen_fn = uniform_pose_generator(robot_uid, pose_from_frame(interp_frames[0], scale=1), reachable_range=reachable_range)
        for gi in range(gantry_attempts):
            if debug:
                cprint('-- gantry sampling iter {}'.format(gi), 'magenta')
            x, y, yaw = next(base_gen_fn)
            # TODO a more formal gantry_base_from_world_base
            y *= -1
            z, = gantry_z_sample_fn()
            gantry_xyz_vals = [x,y,z]
            gantry_conf = Configuration(gantry_xyz_vals,
                    gantry_arm_joint_types[:3], gantry_arm_joint_names[:3])
            client.set_robot_configuration(robot, gantry_conf)
            samples_cnt += 1

            arm_conf_vals = sample_ik_fn(pose_from_frame(interp_frames[0], scale=1))
            # iterate through all 6-axis IK solution
            for arm_conf_val in arm_conf_vals:
                if arm_conf_val is None:
                    continue
                gantry_arm_conf = Configuration(list(gantry_xyz_vals) + list(arm_conf_val),
                    gantry_arm_joint_types, gantry_arm_joint_names)
                if not client.check_collisions(robot, gantry_arm_conf, options=options):
                    # * Cartesian planning, only for the six-axis arm (aka sub_conf)
                    # cart_conf_vals = plan_cartesian_motion(robot_uid, ik_joints[0], ik_tool_link, interp_poses, get_sub_conf=True)
                    for ci in range(2):
                        if debug:
                            print('\tcartesian trial #{}'.format(ci))
                        cart_conf = client.plan_cartesian_motion(robot, interp_frames, start_configuration=gantry_arm_conf,
                            group=GANTRY_ARM_GROUP, options=options)
                        if cart_conf is not None:
                            solution_found = True
                            cprint('Collision free! After {} ik, {} path failure over {} samples.'.format(
                                ik_failures, path_failures, samples_cnt), 'green')
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
            cprint('Cartesian Path planning failure after {} attempts | {} due to IK, {} due to Cart.'.format(
                samples_cnt, ik_failures, path_failures), 'yellow')
    else:
        # TODO make sure start/end conf coincides if provided
        if start_conf is not None and end_conf is not None:
            cprint('Both start/end confs are pre-specified, problem might be too stiff to be solved.', 'yellow')
        if start_conf:
            cprint('One-sided Cartesian planning : start conf set, forward mode')
            forward = True
            gantry_arm_conf = start_conf
        else:
            cprint('One-sided Cartesian planning : end conf set, backward mode')
            forward = False
            gantry_arm_conf = end_conf
            interp_frames = interp_frames[::-1]

        samples_cnt = 0
        for ci in range(5):
            if debug:
                print('\tcartesian trial #{}'.format(ci))
            cart_conf = client.plan_cartesian_motion(robot, interp_frames, start_configuration=gantry_arm_conf,
                group=GANTRY_ARM_GROUP, options=options)
            samples_cnt += 1
            if cart_conf is not None:
                solution_found = True
                cprint('Collision free! After {} path failure over {} samples.'.format(
                    path_failures, samples_cnt), 'green')
                if not forward:
                    cart_conf = reverse_trajectory(cart_conf)
                break
            else:
                path_failures += 1
        else:
            cprint('Cartesian Path planning (w/ prespecified st/end conf) failure after {} attempts.'.format(
                samples_cnt), 'yellow')

    if temp_name in client.extra_disabled_collision_links:
        del client.extra_disabled_collision_links[temp_name]

    traj = None
    if solution_found:
        traj = cart_conf
        if start_conf is not None and not start_conf.close_to(traj.points[0], tol=1e-3):
            cprint('Start conf not coincided - max diff {:.5f}'.format(start_conf.max_difference(traj.points[0])), 'red')
            wait_for_user()
        if end_conf is not None and not end_conf.close_to(traj.points[-1], tol=1e-3):
            cprint('End conf not coincided - max diff {:.5f}'.format(end_conf.max_difference(traj.points[-1])), 'red')
            wait_for_user()
    else:
        cprint('No linear movement found for {}.'.format(movement.short_summary), 'red')
    return fill_in_tool_path(client, robot, traj, group=GANTRY_ARM_GROUP)

##############################

def compute_free_movement(client, robot, process, movement, options=None):
    assert isinstance(movement, RoboticFreeMovement)
    options = options or {}
    # * options
    # sampling attempts, needed only if start/end conf not specified
    debug = options.get('debug', False)

    start_state = process.get_movement_start_state(movement)
    end_state = process.get_movement_end_state(movement)
    orig_start_conf = start_state['robot'].kinematic_config
    orig_end_conf = end_state['robot'].kinematic_config
    # TODO investigate why this happens
    if len(orig_start_conf.joint_names) == 0:
        orig_start_conf.joint_names = orig_end_conf.joint_names
    # print('orig start conf: ', orig_start_conf.joint_names, orig_end_conf.values)
    # print('orig end conf: ', orig_end_conf.joint_names, orig_end_conf.values)

    # * set start state
    set_state(client, robot, process, start_state)
    if debug:
        client._print_object_summary()

    hotfix = True
    if 'Free Move to reach Pickup Approach' in movement.tag:
        # * Start_conf -> (retraction) -> free motion -> end conf
        forward = True
        gantry_arm_conf = orig_start_conf
    elif 'Free Move to reach Storage Approach Frame' in movement.tag:
        # * Start_conf -> free motion -> (retraction) -> end conf
        forward = False
        gantry_arm_conf = orig_end_conf
    else:
        hotfix = False

    if hotfix:
        client.set_robot_configuration(robot, gantry_arm_conf)
        tool_link_name = robot.get_end_effector_link_name(group=BARE_ARM_GROUP)
        tool0_frame = client.get_link_frame_from_name(robot, tool_link_name)
        VER_RETRACTION_DISTANCE = 280 * 1e-3 # meter
        # HOR_RETRACTION_DISTANCE = 400 * 1e-3 # meter
        ver_retract_tf = Transformation.from_frame(Frame([0, 0, VER_RETRACTION_DISTANCE], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]))
        # hor_retract_tf = Transformation.from_frame(Frame([-HOR_RETRACTION_DISTANCE, 0, 0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]))
        tool0_frame_ver_retract = tool0_frame.transformed(ver_retract_tf)
        # tool0_frame_hor_retract = tool0_frame_ver_retract.transformed(hor_retract_tf)

        # interp_frames = [tool0_frame, tool0_frame_ver_retract, tool0_frame_hor_retract]
        interp_frames = [tool0_frame, tool0_frame_ver_retract]
        for frame in interp_frames:
            draw_pose(pose_from_frame(frame))
        # wait_if_gui('hotfix cartesian')

        options['diagnosis'] = True
        samples_cnt = path_failures = 0
        for _ in range(10):
            with WorldSaver():
                cart_conf = client.plan_cartesian_motion(robot, interp_frames, start_configuration=gantry_arm_conf,
                    group=GANTRY_ARM_GROUP, options=options)
            samples_cnt += 1
            if cart_conf is not None:
                solution_found = True
                cprint('(hotfix retraction for free motion) Collision free! After {} path failure over {} samples.'.format(
                    path_failures, samples_cnt), 'green')
                if not forward:
                    cart_conf = reverse_trajectory(cart_conf)
                break
            else:
                path_failures += 1
        else:
            cprint('(hotfix retraction for free motion) Cartesian Path planning (w/ prespecified st/end conf) failure after {} attempts.'.format(
                samples_cnt), 'yellow')
            return None
        if forward:
            # * Start_conf -> (retraction) -> free motion -> end conf
            start_conf = cart_conf.points[-1]
            end_conf = orig_end_conf
        else:
            # * Start_conf -> free motion -> (retraction) -> end conf
            start_conf = orig_start_conf
            end_conf = cart_conf.points[0]
    else:
        start_conf = orig_start_conf
        end_conf = orig_end_conf

    if start_conf is None or end_conf is None:
        cprint('At least one of robot start/end conf is NOT specified in {}, return None'.format(movement.short_summary), 'red')
        cprint('Start {} | End {}'.format(start_conf, end_conf), 'red')
        wait_for_user()
        return None

    # * custom limits
    custom_limits = get_gantry_robot_custom_limits(MAIN_ROBOT_ID)
    if 'custom_limits' not in options:
        options.update({'custom_limits' : custom_limits})

    goal_constraints = robot.constraints_from_configuration(end_conf, [0.01], [0.01], group=GANTRY_ARM_GROUP)
    with LockRenderer():
        traj = client.plan_motion(robot, goal_constraints, start_configuration=start_conf, group=GANTRY_ARM_GROUP, options=options)
    if traj is None:
        cprint('No free movement found for {}.'.format(movement.short_summary), 'red')
    else:
        cprint('Free movement found for {}!'.format(movement.short_summary), 'green')

    if hotfix and traj is not None:
        trajs = [cart_conf, traj] if forward else [traj, cart_conf]
        traj = merge_trajectories(trajs)

    return fill_in_tool_path(client, robot, traj, group=GANTRY_ARM_GROUP)


##########################################
# archived
# auto sample IK for FreeMovements
        # * sample from t0cp if no conf is provided for the robot
        # cprint('No robot start/end conf is specified in {}, performing random IK solves.'.format(movement), 'yellow')
        # start_t0cf_frame = start_state['robot'].current_frame
        # end_t0cf_frame = end_state['robot'].current_frame
        # if start_t0cf_frame is not None:
        #     start_conf = client.inverse_kinematics(robot, start_t0cf_frame, group=GANTRY_ARM_GROUP, options=options)
        # else:
        #     if initial_conf is not None:
        #         cprint('No robot start frame is specified in {}, using initial conf.'.format(movement), 'yellow')
        #         start_conf = initial_conf
        #     else:
        #         cprint('Underspecified problem, solve fails.', 'red')
        #         return None
        # if end_t0cf_frame is not None:
        #     end_conf = client.inverse_kinematics(robot, end_t0cf_frame, group=GANTRY_ARM_GROUP, options=options)
        # else:
        #     if initial_conf is not None:
        #         cprint('No robot end frame is specified in {}, using initial conf.'.format(movement), 'yellow')
        #         end_conf = initial_conf
        #     else:
        #         cprint('Underspecified problem, solve fails.', 'red')
        #         return None
        # if start_conf is None or end_conf is None:
        #     cprint('IK solves for start and end conf fails.', 'red')
        #     return None

