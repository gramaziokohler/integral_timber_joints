import os
import sys
import pybullet
import numpy as np
from termcolor import cprint
from copy import copy, deepcopy
from itertools import product
from collections import defaultdict

from compas.geometry import Frame, distance_point_point, Transformation
from compas_fab.robots import Configuration, Robot
from trac_ik_python.trac_ik_wrap import TRAC_IK

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotClampAssemblyProcess, Movement
from integral_timber_joints.process.state import get_object_from_flange

import ikfast_abb_irb4600_40_255
from trac_ik_python.trac_ik import IK
TRAC_IK_TIMEOUT = 1.0 # 0.1
TRAC_IK_TOL = 1e-6

import pybullet_planning as pp
from pybullet_planning import GREY
from pybullet_planning import link_from_name, get_link_pose, draw_pose, multiply, \
    joints_from_names, LockRenderer, WorldSaver, wait_for_user, joint_from_name, wait_if_gui, has_gui
from pybullet_planning import link_from_name, sample_tool_ik, is_pose_close
from pybullet_planning import compute_inverse_kinematics
from pybullet_planning import INF, invert, get_joint_positions, get_unit_vector, draw_point

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

from collections import namedtuple
IKInfo = namedtuple('IKInfo', ['ik_fn', 'ee_link_name', 'ik_joint_names', 'free_joint_names']) # 'base_link_name',

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

    def get_sample_ik_fn(robot, ik_fn, robot_base_link, ik_joints, ee_from_tool=None):
        def sample_ik_fn(world_from_tcp):
            if ee_from_tool:
                world_from_tcp = multiply(world_from_tcp, ee_from_tool)
            return sample_tool_ik(ik_fn, robot, ik_joints, world_from_tcp, robot_base_link, get_all=True)
        return sample_ik_fn
    sample_ik_fn = get_sample_ik_fn(robot_uid, ikfast_fn, ik_base_link, ik_joints)
    return sample_ik_fn

def base_sample_inverse_kinematics(robot_uid, ik_info, world_from_target,
                              fixed_joints=[], max_ik_attempts=200, max_ik_time=INF,
                              norm=INF, max_joint_distance=INF, free_delta=0.01, use_pybullet=False, **kwargs):
    assert (max_ik_attempts < INF) or (max_ik_time < INF)
    if max_joint_distance is None:
        max_joint_distance = INF
    #assert is_ik_compiled(ikfast_info)
    # ikfast = import_ikfast(ikfast_info)
    # ik_joints = get_ik_joints(robot, ikfast_info, tool_link)
    ik_joints = joints_from_names(robot_uid, ik_info.ik_joint_names)
    free_joints = joints_from_names(robot_uid, ik_info.free_joint_names) if ik_info.free_joint_names else []
    ee_link = link_from_name(robot_uid, ik_info.ee_link_name)
    # base_from_ee = get_base_from_ee(robot_uid, ikfast_info, tool_link, world_from_target)

    difference_fn = get_difference_fn(robot_uid, ik_joints)
    current_conf = get_joint_positions(robot_uid,ik_joints)
    current_positions = get_joint_positions(robot_uid, free_joints)

    # TODO: handle circular joints
    # TODO: use norm=INF to limit the search for free values
    free_deltas = np.array([0. if joint in fixed_joints else free_delta for joint in free_joints])
    lower_limits = np.maximum(get_min_limits(robot_uid, free_joints), current_positions - free_deltas)
    upper_limits = np.minimum(get_max_limits(robot_uid, free_joints), current_positions + free_deltas)
    generator = chain([current_positions], # TODO: sample from a truncated Gaussian nearby
                      interval_generator(lower_limits, upper_limits))
    if max_ik_attempts < INF:
        generator = islice(generator, max_ik_attempts)
    start_time = time.time()
    for free_positions in generator:
        if max_ik_time < elapsed_time(start_time):
            break
        # ! set free joint
        set_joint_positions(robot_uid, free_joints, free_positions)
        if use_pybullet:
            sub_robot, selected_joints, sub_target_link = create_sub_robot(robot_uid, ik_joints[0], ee_link)
            sub_joints = prune_fixed_joints(sub_robot, get_ordered_ancestors(sub_robot, sub_target_link))
            # ! call ik fn on ik_joints
            all_confs = []
            sub_kinematic_conf = inverse_kinematics(sub_robot, sub_target_link, world_from_target)
                                                    # pos_tolerance=pos_tolerance, ori_tolerance=ori_tolerance)
            if sub_kinematic_conf is not None:
                #set_configuration(sub_robot, sub_kinematic_conf)
                sub_kinematic_conf = get_joint_positions(sub_robot, sub_joints)
                set_joint_positions(robot_uid, selected_joints, sub_kinematic_conf)
                all_confs = [sub_kinematic_conf]
        else:
            all_confs = ik_info.ik_fn(world_from_target)

        for conf in randomize(all_confs):
            #solution(robot, ik_joints, conf, tool_link, world_from_target)
            difference = difference_fn(current_conf, conf)
            if not violates_limits(robot_uid, ik_joints, conf) and (get_length(difference, norm=norm) <= max_joint_distance):
                #set_joint_positions(robot, ik_joints, conf)
                yield (free_positions, conf)
            # else:
            #     print('current_conf: ', current_conf)
            #     print('conf: ', conf)
            #     print('diff: {}', difference)
        # else:
        #     min_distance = min([INF] + [get_length(difference_fn(q, current_conf), norm=norm) for q in all_confs])
            # print('None of out {} solutions works, min distance {}.'.format(len(all_confs), min_distance))
            #     lower_limits, upper_limits = get_custom_limits(robot_uid, ik_joints)
            #     print('L: ', lower_limits)
            #     print('U: ', upper_limits)
        if use_pybullet:
            remove_body(sub_robot)

def get_solve_trac_ik_info(trac_ik_solver, robot_uid):
    init_lower, init_upper = trac_ik_solver.get_joint_limits()
    base_link = link_from_name(robot_uid, trac_ik_solver.base_link)
    def track_ik_fn(world_from_tool, nearby_tolerance=INF):
        world_from_base = get_link_pose(robot_uid, base_link)
        # tip_link = link_from_name(robot_uid, ik_solver.tip_link)
        # tool_from_tip = multiply(invert(get_link_pose(robot_uid, self.tool_link)),
        #                          get_link_pose(robot_uid, tip_link))
        # world_from_tip = multiply(world_from_tool, tool_from_tip)
        world_from_tip = world_from_tool

        base_from_tip = multiply(invert(world_from_base), world_from_tip)
        joints = joints_from_names(robot_uid, trac_ik_solver.joint_names)  # self.ik_solver.link_names
        seed_state = get_joint_positions(robot_uid, joints)
        # seed_state = [0.0] * self.ik_solver.number_of_joints

        lower, upper = init_lower, init_upper
        if nearby_tolerance < INF:
            tolerance = nearby_tolerance * np.ones(len(joints))
            lower = np.maximum(lower, seed_state - tolerance)
            upper = np.minimum(upper, seed_state + tolerance)
        trac_ik_solver.set_joint_limits(lower, upper)

        (x, y, z), (rx, ry, rz, rw) = base_from_tip
        # TODO: can also adjust tolerances
        conf = trac_ik_solver.get_ik(seed_state, x, y, z, rx, ry, rz, rw)
        trac_ik_solver.set_joint_limits(init_lower, init_upper)
        if conf is None:
            return conf
        # if nearby_tolerance < INF:
        #    print(lower.round(3))
        #    print(upper.round(3))
        #    print(conf)
        #    print(get_difference(seed_state, conf).round(3))
        pp.set_joint_positions(robot_uid, joints, conf)
        # return pp.get_configuration(robot_uid)
        yield conf

    return IKInfo(track_ik_fn, trac_ik_solver.tip_link, trac_ik_solver.joint_names, []) # 'base_link_name',

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
            # notify('Warning! Go back to the command line now!')
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
    cartesian_attempts = options.get('cartesian_attempts') or 2
    reachable_range = options.get('reachable_range') or (0.2, 2.8) # (0.68, 2.83)
    # in meter
    frame_jump_tolerance = options.get('frame_jump_tolerance', FRAME_TOL*1e3)

    cartesian_move_group = options.get('cartesian_move_group') or GANTRY_ARM_GROUP
    # gantry_group = GANTRY_GROUP
    debug = options.get('debug', False)
    verbose = options.get('verbose', True)
    enforce_continuous = options.get('enforce_continuous', True)

    # * custom limits
    ik_joint_names = robot.get_configurable_joint_names(group=BARE_ARM_GROUP)
    tool_link_name = robot.get_end_effector_link_name(group=BARE_ARM_GROUP)
    ik_base_link_name = robot.get_base_link_name(group=GANTRY_ARM_GROUP)
    ik_tool_link = link_from_name(robot_uid, tool_link_name)

    gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)

    # * construct IK function
    sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
    # gantry_joint_names = get_gantry_control_joint_names(MAIN_ROBOT_ID)
    # gantry_joint_names = None
    # ik_info = IKInfo(sample_ik_fn, tool_link_name, ik_joint_names, gantry_joint_names) # 'base_link_name',
    # solver_type: Speed, Distance, Manipulation1, Manipulation2
    trac_ik_solver = IK(base_link=ik_base_link_name, tip_link=tool_link_name,
                        timeout=TRAC_IK_TIMEOUT, epsilon=TRAC_IK_TOL, solve_type="Speed",
                        urdf_string=pp.read(robot.attributes['pybullet']['cached_robot_filepath']))
    options['customized_ikinfo'] = get_solve_trac_ik_info(trac_ik_solver, robot_uid)
    # TODO switch to client IK
    # ikfast_fn = get_ik_fn_from_ikfast(ikfast_abb_irb4600_40_255.get_ik)
    # ik_solver = InverseKinematicsSolver(robot, move_group, ikfast_fn, base_frame, robotA_tool.frame)
    # client.planner.inverse_kinematics = ik_solver.inverse_kinematics_function()

    # * get target T0CF pose
    start_scene = process.get_movement_start_scene(movement)
    end_scene = process.get_movement_end_scene(movement)
    start_conf = process.get_movement_start_robot_config(movement)
    end_conf = process.get_movement_end_robot_config(movement)

    # * set start state
    try:
        set_state(client, robot, process, start_scene)
    except RuntimeError:
        return None

    # convert to meter
    start_t0cf_frame = start_scene[('robot', 'f')].copy()
    start_t0cf_frame.point *= 1e-3
    end_t0cf_frame = end_scene[('robot', 'f')].copy()
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

    # * verify FK consistency if start_conf is given
    with WorldSaver():
        if start_conf is not None:
            # if not start_conf.joint_names:
            #     start_conf.joint_names = gantry_arm_joint_names
            client.set_robot_configuration(robot, start_conf)
            start_tool_pose = get_link_pose(robot_uid, ik_tool_link)
            start_t0cf_frame_temp = frame_from_pose(start_tool_pose, scale=1)
            # in meter
            if not start_t0cf_frame_temp.__eq__(start_t0cf_frame, tol=frame_jump_tolerance):
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
            if not end_t0cf_frame_temp.__eq__(end_t0cf_frame, tol=frame_jump_tolerance):
                draw_pose(pose_from_frame(end_t0cf_frame_temp))
                wait_for_user('Pose from conf')
                draw_pose(pose_from_frame(end_t0cf_frame))
                wait_for_user('Pose from encoded frame')
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
    cart_conf = None

    # TODO custom limits
    # TODO try fixing IK and only use gantry for Cartesian movements
    if start_conf is None and end_conf is None:
        # * sample from a ball near the pose
        gantry_base_gen_fn = gantry_base_generator(client, robot, interp_frames[0], reachable_range=reachable_range, scale=1.0)
        for gi, base_conf in zip(range(gantry_attempts), gantry_base_gen_fn):
            if verbose:
                cprint('-- gantry sampling iter {}'.format(gi), 'magenta')
            samples_cnt += 1
            # * bare-arm IK sampler
            arm_conf_vals = sample_ik_fn(pose_from_frame(interp_frames[0], scale=1))
            # * iterate through all 6-axis IK solution
            for ik_iter, arm_conf_val in enumerate(arm_conf_vals):
                if verbose:
                    cprint('   -- IK iter {}'.format(ik_iter), 'magenta')
                if arm_conf_val is None:
                    continue
                gantry_arm_conf = Configuration(list(base_conf.joint_values) + list(arm_conf_val),
                    gantry_arm_joint_types, gantry_arm_joint_names)
                if not client.check_collisions(robot, gantry_arm_conf, options=options):
                    # * Cartesian planning, only for the chosen link (aka sub_conf)
                    for ci in range(cartesian_attempts):
                        # TODO pybullet IK struggles for dual arm setting
                        # see if we can create a x-y-z+6 axis subrobot clone
                        if verbose:
                            print('\tcartesian trial #{}'.format(ci))
                        cart_conf = client.plan_cartesian_motion(robot, interp_frames, start_configuration=gantry_arm_conf,
                            group=cartesian_move_group, options=options)
                        # TODO If using GANTRY joints only, the plan cartesian doesn't work...
                        # might need to simplify to use get_base_link,

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
        # else:
        #     # * fallback to ladder graph
        #     lm_options = options.copy()
        #     lm_options['planner_id'] = 'LadderGraph'
        #     lm_options['ik_function'] = _get_sample_bare_arm_ik_fn(client, robot)
        #     # pose -> list(conf values)
        #     cart_conf = client.plan_cartesian_motion(robot, interp_frames, start_configuration=gantry_arm_conf,
        #         group=BARE_ARM_GROUP, options=lm_options)
        #     if cart_conf:
        #         success_planner = 'LadderGraph'

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
        is_continuous = True
        traj = cart_conf
        if start_conf is not None:
            # TODO if return None?
            is_continuous = check_cartesian_conf_agreement(client, robot, start_conf, traj.points[0],
                conf1_tag='given start conf', conf2_tag='traj[0]', options=options, verbose=verbose)
        if end_conf is not None:
            is_continuous = check_cartesian_conf_agreement(client, robot, end_conf, traj.points[-1],
                conf1_tag='given end conf', conf2_tag='traj[-1]', options=options, verbose=verbose)
        if not is_continuous and enforce_continuous:
            return None
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
    gantry_attempts = options.get('gantry_attempts', 500)
    reachable_range = options.get('reachable_range', (0.2, 2.8))

    start_scene = process.get_movement_start_scene(movement)
    end_state = process.get_movement_end_scene(movement)
    orig_start_conf = process.get_movement_start_robot_config(movement)
    orig_end_conf = process.get_movement_end_robot_config(movement)

    # assert orig_end_conf is not None, 'End conf not provided in {}, which should never happen'.format(movement.short_summary)

    # * set start state
    try:
        set_state(client, robot, process, start_scene)
    except RuntimeError:
        return None

    sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
    gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)
    if orig_start_conf is None:
        if verbose:
            cprint('FreeMovement: Robot start conf is NOT specified in {}, we will sample an IK conf based on the given t0cp frame.'.format(movement.short_summary), 'yellow')
            # notify('Warning! Go back to the command line now!')
            # wait_for_user('Please press Enter to confirm.')
        # * sample from t0cp if no conf is provided for the robot
        start_t0cf_frame = start_scene[('robot', 'f')].copy()
        start_t0cf_frame.point *= 1e-3
        sample_found = False
        if start_t0cf_frame is not None:
            gantry_base_gen_fn = gantry_base_generator(client, robot, start_t0cf_frame, reachable_range=reachable_range, scale=1.0)
            for gantry_iter, base_conf in zip(range(gantry_attempts), gantry_base_gen_fn):
                # orig_start_conf = client.inverse_kinematics(robot, start_t0cf_frame, group=GANTRY_ARM_GROUP, options=options)
                # * bare-arm IK sampler
                arm_conf_vals = sample_ik_fn(pose_from_frame(start_t0cf_frame, scale=1))
                # * iterate through all 6-axis IK solution
                for arm_conf_val in arm_conf_vals:
                    if arm_conf_val is None:
                        continue
                    orig_start_conf = Configuration(list(base_conf.joint_values) + list(arm_conf_val),
                        gantry_arm_joint_types, gantry_arm_joint_names)
                    if debug:
                            client.set_robot_configuration(robot, orig_start_conf)
                            print(orig_start_conf.joint_values)
                            wait_if_gui('Sampled start conf')
                    if not client.check_collisions(robot, orig_start_conf, options=options):
                        sample_found = True
                        if verbose: print('Start conf sample found after {} gantry iters.'.format(gantry_iter))
                        break
                if sample_found:
                    break
            else:
                cprint('No start robot IK conf can be found for {} after {} attempts, solve fails.'.format(
                    movement.short_summary, gantry_attempts), 'red')
                return None
        else:
            cprint('No robot start frame is specified in {}, Underspecified problem, solve fails.'.format(movement.short_summary), 'red')
            return None

    # TODO clean up code and make a function for start/end conf sampling
    if orig_end_conf is None:
        if verbose:
            cprint('FreeMovement: Robot end conf is NOT specified in {}, we will sample an IK conf based on the given t0cp frame.'.format(movement.short_summary), 'yellow')
            # notify('Warning! Go back to the command line now!')
            # wait_for_user('Please press Enter to confirm.')
        # * sample from t0cp if no conf is provided for the robot
        end_t0cf_frame = copy(end_state['robot'].current_frame)
        end_t0cf_frame.point *= 1e-3
        sample_found = False
        if end_t0cf_frame is not None:
            gantry_base_gen_fn = gantry_base_generator(client, robot, end_t0cf_frame, reachable_range=reachable_range, scale=1.0)
            for gantry_iter, base_conf in zip(range(gantry_attempts), gantry_base_gen_fn):
                # orig_end_conf = client.inverse_kinematics(robot, end_t0cf_frame, group=GANTRY_ARM_GROUP, options=options)
                # * bare-arm IK sampler
                arm_conf_vals = sample_ik_fn(pose_from_frame(end_t0cf_frame, scale=1))
                # * iterate through all 6-axis IK solution
                for arm_conf_val in arm_conf_vals:
                    if arm_conf_val is None:
                        continue
                    orig_end_conf = Configuration(list(base_conf.joint_values) + list(arm_conf_val),
                        gantry_arm_joint_types, gantry_arm_joint_names)
                    if debug:
                            client.set_robot_configuration(robot, orig_end_conf)
                            print(orig_end_conf.joint_values)
                            wait_if_gui('Sampled end conf')
                    if not client.check_collisions(robot, orig_end_conf, options=options):
                        sample_found = True
                        if verbose: print('End conf sample found after {} gantry iters.'.format(gantry_iter))
                        break
                if sample_found:
                    break
            else:
                cprint('No end robot IK conf can be found for {} after {} attempts, solve fails.'.format(
                    movement.short_summary, gantry_attempts), 'red')
                return None
        else:
            cprint('No robot end frame is specified in {}, Underspecified problem, solve fails.'.format(movement.short_summary), 'red')
            return None

    start_conf = orig_start_conf
    end_conf = orig_end_conf

    # * custom limits
    # custom_limits = get_gantry_robot_custom_limits(MAIN_ROBOT_ID)
    # if 'custom_limits' not in options:
    #     options.update({'custom_limits': custom_limits})

    # https://github.com/yijiangh/compas_fab_pychoreo/blob/7afe6bc74f2c14caf79f573bc6a6fd95442b779a/examples/itj/stream.py#L459
    # TODO insert cartesian segment before and after the free motion
    robot_uid = client.get_robot_pybullet_uid(robot)
    tool_link_name = robot.get_end_effector_link_name(group=BARE_ARM_GROUP)
    tool_link = link_from_name(robot_uid, tool_link_name)
    client.set_robot_configuration(robot, start_conf)
    start_pose = get_link_pose(robot_uid, tool_link)
    client.set_robot_configuration(robot, end_conf)
    end_pose = get_link_pose(robot_uid, tool_link)

    # sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
    # gantry_joint_names = get_gantry_control_joint_names(MAIN_ROBOT_ID)
    # ik_joint_names = robot.get_configurable_joint_names(group=BARE_ARM_GROUP)
    # ik_info = IKInfo(sample_ik_fn, tool_link_name, ik_joint_names, gantry_joint_names) # 'base_link_name',
    # options['customized_ikinfo'] = ik_info

    # * use previous movement's FK frame vector as a guide vector
    ik_base_link_name = robot.get_base_link_name(group=GANTRY_ARM_GROUP)
    trac_ik_solver = IK(base_link=ik_base_link_name, tip_link=tool_link_name,
                        timeout=TRAC_IK_TIMEOUT, epsilon=TRAC_IK_TOL, solve_type="Speed",
                        urdf_string=pp.read(robot.attributes['pybullet']['cached_robot_filepath']))
    options['customized_ikinfo'] = get_solve_trac_ik_info(trac_ik_solver, robot_uid)

    current_mid = process.movements.index(movement)
    if current_mid-1 < 0 or not any([isinstance(process.movements[current_mid-1], m_type) \
            for m_type in [RoboticLinearMovement, RoboticClampSyncLinearMovement]]):
        # no retraction needed
        start_retraction_vector = None
    else:
        prev_movement = process.movements[current_mid-1]
        prev_start_state = process.get_movement_start_scene(prev_movement)
        prev_end_state = process.get_movement_end_scene(prev_movement)
        # convert to meter
        # ? attach -> retreat, vector = retreat(end) - attach(start)
        start_retraction_vector = get_unit_vector(list(prev_end_state['robot'].current_frame.point - prev_start_state['robot'].current_frame.point))

    if current_mid+1 > len(process.movements) or not any([isinstance(process.movements[current_mid+1], m_type) \
            for m_type in [RoboticLinearMovement, RoboticClampSyncLinearMovement]]):
        # no retraction needed
        end_retraction_vector = None
    else:
        next_movement = process.movements[current_mid+1]
        next_start_state = process.get_movement_start_scene(next_movement)
        next_end_state = process.get_movement_end_scene(next_movement)
        # convert to meter
        # ? retreat -> attach, vector = retreat(start) - attach(end)
        end_retraction_vector = get_unit_vector(list(next_start_state['robot'].current_frame.point - next_end_state['robot'].current_frame.point))

    retraction_candidates = options.get('max_free_retraction_distance', np.linspace(0, 0.1, 5))
    traj = None
    retraction_candidates = [0.0] if start_retraction_vector is None and end_retraction_vector is None else retraction_candidates
    for retraction_dist in retraction_candidates:
        if verbose:
            print('Free motion: trying retraction dist {:.4f}'.format(retraction_dist))
        start_cart_traj = None
        end_cart_traj = None
        if abs(retraction_dist) > 1e-6:
            if start_retraction_vector is not None:
                retract_start_pose = multiply(pp.Pose(retraction_dist*pp.Point(*start_retraction_vector)), start_pose)
                if debug:
                    with WorldSaver():
                        draw_point(prev_start_state['robot'].current_frame.point*1e-3)
                        draw_point(prev_end_state['robot'].current_frame.point*1e-3)
                        draw_pose(start_pose, length=0.1)
                        draw_pose(retract_start_pose, length=0.05)
                        client.set_robot_configuration(robot, process.initial_state[process.robot_config_key])
                        wait_if_gui('Retract pose drawn. start vec: {}'.format(start_retraction_vector))
                start_cart_traj = client.plan_cartesian_motion(robot, [frame_from_pose(start_pose), frame_from_pose(retract_start_pose)], start_configuration=start_conf,
                    group=GANTRY_ARM_GROUP, options=options)
                if not start_cart_traj:
                    if verbose: cprint('No start cart traj found.', 'red')
                    # continue
                else:
                    if verbose: cprint('Start cart traj found.', 'green')

            if end_retraction_vector is not None:
                retract_end_pose = multiply(pp.Pose(retraction_dist*pp.Point(*end_retraction_vector)), end_pose)
                if debug:
                    with WorldSaver():
                        draw_point(next_start_state['robot'].current_frame.point*1e-3)
                        draw_point(next_end_state['robot'].current_frame.point*1e-3)
                        draw_pose(end_pose, length=0.1)
                        draw_pose(retract_end_pose, length=0.05)
                        client.set_robot_configuration(robot, process.initial_state[process.robot_config_key])
                        wait_if_gui('Retract pose drawn. end vec: {}'.format(end_retraction_vector))
                end_cart_traj = client.plan_cartesian_motion(robot, [frame_from_pose(end_pose), frame_from_pose(retract_end_pose)], start_configuration=end_conf,
                    group=GANTRY_ARM_GROUP, options=options)
                if not end_cart_traj:
                    if verbose: cprint('No end cart traj found.', 'red')
                else:
                    if verbose: cprint('End cart traj found.', 'green')
                    end_cart_traj = reverse_trajectory(end_cart_traj)
        # else:
        # # ! skip no -buffe attempt
        #     continue

        with LockRenderer():
            new_start_conf = start_conf if start_cart_traj is None else start_cart_traj.points[-1]
            new_end_conf = end_conf if end_cart_traj is None else end_cart_traj.points[0]
            goal_constraints = robot.constraints_from_configuration(new_end_conf, [0.01], [0.01], group=GANTRY_ARM_GROUP)
            free_traj = client.plan_motion(robot, goal_constraints, start_configuration=new_start_conf, group=GANTRY_ARM_GROUP,
                                      options=options)
        if free_traj is not None:
            full_trajs = []
            if start_cart_traj:
                full_trajs.append(start_cart_traj)
            full_trajs.append(free_traj)
            if end_cart_traj:
                full_trajs.append(end_cart_traj)
            traj = merge_trajectories(full_trajs)
            break

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
        goal_constraints = robot.constraints_from_configuration(end_conf, [0.01], [0.01], group=GANTRY_ARM_GROUP)
        traj = client.plan_motion(robot, goal_constraints, start_configuration=start_conf, group=GANTRY_ARM_GROUP,
                                  options=d_options)
        if lockrenderer:
            lockrenderer = LockRenderer()

    return fill_in_tool_path(client, robot, traj, group=GANTRY_ARM_GROUP)
