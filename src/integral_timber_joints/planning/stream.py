import numpy as np
from termcolor import colored
from copy import copy
from itertools import product
from typing import Optional

from compas.geometry import Frame, distance_point_point, Transformation
from compas_fab.robots import Configuration, Robot, Trajectory

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotClampAssemblyProcess, Movement, RobotScrewdriverSyncLinearMovement
from integral_timber_joints.process.state import SceneState

import pybullet_planning as pp
from pybullet_planning import GREY
from pybullet_planning import link_from_name, get_link_pose, draw_pose, multiply, \
    joints_from_names, LockRenderer, WorldSaver, wait_for_user, joint_from_name, wait_if_gui, has_gui
from pybullet_planning import link_from_name, sample_tool_ik, is_pose_close
from pybullet_planning import INF, invert, get_joint_positions, get_unit_vector, draw_point

from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.utils import is_configurations_close, is_frames_close

import ikfast_abb_irb4600_40_255

from integral_timber_joints.planning.robot_setup import BARE_ARM_GROUP, GANTRY_ARM_GROUP, GANTRY_GROUP, \
    USE_TRACK_IK
from integral_timber_joints.planning.utils import reverse_trajectory, merge_trajectories, FRAME_TOL
from integral_timber_joints.planning.state import set_state, gantry_base_generator
from integral_timber_joints.planning.utils import notify, LOGGER

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

def compute_linear_movement(client: PyChoreoClient, robot: Robot, process: RobotClampAssemblyProcess, movement: Movement, options=None, diagnosis=False):
    # assert isinstance(movement, RoboticLinearMovement) or \
    #     isinstance(movement, RoboticClampSyncLinearMovement) or \
    #     isinstance(movement, RobotScrewdriverSyncLinearMovement)
    robot_uid = client.get_robot_pybullet_uid(robot)

    # * options
    # sampling attempts
    options = options or {}
    gantry_attempts = options.get('gantry_attempts') or 10
    cartesian_attempts = options.get('cartesian_attempts') or 2
    reachable_range = options.get('reachable_range') or (0.2, 2.8) # (0.68, 2.83)

    cartesian_move_group = options.get('cartesian_move_group') or GANTRY_ARM_GROUP
    # gantry_group = GANTRY_GROUP
    debug = options.get('debug', False)
    verbose = options.get('verbose', True)

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

    # TODO try except loading trac_ik
    if USE_TRACK_IK:
        from trac_ik_python.trac_ik_wrap import TRAC_IK
        from trac_ik_python.trac_ik import IK
        trac_ik_solver = IK(base_link=ik_base_link_name, tip_link=tool_link_name,
                            timeout=TRAC_IK_TIMEOUT, epsilon=TRAC_IK_TOL, solve_type="Speed",
                            urdf_string=pp.read(robot.attributes['pybullet']['cached_robot_filepath']))
        options['customized_ikinfo'] = get_solve_trac_ik_info(trac_ik_solver, robot_uid)
    else:
        # TODO switch to client IK
        # ikfast_fn = get_ik_fn_from_ikfast(ikfast_abb_irb4600_40_255.get_ik)
        # ik_solver = InverseKinematicsSolver(robot, move_group, ikfast_fn, base_frame, robotA_tool.frame)
        # client.planner.inverse_kinematics = ik_solver.inverse_kinematics_function()
        pass

    # * get target T0CF pose
    start_scene = process.get_movement_start_scene(movement)
    end_scene = process.get_movement_end_scene(movement)
    start_conf = process.get_movement_start_robot_config(movement)
    end_conf = process.get_movement_end_robot_config(movement)

    # * special warning when both start and end config is set
    if start_conf is not None and end_conf is not None:
        LOGGER.warn("Both start and end config is fixed for Linear Movement #{}. This is likely overconstrained.".format(movement.movement_id))

    # * set start state
    if not set_state(client, robot, process, start_scene, options=options):
        LOGGER.error('Compute linear movement: set start state error.')
        return None

    # * get target T0CF pose
    # convert to meter
    start_t0cf_frame = start_scene[('robot', 'f')].copy()
    start_t0cf_frame.point *= 1e-3
    end_t0cf_frame = end_scene[('robot', 'f')].copy()
    end_t0cf_frame.point *= 1e-3

    # * verify FK consistency if start_conf is given
    with WorldSaver():
        if start_conf is not None:
            client.set_robot_configuration(robot, start_conf)
            start_tool_pose = get_link_pose(robot_uid, ik_tool_link)
            start_t0cf_frame_temp = frame_from_pose(start_tool_pose, scale=1)
            # in meter
            if not is_frames_close(start_t0cf_frame, start_t0cf_frame_temp, options=options):
                LOGGER.error('compute_linear_movement : start conf FK inconsistent ({:.5f} m) with given current frame in start state.'.format(
                        distance_point_point(start_t0cf_frame_temp.point, start_t0cf_frame.point)))
                return None
        if end_conf is not None:
            client.set_robot_configuration(robot, end_conf)
            end_tool_pose = get_link_pose(robot_uid, ik_tool_link)
            end_t0cf_frame_temp = frame_from_pose(end_tool_pose, scale=1)
            if not is_frames_close(end_t0cf_frame_temp, end_t0cf_frame, options=options):
                LOGGER.error('compute_linear_movement : end conf FK inconsistent ({:.5f} m) with given current frame in end state.'.format(
                        distance_point_point(end_t0cf_frame_temp.point, end_t0cf_frame.point)))
                return None

    # * ACM setup
    temp_name = '_tmp'
    for o1_name, o2_name in movement.allowed_collision_matrix:
        o1_bodies = client._get_bodies('^{}$'.format(o1_name))
        o2_bodies = client._get_bodies('^{}$'.format(o2_name))
        for parent_body, child_body in product(o1_bodies, o2_bodies):
            client.extra_disabled_collision_links[temp_name].add(
                ((parent_body, None), (child_body, None))
            )

    interp_frames = [start_t0cf_frame, end_t0cf_frame]

    solution_found = False
    samples_cnt = ik_failures = path_failures = 0
    planner_id = options.get('planner_id', 'IterativeIK')
    cart_conf = None

    # TODO custom limits
    # TODO try fixing IK and only use gantry for Cartesian movements
    if start_conf is None and end_conf is None:
        # * Both stand and end conf are not specified
        # * sample from a ball near the pose
        gantry_base_gen_fn = gantry_base_generator(client, robot, interp_frames[0], reachable_range=reachable_range, scale=1.0)
        for gi, base_conf in zip(range(gantry_attempts), gantry_base_gen_fn):
            # if verbose:
            #     cprint('-- gantry sampling iter {}'.format(gi), 'magenta')
            samples_cnt += 1
            # * bare-arm IK sampler
            arm_conf_vals = sample_ik_fn(pose_from_frame(interp_frames[0], scale=1))
            # * iterate through all 6-axis IK solution
            for ik_iter, arm_conf_val in enumerate(arm_conf_vals):
                # if verbose:
                #     cprint('   -- IK iter {}'.format(ik_iter), 'magenta')
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
                            LOGGER.debug('\tcartesian trial #{}'.format(ci))
                        cart_conf = client.plan_cartesian_motion(robot, interp_frames, start_configuration=gantry_arm_conf,
                            group=cartesian_move_group, options=options)
                        # TODO If using GANTRY joints only, the plan cartesian doesn't work...
                        # might need to simplify to use get_base_link,

                        if cart_conf is not None:
                            solution_found = True
                            if verbose:
                                LOGGER.debug(colored('Plan found by {}! After {} ik, {} path failure over {} samples.'.format
                                (planner_id, ik_failures, path_failures, samples_cnt), 'green'))
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
                LOGGER.debug(colored('Cartesian Path planning failure ({}) after {} attempts | {} due to IK, {} due to Cart.'.format(
                    planner_id, samples_cnt, ik_failures, path_failures), 'yellow'))
    else:
        # * At least one of start/end conf is specified
        if start_conf is not None and end_conf is not None:
            LOGGER.error('Both start/end confs are pre-specified, problem might be too stiff to be solved.')
        if start_conf:
            if verbose:
                LOGGER.debug(colored('One-sided Cartesian planning : start conf set, forward mode', 'blue'))
            forward = True
            gantry_arm_conf = start_conf
        else:
            if verbose:
                LOGGER.debug(colored('One-sided Cartesian planning : end conf set, backward mode', 'blue'))
            forward = False
            gantry_arm_conf = end_conf
            interp_frames = interp_frames[::-1]

        samples_cnt = 0
        for ci in range(cartesian_attempts):
            if verbose:
                LOGGER.debug('\tcartesian trial #{}'.format(ci))
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
                LOGGER.debug(colored('Cartesian Path planning (w/ prespecified st or end conf) failure after\n{} attempts of {} + 1 LadderGraph attempt.'.format(
                    planner_id, samples_cnt), 'yellow'))
        else:
            if not forward:
                cart_conf = reverse_trajectory(cart_conf)
            if verbose:
                LOGGER.debug(colored('Plan found by {}! After {} path failure (by {}) over {} samples.'.format(
                    success_planner, path_failures, planner_id, samples_cnt), 'green'))

    if cart_conf is None and diagnosis:
        lockrenderer = options.get('lockrenderer', None)
        if lockrenderer is not None:
            lockrenderer.restore()
        LOGGER.debug('Start diagnosis.')
        LOGGER.debug(f'movement.allowed_collision_matrix: {movement.allowed_collision_matrix}')
        LOGGER.debug(f'extra_disabled_collision_links: {client.extra_disabled_collision_links}')
        # client._print_object_summary()
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
        if start_conf is not None and not is_configurations_close(start_conf, traj.points[0], options=options):
            LOGGER.error('compute_linear_motion: start conf disagreement.')
            return None
        if end_conf is not None and not is_configurations_close(end_conf, traj.points[-1], options=options):
            LOGGER.error('compute_linear_motion: end conf disagreement.')
            return None
    # else:
    #     LOGGER.info('No linear movement found for {}.'.format(movement.short_summary))
    return traj

##############################

def compute_free_movement(client: PyChoreoClient, robot: Robot, process: RobotClampAssemblyProcess, movement: Movement,
        options=None, diagnosis=False):
    # assert isinstance(movement, RoboticFreeMovement)
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

    # * set start state
    if not set_state(client, robot, process, start_scene, options=options):
        LOGGER.error('Compute linear movement: set start state error.')
        return None

    sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
    gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)
    if orig_start_conf is None:
        LOGGER.warning('FreeMovement: Robot start conf is NOT specified in {}, we will sample an IK conf based on the given t0cp frame.'.format(movement.short_summary))
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
                    if not client.check_collisions(robot, orig_start_conf, options=options):
                        sample_found = True
                        if verbose: LOGGER.debug('Start conf sample found after {} gantry iters.'.format(gantry_iter))
                        # if debug:
                        #     client.set_robot_configuration(robot, orig_start_conf)
                        #     LOGGER.debug(orig_start_conf.joint_values)
                        #     wait_if_gui('Sampled start conf')
                        break
                if sample_found:
                    break
            else:
                LOGGER.error(colored('No start robot IK conf can be found for {} after {} attempts, solve fails.'.format(
                    movement.short_summary, gantry_attempts), 'red'))
                return None
        else:
            LOGGER.error('No robot start frame is specified in {}, Underspecified problem, solve fails.'.format(movement.short_summary))
            return None

    # TODO clean up code and make a function for start/end conf sampling
    if orig_end_conf is None:
        LOGGER.debug('FreeMovement: Robot end conf is NOT specified in {}, we will sample an IK conf based on the given t0cp frame.'.format(movement.short_summary))
            # notify('Warning! Go back to the command line now!')
            # wait_for_user('Please press Enter to confirm.')
        # * sample from t0cp if no conf is provided for the robot
        end_t0cf_frame = copy(end_state[('robot', 'f')])
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
                    if not client.check_collisions(robot, orig_end_conf, options=options):
                        sample_found = True
                        if verbose: LOGGER.debug('End conf sample found after {} gantry iters.'.format(gantry_iter))
                        if debug:
                                client.set_robot_configuration(robot, orig_end_conf)
                                # LOGGER.debug(orig_end_conf.joint_values)
                                wait_if_gui('Sampled end conf')
                        break
                if sample_found:
                    break
            else:
                LOGGER.error(colored('No end robot IK conf can be found for {} after {} attempts, solve fails.'.format(
                    movement.short_summary, gantry_attempts), 'red'))
                return None
        else:
            LOGGER.error('No robot end frame is specified in {}, Underspecified problem, solve fails.'.format(movement.short_summary))
            return None

    start_conf = orig_start_conf
    end_conf = orig_end_conf

    # * computer retraction buffer motion before/after the free motion to avoid narrow passages
    robot_uid = client.get_robot_pybullet_uid(robot)
    tool_link_name = robot.get_end_effector_link_name(group=BARE_ARM_GROUP)
    tool_link = link_from_name(robot_uid, tool_link_name)
    client.set_robot_configuration(robot, start_conf)
    start_pose = get_link_pose(robot_uid, tool_link)
    client.set_robot_configuration(robot, end_conf)
    end_pose = get_link_pose(robot_uid, tool_link)

    # * use previous movement's FK frame vector as a guide vector
    if USE_TRACK_IK:
        ik_base_link_name = robot.get_base_link_name(group=GANTRY_ARM_GROUP)
        trac_ik_solver = IK(base_link=ik_base_link_name, tip_link=tool_link_name,
                            timeout=TRAC_IK_TIMEOUT, epsilon=TRAC_IK_TOL, solve_type="Speed",
                            urdf_string=pp.read(robot.attributes['pybullet']['cached_robot_filepath']))
        options['customized_ikinfo'] = get_solve_trac_ik_info(trac_ik_solver, robot_uid)
    else:
        pass
        # sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
        # gantry_joint_names = get_gantry_control_joint_names(MAIN_ROBOT_ID)
        # ik_joint_names = robot.get_configurable_joint_names(group=BARE_ARM_GROUP)
        # ik_info = IKInfo(sample_ik_fn, tool_link_name, ik_joint_names, gantry_joint_names) # 'base_link_name',
        # options['customized_ikinfo'] = ik_info

    # * local vectors for retraction motions, default to not using it
    tmp_xyz = options.get('retraction_vectors', [[0,0,0]])
    # list(np.vstack([np.eye(3), -np.eye(3)]))
    retraction_distances = options.get('max_free_retraction_distances', [0.0])
    # np.linspace(0, 0.1, 3)

    start_retraction_vectors = tmp_xyz
    end_retraction_vectors = tmp_xyz

    # if debug:
    #     def debug_buffer_linear_motion(target_pose, retract_pose, dist, retract_v, msg=''):
    #         pp.remove_all_debug()
    #         pp.camera_focus_on_point(retract_pose[0])
    #         with WorldSaver():
    #             draw_pose(target_pose, length=0.1)
    #             draw_pose(retract_pose, length=0.05)
    #             client.set_robot_configuration(robot, process.initial_state[process.robot_config_key])
    #             wait_if_gui('Retract pose drawn at {} (bigger=1st). start vec: {}, dist {:.4f}'.format(msg, retract_v, dist))

    traj = None
    start_cart_traj = None
    end_cart_traj = None
    free_traj = None
    start_retract_msg = end_retract_msg = ''
    # ! retraction vector to determine the start retraction pose
    for start_v in start_retraction_vectors:
        # ! retraction vector magnitude to determine the start retraction pose
        for start_dist in retraction_distances:
            start_cart_traj = None
            if abs(start_dist) > 1e-6:
                start_retract_msg = '{:.4f} | {}'.format(start_dist, start_v)
                if verbose:
                    LOGGER.debug('Free motion | START linear buffer: trying retraction dist {}'.format(start_retract_msg))
                start_v /= np.linalg.norm(start_v)
                retract_start_pose = multiply(pp.Pose(start_dist*pp.Point(*start_v)), start_pose)
                # if debug:
                #     debug_buffer_linear_motion(start_pose, retract_start_pose, start_dist, start_v, 'start')
                # TODO try floating attachment only planning before full cartesian planning.keys())?
                start_cart_traj = client.plan_cartesian_motion(robot, [frame_from_pose(start_pose), frame_from_pose(retract_start_pose)], start_configuration=start_conf,
                    group=GANTRY_ARM_GROUP, options=options)
                if not start_cart_traj:
                    if verbose: LOGGER.debug(colored('No start cart traj found.', 'red'))
                    # ! continue to next start_dist trail
                    continue
                else:
                    if verbose: LOGGER.debug(colored('Start cart traj found.', 'green'))
            else:
                start_retract_msg = 'No retraction motion specified.'
                LOGGER.debug(colored('ST: '+start_retract_msg, 'green'))

            end_cart_traj = None
            # ! retraction vector to determine the end retraction pose
            for end_v in end_retraction_vectors:
                # ! retraction magnitude to determine the end retraction pose
                for end_dist in retraction_distances:
                    end_cart_traj = None
                    if abs(end_dist) > 1e-6:
                        end_retract_msg = '{:.4f} | {}'.format(end_dist, end_v)
                        if verbose:
                            LOGGER.debug('- Free motion | END linear buffer: trying retraction dist {}'.format(end_retract_msg))
                        end_v /= np.linalg.norm(end_v)
                        retract_end_pose = multiply(pp.Pose(end_dist*pp.Point(*end_v)), end_pose)
                        # if debug:
                        #     debug_buffer_linear_motion(end_pose, retract_end_pose, end_dist, end_v, 'end')

                        end_cart_traj = client.plan_cartesian_motion(robot, [frame_from_pose(end_pose), frame_from_pose(retract_end_pose)], start_configuration=end_conf,
                            group=GANTRY_ARM_GROUP, options=options)
                        if not end_cart_traj:
                            if verbose: LOGGER.debug(colored('No end cart traj found.', 'red'))
                            # ! continue to next end_dist trial
                            continue
                        else:
                            if verbose: LOGGER.debug(colored('End cart traj found.', 'green'))
                            end_cart_traj = reverse_trajectory(end_cart_traj)
                    else:
                        end_retract_msg = 'No retraction motion specified.'
                        LOGGER.debug(colored('END: '+end_retract_msg, 'green'))

                    new_start_conf = start_conf if start_cart_traj is None else start_cart_traj.points[-1]
                    new_end_conf = end_conf if end_cart_traj is None else end_cart_traj.points[0]
                    # if debug:
                    #     client.set_robot_configuration(robot, new_start_conf)
                    #     # print('start conf: ', new_start_conf)
                    #     wait_if_gui('Start conf after retraction.')

                    #     client.set_robot_configuration(robot, new_end_conf)
                    #     # print('end conf: ', new_end_conf)
                    #     wait_if_gui('End conf after retraction.')

                    with LockRenderer(not diagnosis):
                        goal_constraints = robot.constraints_from_configuration(new_end_conf, [0.01], [0.01], group=GANTRY_ARM_GROUP)
                        # d_options = options.copy()
                        # if diagnosis:
                        #     d_options['diagnosis'] = True
                        free_traj = client.plan_motion(robot, goal_constraints, start_configuration=new_start_conf,
                            group=GANTRY_ARM_GROUP, options=options)

                    retraction_msg = "(ST {} ; END {})".format(start_retract_msg, end_retract_msg)
                    if free_traj is not None:
                        # ! concatenate three trajectories
                        full_trajs = []
                        if start_cart_traj:
                            full_trajs.append(start_cart_traj)
                        full_trajs.append(free_traj)
                        if end_cart_traj:
                            full_trajs.append(end_cart_traj)
                        traj = merge_trajectories(full_trajs)

                        if not is_configurations_close(start_conf, traj.points[0], options=options):
                            LOGGER.error('compute_free_motion: start conf disagreement | {}.'.format(retraction_msg))
                            return None
                        if not is_configurations_close(end_conf, traj.points[-1], options=options):
                            LOGGER.error('compute_free_motion: end conf disagreement. | {}'.format(retraction_msg))
                            return None

                        # ! trajectory found!
                        if verbose:
                            LOGGER.debug(colored('Free movement found for {} under current retraction {}!'.format(movement.short_summary, retraction_msg), 'green'))
                        return traj
                    else:
                        if verbose:
                            LOGGER.debug('No free motion found under current retraction: {}.'.format(retraction_msg))
                            LOGGER.debug('='*10)

    if traj is None and diagnosis:
        client._print_object_summary()
        lockrenderer = options.get('lockrenderer', None)
        if lockrenderer:
            lockrenderer.restore()
        LOGGER.debug('Start diagnosis.')
        d_options = options.copy()
        d_options['diagnosis'] = True
        goal_constraints = robot.constraints_from_configuration(end_conf, [0.01], [0.01], group=GANTRY_ARM_GROUP)
        traj = client.plan_motion(robot, goal_constraints, start_configuration=start_conf, group=GANTRY_ARM_GROUP,
                                  options=d_options)
        if lockrenderer:
            lockrenderer = LockRenderer()
    # if verbose:
    #     LOGGER.info('No free movement found for {}.'.format(movement.short_summary))
    return None

def sample_config(client: PyChoreoClient, robot: Robot, target_frame: Frame, options=None):
    """Function used for sampling start and end configuration for robot movement"""
    # * options
    options = options or {}
    debug = options.get('debug', False)
    verbose = options.get('verbose', True)
    gantry_attempts = options.get('gantry_attempts', 500)
    reachable_range = options.get('reachable_range', (0.2, 2.8))

    gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)
    sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)

    # * Check input
    if target_frame is None:
        LOGGER.error('No frame is specified, Underspecified problem, sample_config cannot continue.')
        return None

    # * Scale frame and sample
    target_frame = target_frame.copy()
    target_frame.point *= 1e-3

    gantry_base_gen_fn = gantry_base_generator(client, robot, target_frame, reachable_range=reachable_range, scale=1.0)
    for gantry_iter, base_conf in zip(range(gantry_attempts), gantry_base_gen_fn):

        # * bare-arm IK sampler
        arm_conf_vals = sample_ik_fn(pose_from_frame(target_frame, scale=1))

        # * iterate through all 6-axis IK solution
        for arm_conf_val in arm_conf_vals:
            if arm_conf_val is None:
                continue
            configuration = Configuration(list(base_conf.joint_values) + list(arm_conf_val),
                gantry_arm_joint_types, gantry_arm_joint_names)

            # * If no collision for this Gantry + IKFast combo, return configuration
            if not client.check_collisions(robot, configuration, options=options):
                sample_found = True
                if verbose: LOGGER.debug('Config found after {} gantry iters by sample_config().'.format(gantry_iter))
                # if debug:
                #     client.set_robot_configuration(robot, orig_start_conf)
                #     LOGGER.debug(orig_start_conf.joint_values)
                #     wait_if_gui('Sampled start conf')
                return configuration
    else:
        LOGGER.debug('Config not found after {} gantry iters by sample_config().'.format(gantry_iter))
        return None

def compute_free_movement_with_waypoints(client: PyChoreoClient, robot: Robot, process: RobotClampAssemblyProcess, movement: RoboticFreeMovement,
        options=None, diagnosis=False):
    # assert isinstance(movement, RoboticFreeMovement)
    options = options or {}
    # * options
    debug = options.get('debug', False)
    verbose = options.get('verbose', True)
    # * sampling attempts, needed only if start/end conf not specified

    start_scene = process.get_movement_start_scene(movement)
    end_scene = process.get_movement_end_scene(movement)
    orig_start_conf = process.get_movement_start_robot_config(movement)
    orig_end_conf = process.get_movement_end_robot_config(movement)

    # * set start state
    if not set_state(client, robot, process, start_scene, options=options):
        LOGGER.error('Compute linear movement: set start state error.')
        return None

    sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
    gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)

    # * Sample Start and end configurations
    if orig_start_conf is None:
        LOGGER.info('FreeMovement: Start Conf not specified in {}, will sample IK conf based frame.'.format(movement.short_summary))
        orig_start_conf = sample_config(client, robot, start_scene[('robot', 'f')], options)
        if orig_start_conf is None:
            LOGGER.warning(colored('Start Config IK cannot be found for {}, solve fails.'.format(movement.short_summary), 'red'))
            return None

    # * Sample Start and end configurations
    if orig_end_conf is None:
        LOGGER.info('FreeMovement: End Conf not specified in {}, will sample IK conf based frame.'.format(movement.short_summary))
        orig_end_conf = sample_config(client, robot, end_scene[('robot', 'f')], options)
        if orig_end_conf is None:
            LOGGER.warning(colored('End Config IK cannot be found for {}, solve fails.'.format(movement.short_summary), 'red'))
            return None



    # * computer retraction buffer motion before/after the free motion to avoid narrow passages
    robot_uid = client.get_robot_pybullet_uid(robot)
    tool_link_name = robot.get_end_effector_link_name(group=BARE_ARM_GROUP)

    # * Not sure what this is
    if USE_TRACK_IK:
        ik_base_link_name = robot.get_base_link_name(group=GANTRY_ARM_GROUP)
        trac_ik_solver = IK(base_link=ik_base_link_name, tip_link=tool_link_name,
                            timeout=TRAC_IK_TIMEOUT, epsilon=TRAC_IK_TOL, solve_type="Speed",
                            urdf_string=pp.read(robot.attributes['pybullet']['cached_robot_filepath']))
        options['customized_ikinfo'] = get_solve_trac_ik_info(trac_ik_solver, robot_uid)
    else:
        pass

    # * Plan each portion
    waypoints = movement.intermediate_planning_waypoint
    traj_segments = []
    start_end_config_pairs = list(zip([orig_start_conf] + waypoints, waypoints + [orig_end_conf]))
    for i, (start_conf, end_conf) in enumerate(start_end_config_pairs):

        # Extract only the relavent joint values
        end_conf = Configuration([end_conf[joint_name] for joint_name in gantry_arm_joint_names], gantry_arm_joint_types, gantry_arm_joint_names)

        with LockRenderer(not diagnosis):
            LOGGER.info('Planning waypoint segment {} of {} of {}'.format(i + 1, len(start_end_config_pairs), movement.movement_id))
            goal_constraints = robot.constraints_from_configuration(end_conf, [0.01], [0.01], group=GANTRY_ARM_GROUP)
            traj_segment = client.plan_motion(robot, goal_constraints, start_configuration=start_conf, group=GANTRY_ARM_GROUP,
                options=options) # type: Trajectory
            if traj_segment is None:
                LOGGER.warning(colored('No path for waypoint segment {} of {} of {}, free movement planning fail.'.format(i + 1, len(start_end_config_pairs), movement.movement_id), 'red'))
                return None
            traj_segments.append(traj_segment)

    # * trajectory found!
    traj = merge_trajectories(traj_segments)
    if verbose:
        LOGGER.debug(colored('Free movement found for {} with {} segments!'.format(movement.short_summary, len(start_end_config_pairs)), 'green'))
    return traj


    if traj is None and diagnosis:
        client._print_object_summary()
        lockrenderer = options.get('lockrenderer', None)
        if lockrenderer:
            lockrenderer.restore()
        LOGGER.debug('Start diagnosis.')
        d_options = options.copy()
        d_options['diagnosis'] = True
        goal_constraints = robot.constraints_from_configuration(end_conf, [0.01], [0.01], group=GANTRY_ARM_GROUP)
        traj = client.plan_motion(robot, goal_constraints, start_configuration=start_conf, group=GANTRY_ARM_GROUP,
                                  options=d_options)
        if lockrenderer:
            lockrenderer = LockRenderer()
    # if verbose:
    #     LOGGER.info('No free movement found for {}.'.format(movement.short_summary))
    return None