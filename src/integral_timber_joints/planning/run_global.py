import os
import time
import logging
import numpy as np
import argparse
from tqdm import tqdm
import pybullet_planning as pp
from pybullet_planning.motion_planners.utils import elapsed_time
from typing import Tuple, List, Type, Dict

from copy import deepcopy

from pybullet_planning import wait_if_gui, LockRenderer

from integral_timber_joints.planning.parsing import parse_process, save_process, save_movements, get_process_path, \
    copy_robotic_movements, archive_robotic_movements, move_saved_movement
from integral_timber_joints.planning.robot_setup import load_RFL_world, get_tolerances
from integral_timber_joints.planning.utils import print_title, beam_ids_from_argparse_seq_n, color_from_success, LOGGER
from integral_timber_joints.planning.state import set_state, set_initial_state
from integral_timber_joints.planning.visualization import visualize_movement_trajectory
from integral_timber_joints.planning.solve import get_movement_status, MovementStatus, compute_selected_movements
from integral_timber_joints.planning.smoothing import smooth_movement_trajectory
from integral_timber_joints.planning.run import compute_selected_movements_by_status_priority

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotScrewdriverSyncLinearMovement, Movement
from integral_timber_joints.process.movement import RoboticMovement

from compas_fab_pychoreo.backend_features.pychoreo_plan_motion import MOTION_PLANNING_ALGORITHMS
from compas_fab_pychoreo.utils import LOGGER as PYCHOREO_LOGGER

SOLVE_MODE = [
    'lmg', # linear movement group only
    'fmg', # free movement group only
    'all',
    'movement_id', # 'Compute only for movement with a specific tag, e.g. `A54_M0`.'
]

##############################################

def plan_for_movement_group_with_restart(client, robot, unplanned_process, movements, args, options=None):
    """A wrapper function to plan for a movement group with restart until a plan is found.
    See `compute_movements_for_beam_id` for detailed planning strategies.

    The client will be recreated at each restart as well.
    """
    solve_timeout = options.get('solve_timeout', 600)
    # max solve iter kept rather high to prioritize timeout
    solve_iters = options.get('solve_iters', 1000)
    runtime_data = {}

    wip_process = deepcopy(unplanned_process)

    start_time = time.time()
    trial_i = 0
    while elapsed_time(start_time) < solve_timeout and trial_i < solve_iters:
        copy_st_time = time.time()

        # * set to initial state without initialization (importing tools etc. as collision objects from files)
        set_initial_state(client, robot, wip_process, initialize=False, options=options)
        copy_time = elapsed_time(copy_st_time)

        LOGGER.debug('#'*10)
        LOGGER.info('Trail #{} | time elapsed {:.2f}'.format(trial_i, elapsed_time(start_time)))
        options['profiles'] = {}
        single_run_st_time = time.time()

        success = compute_target_movements(client, robot, wip_process, movements, args, options=options)

        runtime_data[trial_i] = {}
        runtime_data[trial_i]['success'] = success
        runtime_data[trial_i]['profiles'] = deepcopy(options['profiles'])

        if success:
            LOGGER.info('Trail #{} | Plan Success | Runtime of current attempt: {:.1f}'.format(trial_i, elapsed_time(single_run_st_time)))
            # * copy the freshly planned movement back to unplanned_process
            for movement in movements:
                copy_robotic_movements(wip_process, unplanned_process, [], movement_id=movement.movement_id, options=options)
            break
        else:
            LOGGER.info('Trail #{} | Plan Failure | Runtime of current attempt: {:.1f}'.format(trial_i, elapsed_time(single_run_st_time)))
            # * reset target movements from the original, unplanned process file
            for movement in movements:
                copy_robotic_movements(unplanned_process, wip_process, [], movement_id=movement.movement_id, options=options)

            trial_i += 1
            # process/client reset time shouldn't be counted in timeout
            solve_timeout += copy_time
            LOGGER.debug('Copy process takes {} | total timeout {}'.format(copy_time, solve_timeout))
    else:
        failure_reason = 'exceeding solve timeout {:.2f}.'.format(solve_timeout) if trial_i < solve_iters else 'exceeding max solve iteration {}'.format(trial_i)
        LOGGER.error('Planning (with restarts) failed: {}'.format(failure_reason))

    return success, runtime_data

def compute_target_movements(client, robot, process, movements, args, options=None):
    solved_movements = []
    st_time = time.time()
    with LockRenderer(not args.debug and not args.diagnosis) as lockrenderer:
        options['lockrenderer'] = lockrenderer

        # * Only compute one linear motion group. Priority Applies
        # LOGGER.info('Computing priority 1 LinearMovement(s)'.format())
        options['movement_id_filter'] = [m.movement_id for m in movements]

        # * Priority 1
        success, movements = compute_selected_movements_by_status_priority(client, robot, process, None,
            planning_priority_filter = [1],
            options=options, diagnosis=args.diagnosis)
        if not success:
            LOGGER.info('Computing failed priority 1 Movement(s)'.format())
            return False

        # * Priority 0
        success, movements = compute_selected_movements_by_status_priority(client, robot, process, None,
            planning_priority_filter = [0],
            options=options, diagnosis=args.diagnosis)
        if not success:
            LOGGER.info('Computing failed priority 0 Movement(s)'.format())
            return False

        solved_movements += movements

    LOGGER.info('Solved {} Movements in {:.2f} s: {} '.format(len(solved_movements), elapsed_time(st_time), [m.movement_id for m in solved_movements]))

    # * export computed movements (unsmoothed)
    if args.write:
        # * solved_movements are saved externally in the results folder
        save_movements(args.design_dir, solved_movements, save_dir=args.problem_subdir, movement_subdir='movements')
        # movements without trajectories but end conf set will be saved into the WIP process file
        # save_process(args.design_dir, args.problem, process, save_dir=args.problem_subdir)

    # * smoothing
    if not args.no_smooth:
        smoothed_movements = []
        st_time = time.time()
        with pp.LockRenderer(): # not args.debug):
            solved_free_movements = [fm for fm in solved_movements if isinstance(fm, RoboticFreeMovement)]

            if len(solved_free_movements) > 0:
                with tqdm(total=len(solved_free_movements), desc='smoothing') as pbar:
                    for free_m in solved_free_movements:
                        pbar.set_postfix_str(f'{free_m.movement_id}:{free_m.__class__.__name__}')
                        success, smoothed_traj, msg = smooth_movement_trajectory(client, process, robot, free_m, options=options)
                        free_m.trajectory = smoothed_traj
                        smoothed_movements.append(free_m)
                        if not success:
                            LOGGER.error('Smooth success: {} | msg: {}'.format(success, msg))
                            # TODO return False
                        pbar.update(1)
        LOGGER.debug('Smoothing takes {:.2f} s'.format(elapsed_time(st_time)))
        # * export smoothed movements
        if args.write and len(smoothed_movements) > 0:
            save_movements(args.design_dir, smoothed_movements, save_dir=args.problem_subdir, movement_subdir='smoothed_movements')

    # * final visualization
    if args.watch:
        # order movement according to their ids
        viz_movements = {}
        for am in solved_movements:
            index = process.movements.index(am)
            viz_movements[index] = am
        print_title('Visualize results')
        wait_if_gui('Start simulating results. Press enter to start.')
        set_state(client, robot, process, process.initial_state, options=options)
        for m_id in sorted(viz_movements.keys()):
            visualize_movement_trajectory(client, robot, process, viz_movements[m_id], step_sim=args.step_sim)

    # LOGGER.debug('A plan has been found for (seq_n={}) beam id {}!'.format(seq_n, beam_id))
    return success

#################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--design_dir', default='210605_ScrewdriverTestProcess',
                        help='problem json\'s containing folder\'s name.')
    parser.add_argument('--problem', default='nine_pieces_process.json', # twelve_pieces_process.json
                        help='The name of the problem to solve (json file\'s name, e.g. "nine_pieces_process.json")')
    parser.add_argument('--problem_subdir', default='results',
                        help='subdir for saving movements, default to `results`.')
    #
    parser.add_argument('--movement_id', default=None, type=str, help='Compute only for movement with a specific tag, e.g. `A54_M0`. This will force solve_mode to `movement_id` mode and plan for the residing group.')
    #
    parser.add_argument('--solve_mode', default='all', choices=SOLVE_MODE, help='solve mode.')
    parser.add_argument('--no_smooth', action='store_true', help='Not apply smoothing on free motions upon a plan is found. Defaults to False.')
    parser.add_argument('--keep_planned_movements', action='store_true', help='Skip planning for movements with trajectories. Defaults to False.')
    #
    parser.add_argument('--write', action='store_true', help='Write output json.')
    #
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
    parser.add_argument('--watch', action='store_true', help='Watch computed trajectories in the pybullet GUI.')
    parser.add_argument('--step_sim', action='store_true', help='Pause after each conf viz.')
    #
    parser.add_argument('--debug', action='store_true', help='Debug mode.')
    parser.add_argument('--diagnosis', action='store_true', help='Diagnosis mode, show collisions whenever encountered')
    parser.add_argument('--quiet', action='store_true', help='Disable print out verbose. Defaults to False.')
    parser.add_argument('--draw_mp_exploration', action='store_true', help='Draw motion planning graph exploration. Should be used together with diagnosis')
    parser.add_argument('--use_stored_seed', action='store_true', help='Use saved random seed in movement for planning.')
    #
    parser.add_argument('--reinit_tool', action='store_true', help='Regenerate tool URDFs.')
    #
    parser.add_argument('--low_res', action='store_true', help='Run the planning with low resolutions. Defaults to True.')
    parser.add_argument('--solve_timeout', default=600.0, type=float, help='For automatic planning retry, number of seconds before giving up. Defaults to 600.')
    parser.add_argument('--solve_iters', default=1000, type=int, help='For automatic planning retry, number of restart before giving up. Defaults to 1000.')
    parser.add_argument('--mp_algorithm', default='birrt', type=str, choices=MOTION_PLANNING_ALGORITHMS, help='Motion planning algorithms.')
    parser.add_argument('--rrt_iterations', default=400, type=int, help='Number of iterations within one rrt session. Defaults to 400.')
    parser.add_argument('--buffers_for_free_motions', action='store_true', help='Turn on buffering linear motions for free movements, used for narrow passage scenarios. Defaults to False.')
    parser.add_argument('--reachable_range', nargs=2, default=[0.2, 2.40], type=float, help='Reachable range (m) of the robot tcp from the base. Two numbers Defaults to `--reachable_range 0.2, 2.4`. It is possible to relax it to 3.0')
    parser.add_argument('--mesh_split_long_edge_max_length', default=0.0, type=float, help='the range of edges to be split if they are longer than given threshold used in CGAL\'s split mesh edges function. Unit in millimeter. By default 0.0 will turn this feature off.')
    args = parser.parse_args()

    if args.movement_id is not None:
        args.solve_mode = 'movement_id'

    log_folder = os.path.dirname(get_process_path(args.design_dir, args.problem, subdir=args.problem_subdir))
    log_path = os.path.join(log_folder, 'run.log')

    logging_level = logging.DEBUG if args.debug else logging.INFO
    LOGGER.setLevel(logging_level)
    PYCHOREO_LOGGER.setLevel(logging_level)

    file_handler = logging.FileHandler(filename=log_path, mode='a')
    formatter = logging.Formatter('%(asctime)s | %(name)s | %(levelname)s | %(message)s')
    file_handler.setFormatter(formatter)
    file_handler.setLevel(logging_level)
    LOGGER.addHandler(file_handler)
    LOGGER.info("planning.run.py started with args: %s" % args)

    # * Connect to path planning backend and initialize robot parameters
    client, robot, _ = load_RFL_world(viewer=args.viewer or args.diagnosis or args.watch or args.step_sim)

    #########
    options = {
        'debug' : args.debug,
        'diagnosis' : args.diagnosis,
        'verbose' : not args.quiet,
        'reinit_tool' : args.reinit_tool,
        'gantry_attempts' : 100, # number of gantry sampling attempts when computing IK
        # restart solve iters for each beam, can set to a large number to prioritize solve_timeout
        # ! restart is disabled when use_stored_seed = True
        'solve_timeout': args.solve_timeout,
        'solve_iters': args.solve_iters,
        'rrt_iterations': args.rrt_iterations,
        'draw_mp_exploration' : args.draw_mp_exploration and args.diagnosis,
        'mp_algorithm' : args.mp_algorithm,
        'check_sweeping_collision': True,
        'use_stored_seed' : args.use_stored_seed,
        'mesh_split_long_edge_max_length' : args.mesh_split_long_edge_max_length,
    }
    # ! frame, conf compare, joint flip and allowable collision tolerances are set here
    options.update(get_tolerances(robot, low_res=args.low_res))
    if len(args.reachable_range) == 2:
        options.update({
        'reachable_range': (args.reachable_range[0], args.reachable_range[1]),
        })
    if not args.no_smooth:
        options.update({
            'smooth_iterations' : 150,
            'max_smooth_time' : 180,
             })
    if args.buffers_for_free_motions:
        options.update({
            'retraction_vectors' : list(np.vstack([np.eye(3), -np.eye(3)])),
            'max_free_retraction_distances' : np.linspace(0, 0.1, 3),
        })

    #########
    # * Load process
    process = parse_process(args.design_dir, args.problem, subdir=args.problem_subdir)

    result_path = get_process_path(args.design_dir, args.problem, subdir=args.problem_subdir)
    ext_movement_path = os.path.dirname(result_path)

    # * locate all target movement groups, all high-level filtering happens here
    target_movement_groups = []
    if args.solve_mode == 'movement_id':
        # * only plan for the residing group for the target movement, if movement_id is provided
        movement = process.get_movement_by_movement_id(args.movement_id)
        assert isinstance(movement, RoboticMovement)

        # Get neighbouring Free Movements if the target group is a linear movement group
        if isinstance(movement, RoboticLinearMovement):
            movement_group = process.get_linear_movement_group(movement)
            target_movement_groups.append(process.get_prev_movement_group(movement_group[0]))
            target_movement_groups.append(movement_group)
            target_movement_groups.append(process.get_next_movement_group(movement_group[-1]))
        elif isinstance(movement, RoboticFreeMovement):
            movement_group = process.get_free_movement_group(movement)
            target_movement_groups = [movement_group]
        else:
            raise TypeError(movement.tag)
    else:
        movement = process.movements[0]
        if not isinstance(movement, RoboticMovement):
            movement = process.get_next_robotic_movement(movement)
        # * iterate through all the movements and apply certain filters for groups
        while movement is not None:
            # * movement type filter
            if args.solve_mode == 'lmg':
                correct_mv_type = isinstance(movement, RoboticLinearMovement)
            elif args.solve_mode == 'fmg':
                correct_mv_type = isinstance(movement, RoboticFreeMovement)
            elif args.solve_mode == 'all':
                correct_mv_type = True
            if isinstance(movement, RoboticLinearMovement):
                movement_group = process.get_linear_movement_group(movement)
            elif isinstance(movement, RoboticFreeMovement):
                movement_group = process.get_free_movement_group(movement)
            else:
                raise TypeError(movement.tag)

            if correct_mv_type:
                # * keep_planned_movements filter
                if args.keep_planned_movements and len(movement_group) > 0:
                    # if ALL of the movement within the group has planned trajectory, skip the current group
                    all_members_has_traj = True
                    for m_member in movement_group:
                        smoothed_path = os.path.join(ext_movement_path, m_member.get_filepath(subdir='smoothed_movements'))
                        nonsmoothed_path = os.path.join(ext_movement_path, m_member.get_filepath(subdir='movements'))
                        if not (os.path.exists(smoothed_path) or os.path.exists(nonsmoothed_path)):
                            all_members_has_traj = False
                            break
                    correct_mv_type = (not all_members_has_traj)

                if correct_mv_type and len(movement_group) > 0:
                    if args.solve_mode == 'lmg':
                        # remove neighbor free motion groups and add them to the target_groups
                        target_movement_groups.append(process.get_prev_movement_group(movement_group[0]))
                        target_movement_groups.append(movement_group)
                        target_movement_groups.append(process.get_next_movement_group(movement_group[-1]))
                    else:
                        # otherwise simply add the target group
                        # in the case of solve_mode == 'all', the free movement group will be processed sequentially in the while loop
                        target_movement_groups.append(movement_group)

            movement = process.get_next_robotic_movement(movement_group[-1])

    # * Archive the target movements (if they already exist in the external movement folder)
    for m_group in target_movement_groups:
        for movement in m_group:
            moved = move_saved_movement(movement, ext_movement_path)
            if moved: LOGGER.info("Movement group member {} removed.".format(movement.movement_id))

    # * load previously planned movements
    process.load_external_movements(ext_movement_path)

    # * Initialize (only needed once) collision objects and tools
    set_initial_state(client, robot, process, initialize=True, options=options)

    # * Plan all the linear groups first
    linear_movement_groups = []
    for mv_group in target_movement_groups:
        if len(mv_group) > 0 and isinstance(mv_group[0], RoboticLinearMovement):
            linear_movement_groups.append(mv_group)
    if len(linear_movement_groups) > 0:
        with tqdm(total=len(linear_movement_groups), desc='Linear movement groups') as pbar:
            for i, lm_group in enumerate(linear_movement_groups):
                success, _ = plan_for_movement_group_with_restart(client, robot, process, lm_group, args, options=options)
                if not success:
                    LOGGER.error('Linear movement group #{}/{} fails, which contains {}'.format(i, len(linear_movement_groups), 
                        [m.movement_id for m in lm_group]))
                    # continue
                pbar.update(1)

    # * Plan the free movements connecting the planned linear groups
    free_movement_groups = []
    for mv_group in target_movement_groups:
        if len(mv_group) > 0 and isinstance(mv_group[0], RoboticFreeMovement):
            free_movement_groups.append(mv_group)
    if len(linear_movement_groups) > 0:
        with tqdm(total=len(free_movement_groups), desc='Free movement groups') as pbar:
            for i, fm_group in enumerate(free_movement_groups):
                success, _ = plan_for_movement_group_with_restart(client, robot, process, fm_group, args, options=options)
                if not success:
                    LOGGER.error('Free movement group #{}/{} fails, which contains {}'.format(i, len(linear_movement_groups), 
                        [m.movement_id for m in fm_group]))
                    # TODO restart if the free movement planning fails, need to remove adjacent linear groups
                    # but we should only remove the "hard" linear group and keep the "easy" linear group
                    # continue
                pbar.update(1)

    LOGGER.info('Planning done.')
    client.disconnect()

if __name__ == '__main__':
    main()
