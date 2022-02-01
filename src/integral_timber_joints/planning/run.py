import os
import time
import logging
import numpy as np
import argparse
import pybullet_planning as pp
from pybullet_planning.motion_planners.utils import elapsed_time

from termcolor import colored
from copy import deepcopy

from pybullet_planning import wait_if_gui, LockRenderer, HideOutput

from integral_timber_joints.planning.parsing import parse_process, save_process_and_movements, get_process_path, save_process
from integral_timber_joints.planning.robot_setup import load_RFL_world, get_tolerances, MAIN_ROBOT_ID, \
    get_gantry_robot_custom_limits
from integral_timber_joints.planning.utils import print_title, beam_ids_from_argparse_seq_n, color_from_success, LOGGER
from integral_timber_joints.planning.state import set_state, set_initial_state
from integral_timber_joints.planning.visualization import visualize_movement_trajectory
from integral_timber_joints.planning.solve import get_movement_status, MovementStatus, compute_selected_movements
from integral_timber_joints.planning.smoothing import smooth_movement_trajectory

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotScrewdriverSyncLinearMovement
from integral_timber_joints.process.movement import RoboticMovement

from compas_fab_pychoreo.backend_features.pychoreo_plan_motion import MOTION_PLANNING_ALGORITHMS

SOLVE_MODE = [
    'nonlinear',
    'linear',
    'movement_id', # 'Compute only for movement with a specific tag, e.g. `A54_M0`.'
    'free_motion_only', # 'Only compute free motions.'
]

# * Next steps
# TODO use linkstatistics joint weight and resolutions
# TODO backtrack in case of subsequent sampling cannot find a solution (linear movement with one end specified)

##############################################

def plan_for_beam_id_with_restart(client, robot, process, beam_id, args, options=None):
    """A wrapper function to plan for all the movements of a beam with restart until a plan is found.
    See `compute_movements_for_beam_id` for detailed planning strategies.

    The client will be recreated at each restart as well.
    """
    solve_timeout = options.get('solve_timeout', 600)
    # TODO change back to 10
    solve_iters = options.get('solve_iters', 40)
    return_upon_success = options.get('return_upon_success', True)
    ignore_taught_confs = options.get('ignore_taught_confs', False)
    runtime_data = {}

    start_time = time.time()
    trial_i = 0
    while elapsed_time(start_time) < solve_timeout and trial_i < solve_iters:
        LOGGER.info('#'*10)
        LOGGER.info('Beam {} | {} | Trail #{} | time {:.2f}'.format(beam_id, args.solve_mode, trial_i, elapsed_time(start_time)))
        options['profiles'] = {}
        success = compute_movements_for_beam_id(client, robot, process, beam_id, args, options=options)
        runtime_data[trial_i] = {}
        runtime_data[trial_i]['success'] = success
        runtime_data[trial_i]['profiles'] = deepcopy(options['profiles'])
        LOGGER.info(colored('Return success: {}'.format(success), 'green' if success else 'red'))
        if return_upon_success and success:
            break
        trial_i += 1
        copy_st_time = time.time()
        # * recreate process and client for the next attempt
        process = parse_process(args.design_dir, args.problem, subdir=args.problem_subdir)
        if ignore_taught_confs:
            for m in process.movements:
                process.set_movement_end_robot_config(m, None)
        client.disconnect()
        client, robot, _ = load_RFL_world(viewer=args.viewer, verbose=False)
        set_initial_state(client, robot, process, disable_env=False, reinit_tool=False)
        copy_time = elapsed_time(copy_st_time)
        solve_timeout += copy_time
        LOGGER.debug('Restarting client/process takes {} | total timeout {}'.format(copy_time, solve_timeout))
        # ! process/client reset time shouldn't be counted in
    else:
        LOGGER.error('Planning (with restarts) for beam {} failed, exceeding timeout {} after {}.'.format(beam_id, solve_timeout, trial_i))

    return success, runtime_data

def compute_movements_for_beam_id(client, robot, process, beam_id, args, options=None):
    """Two types of movement planning strategies are provided:
        1. 'nonlinear': plan according to movement priorities, filtered by priority score and movement types
        2. 'linear': plan according to movements' order, taught conf cannot be

    Two additional type of planning mode are provided for plan fine-tuning:
        1. 'free_motion_only': only plan the free motions in the movements
        2. 'movement_id': only replan for a specific movement_id, movement_id can be
            index-based (e.g. 79) or string name (e.g. 'A78_M0')
    """
    seq_n = process.assembly.sequence.index(beam_id)
    all_movements = process.get_movements_by_beam_id(beam_id)
    options['samplig_order_counter'] = 0
    options['enforce_continuous'] = True
    if 'movement_id_filter' in options:
        del options['movement_id_filter']

    with LockRenderer(not args.debug and not args.diagnosis) as lockrenderer:
        options['lockrenderer'] = lockrenderer
        altered_movements = []
        with HideOutput(False): #
            if args.solve_mode == 'nonlinear':
                success, altered_ms = compute_selected_movements(client, robot, process, beam_id, 1, [RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotScrewdriverSyncLinearMovement],
                    [MovementStatus.neither_done, MovementStatus.one_sided],
                    options=options, diagnosis=args.diagnosis)
                if not success:
                    LOGGER.info('A plan NOT found using nonlinear planning at stage 1 for (seq_n={}) beam {}!'.format(seq_n, beam_id))
                    return False
                else:
                    altered_movements.extend(altered_ms)

                # TODO if fails remove the related movement's trajectory and try again
                success, altered_ms = compute_selected_movements(client, robot, process, beam_id, 0, [RoboticLinearMovement],
                    [MovementStatus.one_sided],
                    options=options, diagnosis=args.diagnosis)
                if not success:
                    LOGGER.info('A plan NOT found using nonlinear planning at stage 2 for (seq_n={}) beam {}!'.format(seq_n, beam_id))
                    return False
                else:
                    altered_movements.extend(altered_ms)

                # since adjacent "neither_done" states will change to `one_sided` and get skipped
                # which will cause adjacent linear movement joint flip problems (especially for clamp placements)
                # Thus, when solving `neither_done`, we solve for both `neither_done` and `one_sided` sequentially
                # The movement statuses get changed on the fly.
                success, altered_ms = compute_selected_movements(client, robot, process, beam_id, 0, [RoboticLinearMovement],
                    [MovementStatus.neither_done, MovementStatus.one_sided],
                    options=options, diagnosis=args.diagnosis)
                if not success:
                    LOGGER.info('A plan NOT found using nonlinear planning at stage 3 for (seq_n={}) beam {}!'.format(seq_n, beam_id))
                    return False
                else:
                    altered_movements.extend(altered_ms)

                # Ideally, all the free motions should have both start and end conf specified.
                # one_sided is used to sample the start conf if none is given (especially when `arg.problem_dir = 'YJ_tmp'` is not used).
                success, altered_ms = compute_selected_movements(client, robot, process, beam_id, 0, [RoboticFreeMovement],
                    [MovementStatus.both_done, MovementStatus.one_sided],
                    options=options, diagnosis=args.diagnosis)
                if not success:
                    LOGGER.info('A plan NOT found using nonlinear planning at stage 4 for (seq_n={}) beam {}!'.format(seq_n, beam_id))
                    return False
                else:
                    altered_movements.extend(altered_ms)

            elif args.solve_mode == 'linear':
                movement_id_range = options.get('movement_id_range', range(0, len(all_movements)))
                options['movement_id_filter'] = [all_movements[m_i].movement_id for m_i in movement_id_range]
                # options['enforce_continuous'] = False
                success, altered_ms = compute_selected_movements(client, robot, process, beam_id, 0, [RoboticMovement],
                    [MovementStatus.correct_type], options=options, diagnosis=args.diagnosis, \
                    check_type_only=True)
                if not success:
                    LOGGER.info('A plan NOT found using linear (chained) planning for (seq_n={}) beam {}!'.format(seq_n, beam_id))
                    return False
                altered_movements.extend(altered_ms)

            elif args.solve_mode == 'free_motion_only':
                success, altered_ms = compute_selected_movements(client, robot, process, beam_id, None, [RoboticFreeMovement],
                    [MovementStatus.both_done, MovementStatus.one_sided],
                    options=options, diagnosis=args.diagnosis)
                if not success:
                    LOGGER.info('A plan NOT found for free motion for (seq_n={}) beam {}!'.format(seq_n, beam_id))
                    return False
                altered_movements.extend(altered_ms)

            elif args.solve_mode == 'movement_id':
                # * compute for movement_id movement
                LOGGER.info('Computing only for {}'.format(args.movement_id))
                options['movement_id_filter'] = [args.movement_id]
                # ! allow for both integer-based index and string names
                if args.movement_id.startswith('A'):
                    chosen_m = process.get_movement_by_movement_id(args.movement_id)
                else:
                    chosen_m = process.movements[int(args.movement_id)]
                # * if linear movement and has both end specified, ask user keep start or end
                if get_movement_status(process, chosen_m, [RoboticLinearMovement]) in [MovementStatus.has_traj, MovementStatus.both_done]:
                    chosen_m.trajectory = None
                    keep_end = int(input("Keep start or end conf? Enter 0 for start, 1 for end. 2 for abandoning both."))
                    if keep_end == 0:
                        # keep start
                        process.set_movement_end_robot_config(chosen_m, None)
                    elif keep_end == 1:
                        process.set_movement_start_robot_config(chosen_m, None)
                    elif keep_end == 2:
                        process.set_movement_start_robot_config(chosen_m, None)
                        process.set_movement_end_robot_config(chosen_m, None)
                        assert get_movement_status(process, chosen_m, [RoboticLinearMovement]) in [MovementStatus.neither_done]
                    if keep_end != 2:
                        assert get_movement_status(process, chosen_m, [RoboticLinearMovement]) in [MovementStatus.one_sided]

                success, altered_ms = compute_selected_movements(client, robot, process, beam_id, 0, [],
                    None, options=options, diagnosis=args.diagnosis)
                if not success:
                    LOGGER.info('A plan NOT found for movement_id {}!'.format(args.movement_id))
                    return False
                altered_movements.extend(altered_ms)
            else:
                raise NotImplementedError('Solver {} not implemented!'.format(args.solve_mode))

    # * export computed movements
    if args.write:
        save_process_and_movements(args.design_dir, args.problem, process, altered_movements, overwrite=False,
            include_traj_in_process=False)

    # * smoothing
    if args.smooth:
        LOGGER.debug('Smoothing trajectory...')
        smoothed_movements = []
        with pp.LockRenderer(): # not args.debug):
            for altered_m in altered_movements:
                if not isinstance(altered_m, RoboticFreeMovement):
                    continue
                success, smoothed_traj, msg = smooth_movement_trajectory(client, process, robot, altered_m, options=options)
                altered_m.trajectory = smoothed_traj
                smoothed_movements.append(altered_m)
                LOGGER.debug(colored('Smooth success: {} | msg: {}'.format(success, msg), color_from_success(success)))
        if args.write:
            save_process_and_movements(args.design_dir, args.problem, process, smoothed_movements, overwrite=False,
                include_traj_in_process=False, movement_subdir='smoothed_movements')

    # * final visualization
    if args.watch:
        # order movement according to their ids
        viz_movements = {}
        for am in altered_movements:
            index = process.movements.index(am)
            viz_movements[index] = am
        print_title('Visualize results')
        wait_if_gui('Start simulating results. Press enter to start.')
        set_state(client, robot, process, process.initial_state, options=options)
        for m_id in sorted(viz_movements.keys()):
            visualize_movement_trajectory(client, robot, process, viz_movements[m_id], step_sim=args.step_sim)

    LOGGER.info('A plan has been found for (seq_n={}) beam id {}!'.format(seq_n, beam_id))
    return success

#################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--design_dir', default='210605_ScrewdriverTestProcess',
                        help='problem json\'s containing folder\'s name.')
    parser.add_argument('--problem', default='nine_pieces_process.json', # twelve_pieces_process.json
                        help='The name of the problem to solve (json file\'s name, e.g. "nine_pieces_process.json")')
    parser.add_argument('--problem_subdir', default='.',
                        help='subdir of the process file, default to `.`. Popular use: `results`')
    #
    parser.add_argument('--seq_n', nargs='+', type=int, help='Zero-based index according to the Beam sequence in process.assembly.sequence. If only provide one number, `--seq_n 1`, we will only plan for one beam. If provide two numbers, `--seq_n start_id end_id`, we will plan from #start_id UNTIL #end_id. If more numbers are provided. By default, all the beams will be checked.')
    parser.add_argument('--movement_id', default=None, type=str, help='Compute only for movement with a specific tag, e.g. `A54_M0`.')
    #
    parser.add_argument('--solve_mode', default='nonlinear', choices=SOLVE_MODE, help='solve mode.')
    parser.add_argument('--smooth', action='store_false', help='Apply smoothing.')
    #
    parser.add_argument('--write', action='store_true', help='Write output json.')
    parser.add_argument('--load_external_movements', action='store_true', help='Load externally saved movements into the parsed process, default to False.')
    #
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
    parser.add_argument('--watch', action='store_true', help='Watch computed trajectories in the pybullet GUI.')
    parser.add_argument('--step_sim', action='store_true', help='Pause after each conf viz.')
    #
    parser.add_argument('--debug', action='store_true', help='Debug mode.')
    parser.add_argument('--diagnosis', action='store_true', help='Diagnosis mode, show collisions whenever encountered')
    parser.add_argument('--quiet', action='store_true', help='Disable print out verbose. Defaults to False.')
    parser.add_argument('--draw_mp_exploration', action='store_true', help='Draw motion planning graph exploration. Should be used together with diagnosis')
    #
    parser.add_argument('--reinit_tool', action='store_true', help='Regenerate tool URDFs.')
    #
    parser.add_argument('--low_res', action='store_true', help='Run the planning with low resolutions. Defaults to True.')
    parser.add_argument('--solve_timeout', default=600.0, type=float, help='For automatic planning retry, number of seconds before giving up. Defaults to 600.')
    parser.add_argument('--mp_algorithm', default='birrt', type=str, choices=MOTION_PLANNING_ALGORITHMS, help='Motion planning algorithms.')
    parser.add_argument('--rrt_iterations', default=400, type=int, help='Number of iterations within one rrt session. Defaults to 400.')
    parser.add_argument('--buffers_for_free_motions', action='store_true', help='Turn on buffering linear motions for free movements, used for narrow passage scenarios. Defaults to False.')
    parser.add_argument('--reachable_range', nargs=2, default=[0.2, 2.40], type=float, help='Reachable range (m) of the robot tcp from the base. Two numbers Defaults to `--reachable_range 0.2, 2.4`. It is possible to relax it to 3.0')
    args = parser.parse_args()

    log_folder = os.path.dirname(get_process_path(args.design_dir, args.problem, subdir='results'))
    log_path = os.path.join(log_folder, 'run.log')

    logging_level = logging.DEBUG if args.debug else logging.INFO
    LOGGER.setLevel(logging_level)

    file_handler = logging.FileHandler(filename=log_path, mode='w')
    formatter = logging.Formatter('%(asctime)s | %(name)s | %(levelname)s | %(message)s')
    file_handler.setFormatter(formatter)
    file_handler.setLevel(logging_level)
    LOGGER.addHandler(file_handler)
    LOGGER.info("planning.run.py started with args: %s" % args)

    if args.movement_id is not None:
        args.solve_mode = 'movement_id'

    # * Connect to path planning backend and initialize robot parameters
    client, robot, _ = load_RFL_world(viewer=args.viewer or args.diagnosis or args.watch or args.step_sim)

    #########
    # * Load process and recompute actions and states
    process = parse_process(args.design_dir, args.problem, subdir=args.problem_subdir)

    # * force load external if only planning for the free motions
    args.load_external_movements = args.load_external_movements or \
        args.solve_mode == 'free_motion_only'
        # or args.solve_mode == 'movement_id'
    if args.load_external_movements:
        result_path = get_process_path(args.design_dir, args.problem, subdir='results')
        ext_movement_path = os.path.dirname(result_path)
        LOGGER.info('Loading external movements from {}'.format(ext_movement_path))
        process.load_external_movements(ext_movement_path)

    #########
    options = {
        'debug' : args.debug,
        'diagnosis' : args.diagnosis,
        'verbose' : not args.quiet,
        'low_res' : args.low_res,
        'gantry_attempts' : 100, # number of gantry sampling attempts when computing IK
        'custom_limits' : get_gantry_robot_custom_limits(MAIN_ROBOT_ID),
        # the collision is counted when penetration distance is bigger than this value
        'collision_distance_threshold' : 0.0012, # in meter,
        'solve_timeout': args.solve_timeout,
        'rrt_iterations': args.rrt_iterations,
        'draw_mp_exploration' : args.draw_mp_exploration and args.diagnosis,
        'mp_algorithm' : args.mp_algorithm,
    }
    # ! frame, conf compare, joint flip tolerances are set here
    options.update(get_tolerances(robot))
    if len(args.reachable_range) == 2:
        options.update({
        'reachable_range': (args.reachable_range[0], args.reachable_range[1]),
        })
    if args.smooth:
        options.update({
            'smooth_iterations' : 150,
            'max_smooth_time' : 60,
             })
    if args.buffers_for_free_motions:
        options.update({
            'retraction_vectors' : list(np.vstack([np.eye(3), -np.eye(3)])),
            'max_free_retraction_distances' : np.linspace(0, 0.1, 3),
        })

    #########
    set_initial_state(client, robot, process, reinit_tool=args.reinit_tool)
    beam_ids = beam_ids_from_argparse_seq_n(process, args.seq_n, args.movement_id)
    for beam_id in beam_ids:
        LOGGER.info('-'*20)
        success, trial_runtime_data = plan_for_beam_id_with_restart(client, robot, process, beam_id, args, options=options)

    LOGGER.info(colored('Planning done.', 'green'))
    client.disconnect()

if __name__ == '__main__':
    main()
