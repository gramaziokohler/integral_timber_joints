import os
import time
import logging
import numpy as np
import argparse
from tqdm import tqdm
import pybullet_planning as pp
from pybullet_planning.motion_planners.utils import elapsed_time
from typing import Tuple, List, Type, Dict

from termcolor import colored
from copy import deepcopy

from pybullet_planning import wait_if_gui, LockRenderer, HideOutput

from integral_timber_joints.planning.parsing import parse_process, save_process, save_movements, get_process_path, \
    copy_robotic_movements, archive_robotic_movements, move_saved_movement
from integral_timber_joints.planning.robot_setup import load_RFL_world, get_tolerances
from integral_timber_joints.planning.utils import print_title, beam_ids_from_argparse_seq_n, color_from_success, LOGGER
from integral_timber_joints.planning.state import set_state, set_initial_state
from integral_timber_joints.planning.visualization import visualize_movement_trajectory
from integral_timber_joints.planning.solve import get_movement_status, MovementStatus, compute_selected_movements
from integral_timber_joints.planning.smoothing import smooth_movement_trajectory

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotScrewdriverSyncLinearMovement, Movement
from integral_timber_joints.process.movement import RoboticMovement

from compas_fab_pychoreo.backend_features.pychoreo_plan_motion import MOTION_PLANNING_ALGORITHMS
from compas_fab_pychoreo.utils import LOGGER as PYCHOREO_LOGGER

SOLVE_MODE = [
    'nonlinear',
    'linear',
    'movement_id', # 'Compute only for movement with a specific tag, e.g. `A54_M0`.'
    'free_motion_only', # 'Only compute free motions. Priority Applies'
    'linear_motion_only', # 'Only compute linear motions. Priority Applies'
    'linear_movement_group',
]

##############################################

def plan_for_beam_id_with_restart(client, robot, unplanned_process, beam_id, args, options=None):
    """A wrapper function to plan for all the movements of a beam with restart until a plan is found.
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
        LOGGER.info('Beam {} | {} | Trail #{} | time elapsed {:.2f}'.format(beam_id, args.solve_mode, trial_i, elapsed_time(start_time)))
        options['profiles'] = {}
        single_run_st_time = time.time()

        success = compute_movements_for_beam_id(client, robot, wip_process, beam_id, args, options=options)

        runtime_data[trial_i] = {}
        runtime_data[trial_i]['success'] = success
        runtime_data[trial_i]['profiles'] = deepcopy(options['profiles'])

        if success:
            LOGGER.info('Beam {} | {} | Trail #{} | Plan Success | Runtime of current attempt: {:.1f}'.format(beam_id, args.solve_mode, trial_i, elapsed_time(single_run_st_time)))
            # * copy the freshly planned movement back to unplanned_process
            copy_robotic_movements(wip_process, unplanned_process, [beam_id], movement_id=args.movement_id, options=options)
            break
        else:
            LOGGER.info('Beam {} | {} | Trail #{} | Plan Failure | Runtime of current attempt: {:.1f}'.format(beam_id, args.solve_mode, trial_i, elapsed_time(single_run_st_time)))
            # * reset target movements from the original, unplanned process file
            copy_robotic_movements(unplanned_process, wip_process, [beam_id], movement_id=args.movement_id, options=options)

            trial_i += 1
            # process/client reset time shouldn't be counted in timeout
            solve_timeout += copy_time
            LOGGER.debug('Copy process takes {} | total timeout {}'.format(copy_time, solve_timeout))
    else:
        failure_reason = 'exceeding solve timeout {:.2f}.'.format(solve_timeout) if trial_i < solve_iters else 'exceeding max solve iteration {}'.format(trial_i)
        LOGGER.error('Planning (with restarts) for beam {} failed: {}'.format(beam_id, failure_reason))

    return success, runtime_data

def compute_selected_movements_by_status_priority(client, robot, process,
                               beam_id: str,
                               planning_priority_filter: List[int] = None,
                               movement_type_filter: List[Type] = None,
                               has_no_trajectory : bool = True,
                               options:  Dict = None,
                               viz_upon_found: bool = False,
                               diagnosis: bool = False,
                               ) -> Tuple[bool, List[Movement]]:
    """
    Wrapper for `compute_selected_movements`
    Solving first for MovementStatus.both_done, then one_sided, finally neither_done.
    """

    success, computed_movements_0 = compute_selected_movements(client, robot, process, beam_id, planning_priority_filter, movement_type_filter,
                                movement_status_filter = [MovementStatus.both_done],
                                has_no_trajectory = has_no_trajectory, options = options, viz_upon_found = viz_upon_found, diagnosis = diagnosis)
    if not success: return False, []
    success, computed_movements_1 = compute_selected_movements(client, robot, process, beam_id, planning_priority_filter, movement_type_filter,
                                movement_status_filter = [MovementStatus.one_sided],
                                has_no_trajectory = has_no_trajectory, options = options, viz_upon_found = viz_upon_found, diagnosis = diagnosis)
    if not success: return False, []
    success, computed_movements_2 = compute_selected_movements(client, robot, process, beam_id, planning_priority_filter, movement_type_filter,
                                movement_status_filter = [MovementStatus.neither_done],
                                has_no_trajectory = has_no_trajectory, options = options, viz_upon_found = viz_upon_found, diagnosis = diagnosis)
    if not success: return False, []
    return (True, computed_movements_0 + computed_movements_1 + computed_movements_2)

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
    beam_movements = process.get_movements_by_beam_id(beam_id)
    if 'movement_id_filter' in options:
        del options['movement_id_filter']

    solved_movements = []
    st_time = time.time()
    with LockRenderer(not args.debug and not args.diagnosis) as lockrenderer:
        options['lockrenderer'] = lockrenderer
        # * Non-linearly solving the movements within a beam_id
        # * First by priority number, then by [Linear > Free], then by [both_done > one_sided > neither_done]
        if args.solve_mode == 'nonlinear':
            LOGGER.info('Computing priority 1 LinearMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
            success, movements = compute_selected_movements_by_status_priority(client, robot, process, beam_id,
                planning_priority_filter = [1],
                movement_type_filter = [RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotScrewdriverSyncLinearMovement],
                options=options, diagnosis=args.diagnosis
                )
            if not success:
                LOGGER.info('Compute failed priority 1 LinearMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
                return False
            solved_movements += movements

            LOGGER.info('Computing priority 0 LinearMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
            success, movements = compute_selected_movements_by_status_priority(client, robot, process, beam_id,
                planning_priority_filter = [0],
                movement_type_filter = [RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotScrewdriverSyncLinearMovement],
                options=options, diagnosis=args.diagnosis)
            if not success:
                LOGGER.info('Compute failed priority 0 LinearMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
                return False
            solved_movements += movements

            LOGGER.info('Computing priority 1 FreeMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
            success, movements = compute_selected_movements_by_status_priority(client, robot, process, beam_id,
                planning_priority_filter = [1],
                movement_type_filter = [RoboticFreeMovement],
                options=options, diagnosis=args.diagnosis)
            if not success:
                LOGGER.info('Computing failed priority 1 FreeMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
                return False
            solved_movements += movements

            # Ideally, all the free motions should have both start and end conf specified.
            # one_sided is used to sample the start conf if none is given.
            LOGGER.info('Computing priority 0 FreeMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
            success, movements = compute_selected_movements_by_status_priority(client, robot, process, beam_id,
                planning_priority_filter = [0],
                movement_type_filter = [RoboticFreeMovement],
                options=options, diagnosis=args.diagnosis)
            if not success:
                LOGGER.info('Computing failed priority 0 FreeMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
                return False
            solved_movements += movements

        # * Linearly (Sequentially) solving the movements within a beam_id
        elif args.solve_mode == 'linear':
            movement_id_range = options.get('movement_id_range', range(0, len(beam_movements)))
            options['movement_id_filter'] = [beam_movements[m_i].movement_id for m_i in movement_id_range]
            success, movements = compute_selected_movements(client, robot, process, beam_id, options=options, diagnosis=args.diagnosis, )
            if not success:
                LOGGER.info('A plan NOT found using linear (chained) planning for (seq_n={}) beam {}!'.format(seq_n, beam_id))
                return False
            solved_movements += movements

        # * Only compute free motions. Priority Applies
        elif args.solve_mode == 'free_motion_only':
            LOGGER.info('Computing priority 1 FreeMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
            success, movements = compute_selected_movements_by_status_priority(client, robot, process, beam_id,
                planning_priority_filter = [1],
                movement_type_filter = [RoboticFreeMovement],
                options=options, diagnosis=args.diagnosis)
            if not success:
                LOGGER.info('Computing failed priority 1 FreeMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
                return False
            solved_movements += movements

            # Ideally, all the free motions should have both start and end conf specified.
            # one_sided is used to sample the start conf if none is given.
            LOGGER.info('Computing priority 0 FreeMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
            success, movements = compute_selected_movements_by_status_priority(client, robot, process, beam_id,
                planning_priority_filter = [0],
                movement_type_filter = [RoboticFreeMovement],
                options=options, diagnosis=args.diagnosis)
            if not success:
                LOGGER.info('Computing failed priority 0 FreeMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
                return False
            solved_movements += movements

        # * Only compute linear motions. Priority Applies
        elif args.solve_mode == 'linear_motion_only':
            LOGGER.info('Computing priority 1 LinearMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
            success, movements = compute_selected_movements_by_status_priority(client, robot, process, beam_id,
                planning_priority_filter = [1],
                movement_type_filter = [RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotScrewdriverSyncLinearMovement],
                options=options, diagnosis=args.diagnosis)
            if not success:
                LOGGER.info('Computing failed priority 1 LinearMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
                return False
            solved_movements += movements

            # Ideally, all the free motions should have both start and end conf specified.
            # one_sided is used to sample the start conf if none is given.
            LOGGER.info('Computing priority 0 LinearMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
            success, movements = compute_selected_movements_by_status_priority(client, robot, process, beam_id,
                planning_priority_filter = [0],
                movement_type_filter = [RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotScrewdriverSyncLinearMovement],
                options=options, diagnosis=args.diagnosis)
            if not success:
                LOGGER.info('Computing failed priority 0 LinearMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
                return False
            solved_movements += movements

        # * Only compute one linear motion group. Priority Applies
        elif args.solve_mode == 'linear_movement_group':
            LOGGER.info('Computing priority 1 LinearMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))

            # Get all movement_ids of a linear motion group
            movement = process.get_movement_by_movement_id(args.movement_id)
            movements = process.get_linear_movement_group(movement)
            options['movement_id_filter'] = [m.movement_id for m in movements]

            # * Priority 1
            success, movements = compute_selected_movements_by_status_priority(client, robot, process, None,
                planning_priority_filter = [1],
                options=options, diagnosis=args.diagnosis)
            if not success:
                LOGGER.info('Computing failed priority 1 LinearMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
                return False
            solved_movements += movements

            # * Priority 0
            success, movements = compute_selected_movements_by_status_priority(client, robot, process, None,
                planning_priority_filter = [0],
                options=options, diagnosis=args.diagnosis)
            if not success:
                LOGGER.info('Computing failed priority 0 LinearMovement(s) (seq_n={}, {})'.format(seq_n, beam_id))
                return False
            solved_movements += movements

        elif args.solve_mode == 'movement_id':
            # * compute for movement_id movement
            LOGGER.info('Computing only Movement {}'.format(args.movement_id))
            options['movement_id_filter'] = [args.movement_id]
            success, movements = compute_selected_movements(client, robot, process, beam_id, options=options, diagnosis=args.diagnosis)
            if not success:
                LOGGER.info('A plan NOT found for movement_id {}!'.format(args.movement_id))
                return False
            solved_movements += movements
        else:
            raise NotImplementedError('Solver {} not implemented!'.format(args.solve_mode))

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
            if args.force_linear_to_free_movement:
                solved_free_movements = [fm for fm in solved_movements if isinstance(fm, RoboticMovement)]
            else:
                solved_free_movements = [fm for fm in solved_movements if isinstance(fm, RoboticFreeMovement)]

            with tqdm(total=len(solved_free_movements), desc='smoothing') as pbar:
                for free_m in solved_free_movements:
                    pbar.set_postfix_str(f'{free_m.movement_id}:{free_m.__class__.__name__}, {free_m.tag}')
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

    LOGGER.debug('A plan has been found for (seq_n={}) beam id {}!'.format(seq_n, beam_id))
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
    parser.add_argument('--seq_n', nargs='+', type=int, help='Zero-based index according to the Beam sequence in process.assembly.sequence. If only provide one number, `--seq_n 1`, we will only plan for one beam. If provide two numbers, `--seq_n start_id end_id`, we will plan from #start_id UNTIL #end_id. If more numbers are provided. By default, all the beams will be checked.')
    parser.add_argument('--beam_id', default=None, type=str, help='If specified, overrides the seq_n setting.')
    parser.add_argument('--movement_id', default=None, type=str, help='Compute only for movement with a specific tag, e.g. `A54_M0`.')
    #
    parser.add_argument('--solve_mode', default='nonlinear', choices=SOLVE_MODE, help='solve mode.')
    parser.add_argument('--no_smooth', action='store_true', help='Not apply smoothing on free motions upon a plan is found. Defaults to False.')
    parser.add_argument('--keep_planned_movements', action='store_true', help='Defaults to False.')
    parser.add_argument('-ro', '--remove_overconstraining_free_movement', action='store_true', help='Defaults to False.')
    parser.add_argument('--force_linear_to_free_movement', action='store_true', help='Defaults to False.')
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
    parser.add_argument('--super_res', action='store_true', help='Run the planning with fine resolutions. Defaults to False.')
    parser.add_argument('--solve_timeout', default=600.0, type=float, help='For automatic planning retry, number of seconds before giving up. Defaults to 600.')
    parser.add_argument('--solve_iters', default=1000, type=int, help='For automatic planning retry, number of restart before giving up. Defaults to 1000.')
    parser.add_argument('--mp_algorithm', default='birrt', type=str, choices=MOTION_PLANNING_ALGORITHMS, help='Motion planning algorithms.')
    parser.add_argument('--rrt_iterations', default=400, type=int, help='Number of iterations within one rrt session. Defaults to 400.')
    parser.add_argument('--buffers_for_free_motions', action='store_true', help='Turn on buffering linear motions for free movements, used for narrow passage scenarios. Defaults to False.')
    parser.add_argument('--reachable_range', nargs=2, default=[0.2, 2.40], type=float, help='Reachable range (m) of the robot tcp from the base. Two numbers Defaults to `--reachable_range 0.2, 2.4`. It is possible to relax it to 3.0')
    parser.add_argument('--mesh_split_long_edge_max_length', default=0.0, type=float, help='the range of edges to be split if they are longer than given threshold used in CGAL\'s split mesh edges function. The sampled points are used for performing the polyline (ray-casting) sweep collision check between each pair of configurations in trajectories. ONLY BEAM are checked and NO other tools and robot links is checked! Unit in millimeter. 0.0 will turn this feature off. By default it is set to be 250.0 mm.')
    args = parser.parse_args()

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

    if args.movement_id is not None:
        if args.solve_mode != 'movement_id' and args.solve_mode != 'linear_movement_group':
            LOGGER.error("If movement_id is supplied. Solve mode must be movement_id or linear_movement_group")
            exit()

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
        'force_linear_to_free_movement' : args.force_linear_to_free_movement,
        'mesh_split_long_edge_max_length' : args.mesh_split_long_edge_max_length,
    }
    # ! frame, conf compare, joint flip and allowable collision tolerances are set here
    options.update(get_tolerances(robot, super_res=args.super_res))
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
    if args.beam_id is not None:
        args.seq_n = [process.assembly.sequence.index(args.beam_id)]
    beam_ids = beam_ids_from_argparse_seq_n(process, args.seq_n, movement_id=args.movement_id)

    # * Archive the target movements (if they already exist in the external movement folder)
    # if movement_id is not None, only one movement json will be moved to the `archive` folder
    # otherwise, all the movement under beam_id will be moved
    result_path = get_process_path(args.design_dir, args.problem, subdir=args.problem_subdir)
    ext_movement_path = os.path.dirname(result_path)
    if not args.keep_planned_movements:
        archive_robotic_movements(process, beam_ids, ext_movement_path, movement_id=args.movement_id)

    # * Archive linear motion group:
    if args.solve_mode == 'linear_movement_group' and not args.keep_planned_movements:
        movement = process.get_movement_by_movement_id(args.movement_id)
        movements = process.get_linear_movement_group(movement)
        for movement in movements:
            moved = move_saved_movement(movement, ext_movement_path)
            if moved: LOGGER.info("Linear Movement group member {} removed.".format(movement.movement_id))
        # ! This is not finished because the neighbouring Free Move also need to be deleted.

    # * Archive neighbouring already planned Free Movements
    if args.solve_mode is not 'movement_id' and args.remove_overconstraining_free_movement:
        for beam_id in beam_ids:
            movements = process.get_robotic_movements_by_beam_id(beam_id)
            first_robotic_movement = movements[0]
            last_robotic_movement = movements[-1]
            if (isinstance(first_robotic_movement, RoboticLinearMovement)
                # and process.movement_has_start_robot_config(first_robotic_movement)
                # and process.movement_has_end_robot_config(first_robotic_movement)
                ):
                    neighbour_movement = process.get_prev_robotic_movement(first_robotic_movement)
                    if isinstance(neighbour_movement, RoboticFreeMovement):
                        moved = move_saved_movement(neighbour_movement, ext_movement_path)
                        if moved: LOGGER.warning(colored("Neighbouring Free Movement {} removed due to overconstraint with first Linear Movement {} of {}".format(
                            neighbour_movement.movement_id, first_robotic_movement.movement_id, beam_id
                        ), "red"))

            if (isinstance(last_robotic_movement, RoboticLinearMovement)
                # and process.movement_has_start_robot_config(last_robotic_movement)
                # and process.movement_has_end_robot_config(last_robotic_movement)
                ):
                    neighbour_movement = process.get_next_robotic_movement(last_robotic_movement)
                    if isinstance(neighbour_movement, RoboticFreeMovement):
                        moved = move_saved_movement(neighbour_movement, ext_movement_path)
                        if moved: LOGGER.warning(colored("Neighbouring Free Movement {} removed due to overconstraint with last Linear Movement {} of {}".format(
                            neighbour_movement.movement_id, last_robotic_movement.movement_id, beam_id
                        ), "red"))

    # * load previously planned movements
    process.load_external_movements(ext_movement_path)

    # * Initialize (only needed once) collision objects and tools
    set_initial_state(client, robot, process, initialize=True, options=options)
    for beam_id in beam_ids:
        LOGGER.debug('-'*20)
        success, _ = plan_for_beam_id_with_restart(client, robot, process, beam_id, args, options=options)

    LOGGER.info('Planning done.')
    client.disconnect()

if __name__ == '__main__':
    main()
