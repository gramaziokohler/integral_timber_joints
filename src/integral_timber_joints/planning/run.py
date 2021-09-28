import os
import time
import logging
import numpy as np
import argparse
import pybullet_planning as pp
from pybullet_planning.motion_planners.utils import elapsed_time

from termcolor import cprint, colored
from copy import deepcopy

from compas.robots import Joint
from pybullet_planning import wait_if_gui, wait_for_user, LockRenderer, WorldSaver, HideOutput

from integral_timber_joints.planning.parsing import parse_process, save_process_and_movements, get_process_path, save_process
from integral_timber_joints.planning.robot_setup import load_RFL_world, GANTRY_ARM_GROUP
from integral_timber_joints.planning.utils import notify, print_title, beam_ids_from_argparse_seq_n, color_from_success
from integral_timber_joints.planning.state import set_state, set_initial_state
from integral_timber_joints.planning.visualization import visualize_movement_trajectory
from integral_timber_joints.planning.solve import get_movement_status, MovementStatus, compute_selected_movements
from integral_timber_joints.planning.smoothing import smooth_movement_trajectory

from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotScrewdriverSyncLinearMovement
from integral_timber_joints.process.movement import RoboticMovement

logging.basicConfig(filename='run.log', format='%(asctime)s | %(levelname)s | %(message)s', level=logging.DEBUG)

SOLVE_MODE = [
    'nonlinear',
    'linear',
    'movement_id', # 'Compute only for movement with a specific tag, e.g. `A54_M0`.'
    'free_motion_only', # 'Only compute free motions.'
    'propagate_only', # 'Only do state propagation and impacted movement planning.'
]

# * Next steps
# TODO use linkstatistics joint weight and resolutions
# TODO backtrack in case of subsequent sampling cannot find a solution (linear movement with one end specified)

##############################################

def plan_for_beam_id_with_restart(client, robot, process, beam_id, args, options=None):
    solve_timeout = options.get('solve_timeout', 600)
    solve_iters = options.get('solve_iters', 40)
    return_upon_success = options.get('return_upon_success', True)
    ignore_taught_confs = options.get('ignore_taught_confs', False)
    runtime_data = {}

    start_time = time.time()
    trial_i = 0
    while elapsed_time(start_time) < solve_timeout and trial_i < solve_iters:
        print('#'*10)
        print('Beam {} | {} | Inner Trail #{} | time {:.2f}'.format(beam_id, args.solve_mode, trial_i, elapsed_time(start_time)))
        options['profiles'] = {}
        success = compute_movements_for_beam_id(client, robot, process, beam_id, args, options=options)
        runtime_data[trial_i] = {}
        runtime_data[trial_i]['success'] = success
        runtime_data[trial_i]['profiles'] = deepcopy(options['profiles'])
        cprint('Return success: {}'.format(success), 'green' if success else 'red')
        if return_upon_success and success:
            break
        trial_i += 1
        copy_st_time = time.time()
        process = parse_process(args.design_dir, args.problem, subdir=args.problem_subdir)
        if ignore_taught_confs:
            for m in process.movements:
                process.set_movement_end_robot_config(m, None)
        client.disconnect()
        client, robot, _ = load_RFL_world(viewer=args.viewer, verbose=False)
        set_initial_state(client, robot, process, disable_env=False, reinit_tool=False)
        copy_time = elapsed_time(copy_st_time)
        solve_timeout += copy_time
        print('Restarting client/process takes {} | total timeout {}'.format(copy_time, solve_timeout))
        # ! process/client reset time shouldn't be counted in

    return success, runtime_data

def compute_movements_for_beam_id(client, robot, process, beam_id, args, options=None):
    # ! Returns a boolean flag for planning success
    # if args.verbose:
    #     print_title('0) Before planning')
    #     process.get_movement_summary_by_beam_id(beam_id)
        # wait_for_user()
    all_movements = process.get_movements_by_beam_id(beam_id)
    options['samplig_order_counter'] = 0
    options['enforce_continuous'] = True
    if 'movement_id_filter' in options:
        del options['movement_id_filter']

    with LockRenderer(not args.debug) as lockrenderer:
        options['lockrenderer'] = lockrenderer
        # TODO loop and backtrack
        # TODO have to find a way to recover movements
        altered_movements = []
        with HideOutput(): #args.verbose
            if args.solve_mode == 'nonlinear':
                success, altered_ms = compute_selected_movements(client, robot, process, beam_id, 1, [RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotScrewdriverSyncLinearMovement],
                    [MovementStatus.neither_done, MovementStatus.one_sided],
                    options=options, diagnosis=args.diagnosis)
                if not success:
                    logging.info('A plan NOT found using nonlinear planning at stage 1 for beam {}!'.format(beam_id))
                    print('No success for nonlinear planning.')
                    return False
                else:
                    altered_movements.extend(altered_ms)

                # TODO if fails remove the related movement's trajectory and try again
                success, altered_ms = compute_selected_movements(client, robot, process, beam_id, 0, [RoboticLinearMovement],
                    [MovementStatus.one_sided],
                    options=options, diagnosis=args.diagnosis)
                if not success:
                    logging.info('A plan NOT found using nonlinear planning at stage 2 for beam {}!'.format(beam_id))
                    print('No success for nonlinear planning.')
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
                    logging.info('A plan NOT found using nonlinear planning at stage 3 for beam {}!'.format(beam_id))
                    print('No success for nonlinear planning.')
                    return False
                else:
                    altered_movements.extend(altered_ms)

                # Ideally, all the free motions should have both start and end conf specified.
                # one_sided is used to sample the start conf if none is given (especially when `arg.problem_dir = 'YJ_tmp'` is not used).
                success, altered_ms = compute_selected_movements(client, robot, process, beam_id, 0, [RoboticFreeMovement],
                    [MovementStatus.both_done, MovementStatus.one_sided],
                    options=options, diagnosis=args.diagnosis)
                if not success:
                    logging.info('A plan NOT found using nonlinear planning at stage 4 for beam {}!'.format(beam_id))
                    print('No success for nonlinear planning.')
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
                    print('No success for linear (chained) planning.')
                    logging.info('A plan NOT found using linear (chained) planning {}!'.format(beam_id))
                    return False
                altered_movements.extend(altered_ms)

            elif args.solve_mode == 'free_motion_only':
                success, altered_ms = compute_selected_movements(client, robot, process, beam_id, None, [RoboticFreeMovement],
                    [MovementStatus.both_done, MovementStatus.one_sided],
                    options=options, diagnosis=args.diagnosis)
                if not success:
                    print('No success for free motions')
                    logging.info('A plan NOT found for free motion in beam id {}!'.format(beam_id))
                    return False
                altered_movements.extend(altered_ms)

            elif args.solve_mode == 'movement_id':
                # * compute for movement_id movement
                cprint('Computing only for {}'.format(args.movement_id), 'yellow')
                options['movement_id_filter'] = [args.movement_id]
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
                    logging.info('A plan NOT found for movement_id {}!'.format(args.movement_id))
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
        print('Smoothing trajectory...')
        smoothed_movements = []
        with pp.LockRenderer(): # not args.debug):
            for altered_m in altered_movements:
                if not isinstance(altered_m, RoboticFreeMovement):
                    continue
                success, smoothed_traj, msg = smooth_movement_trajectory(client, process, robot, altered_m, options=options)
                altered_m.trajectory = smoothed_traj
                smoothed_movements.append(altered_m)
                cprint('Smooth success: {} | msg: {}'.format(success, msg), color_from_success(success))
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
        print('='*20)
        print_title('Visualize results')
        wait_if_gui('Start simulating results. Press enter to start.')
        set_state(client, robot, process, process.initial_state)
        for m_id in sorted(viz_movements.keys()):
            visualize_movement_trajectory(client, robot, process, viz_movements[m_id], step_sim=args.step_sim)

    if args.verbose:
        notify('A plan has been found for beam id {}!'.format(beam_id))

    logging.info('A plan has been found for beam id {}!'.format(beam_id))
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
    parser.add_argument('--verbose', action='store_false', help='Print out verbose. Defaults to True.')
    #
    parser.add_argument('--reinit_tool', action='store_true', help='Regenerate tool URDFs.')
    #
    parser.add_argument('--low_res', action='store_true', help='Run the planning with low resolutions. Defaults to True.')
    parser.add_argument('--max_distance', default=0.0, type=float, help='Buffering distance for collision checking, larger means safer. Defaults to 0.')
    parser.add_argument('--solve_timeout', default=600.0, type=float, help='For automatic planning retry, number of seconds before giving up. Defaults to 600.')

    args = parser.parse_args()
    print('Arguments:', args)
    print('='*10)
    if args.movement_id is not None:
        args.solve_mode = 'movement_id'

    # * Connect to path planning backend and initialize robot parameters
    client, robot, _ = load_RFL_world(viewer=args.viewer or args.diagnosis or args.watch or args.step_sim)

    #########
    # * Load process and recompute actions and states
    process = parse_process(args.design_dir, args.problem, subdir=args.problem_subdir)

    # * force load external if only planning for the free motions
    args.load_external_movements = args.load_external_movements or args.solve_mode == 'free_motion_only' or args.solve_mode == 'movement_id'
    if args.load_external_movements:
        result_path = get_process_path(args.design_dir, args.problem, subdir='results')
        ext_movement_path = os.path.dirname(result_path)
        cprint('Loading external movements from {}'.format(ext_movement_path), 'cyan')
        process.load_external_movements(ext_movement_path)

    #########

    # * threshold to check joint flipping
    joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    joint_types = robot.get_joint_types_by_names(joint_names)
    # 0.1 rad = 5.7 deg
    joint_jump_threshold = {jt_name : np.pi/6 \
            if jt_type in [Joint.REVOLUTE, Joint.CONTINUOUS] else 0.1 \
            for jt_name, jt_type in zip(joint_names, joint_types)}
    options = {
        'debug' : args.debug,
        'diagnosis' : args.diagnosis,
        'low_res' : args.low_res,
        'distance_threshold' : 0.0012, # in meter
        'frame_jump_tolerance' : 0.0012, # in meter
        'verbose' : args.verbose,
        'jump_threshold' : joint_jump_threshold,
        'max_distance' : args.max_distance,
        'propagate_only' : args.solve_mode == 'propagate_only',
        'solve_timeout': args.solve_timeout,
    }
    if args.smooth:
        options.update(
            {'smooth_iterations' : 150,
             'max_smooth_time' : 60,
             }
        )

    set_initial_state(client, robot, process, reinit_tool=args.reinit_tool)

    # restart attempts for each beam
    success = False
    num_trails = 1
    beam_ids = beam_ids_from_argparse_seq_n(process, args.seq_n, args.movement_id)
    for beam_id in beam_ids:
        print('-'*20)
        s_i = process.assembly.sequence.index(beam_id)
        print('({}) Beam#{}'.format(s_i, beam_id))
        for attempt_i in range(num_trails):
            print('Beam {} | Outer Trail {} #{}'.format(beam_id, args.solve_mode, attempt_i))
            process = parse_process(args.design_dir, args.problem, subdir=args.problem_subdir)
            success, trial_data = plan_for_beam_id_with_restart(client, robot, process, beam_id, args, options=options)
            if success:
                break

    cprint('Planning done.', 'green')
    client.disconnect()

if __name__ == '__main__':
    main()
