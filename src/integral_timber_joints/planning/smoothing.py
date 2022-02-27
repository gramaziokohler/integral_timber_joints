import os, time
import argparse
from termcolor import cprint, colored
from itertools import product, combinations
import numpy as np
from tqdm import tqdm

import pybullet_planning as pp
from pybullet_planning import wait_if_gui, wait_for_user

from compas_fab_pychoreo.backend_features.pychoreo_trajectory_smoother import PyChoreoTrajectorySmoother
from compas_fab_pychoreo.backend_features.custom_trajectory_smoother import CustomTrajectorySmoother

from compas_fab_pychoreo.utils import is_configurations_close
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose

from integral_timber_joints.planning.parsing import parse_process, get_process_path, save_movements
from integral_timber_joints.planning.robot_setup import load_RFL_world, GANTRY_ARM_GROUP
from integral_timber_joints.planning.state import set_state
from integral_timber_joints.planning.utils import print_title, color_from_success, beam_ids_from_argparse_seq_n, LOGGER
from integral_timber_joints.planning.visualization import visualize_movement_trajectory

from integral_timber_joints.process import RoboticMovement, RoboticFreeMovement

#################################

def smooth_movement_trajectory(client, process, robot, movement, options=None):
    """
    Returns
    -------
    triplet
        (success_bool, smoothed trajectory, message)
    """
    options = options or {}
    use_custom_smoother = options.get('use_custom_smoother', False)

    # * update state
    start_state = process.get_movement_start_scene(movement)
    set_state(client, robot, process, start_state, options=options)

    if use_custom_smoother:
        traj_smoother = CustomTrajectorySmoother(client)
    else:
        traj_smoother = PyChoreoTrajectorySmoother(client)
    return traj_smoother.smooth_trajectory(robot, movement.trajectory, options=options)

#################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--design_dir', default='210605_ScrewdriverTestProcess',
                        help='problem json\'s containing folder\'s name.')
    parser.add_argument('--problem', default='nine_pieces_process.json', # twelve_pieces_process.json
                        help='The name of the problem to solve')
    parser.add_argument('--problem_subdir', default='results',
                        help='subdir of the process file. Popular use: `.` or `results`')
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
    #
    parser.add_argument('--seq_n', nargs='+', type=int, help='Zero-based index according to the Beam sequence in process.assembly.sequence. If only provide one number, `--seq_n 1`, we will only plan for one beam. If provide two numbers, `--seq_n start_id end_id`, we will plan from #start_id UNTIL #end_id.')
    #
    parser.add_argument('--movement_id', default=None, type=str, help='Compute only for movement with a specific tag, e.g. `A54_M0`.')
    parser.add_argument('--use_custom_smoother', action='store_true', help='Use custom trajectory smoother.')
    #
    parser.add_argument('--write', action='store_true', help='Write output json.')
    #
    parser.add_argument('--debug', action='store_true', help='Debug mode')
    parser.add_argument('--verbose', action='store_true', help='Print out verbose. Defaults to False.')
    parser.add_argument('--diagnosis', action='store_true', help='Diagnosis mode')
    #
    parser.add_argument('--watch', action='store_true', help='Pause after each conf viz.')
    parser.add_argument('--step_sim', action='store_true', help='Pause after each conf viz.')

    args = parser.parse_args()
    print('Arguments:', args)
    print('='*10)

    #########
    # * Load process and recompute actions and states
    process = parse_process(args.design_dir, args.problem, subdir=args.problem_subdir)
    result_path = get_process_path(args.design_dir, args.problem, subdir='results')

    # * force load external movements
    ext_movement_path = os.path.dirname(result_path)
    cprint('Loading external movements from {}'.format(ext_movement_path), 'cyan')
    movements_modified = process.load_external_movements(ext_movement_path)
    assert len(movements_modified) > 0, 'At least one external movements should be loaded for smoothing.'

    #########
    # * Connect to path planning backend and initialize robot parameters
    client, robot, _ = load_RFL_world(viewer=args.viewer or args.diagnosis or args.watch or args.step_sim)
    set_state(client, robot, process, process.initial_state, initialize=True,
        options={'debug' : False, 'reinit_tool' : False})

    options = {
        'debug' : args.debug,
        'diagnosis' : args.diagnosis,
        'verbose' : args.verbose,
        'smooth_iterations' : 200,
        'max_smooth_time' : 60,
        'use_custom_smoother' : args.use_custom_smoother,
    }

    # * gather all the movements to be checked
    all_movements = process.movements
    beam_ids = beam_ids_from_argparse_seq_n(process, args.seq_n, args.movement_id, msg_prefix='Smoothing')
    chosen_movements = []
    for beam_id in beam_ids:
        actions = process.assembly.get_beam_attribute(beam_id, 'actions')
        for action in actions:
            for i, m in enumerate(action.movements):
                global_movement_id = all_movements.index(m)
                # * filter by movement id, support both integer-based index and string id
                if args.movement_id is not None and \
                    (m.movement_id != args.movement_id and args.movement_id != str(global_movement_id)):
                    continue
                # * skip non-RoboticFreeMovement
                if not isinstance(m, RoboticFreeMovement):
                    continue
                chosen_movements.append(m)

    altered_movements = []
    movements_need_fix = []
    movements_failure_reasons = []
    with tqdm(total=len(chosen_movements), desc='smoothing movements') as pbar:
        for m in chosen_movements:
            pbar.set_postfix_str(f'{m.movement_id}:{m.__class__.__name__}, {m.tag}')

            with pp.LockRenderer(): # not args.debug):
                success, smoothed_traj, msg = smooth_movement_trajectory(client, process, robot, m, options=options)
            LOGGER.debug('Smooth success: {} | msg: {}'.format(success, msg))

            if success:
                if args.watch and args.debug:
                    print('='*20)
                    wait_if_gui('Trajectory before smoothing. Press enter to start.')
                    visualize_movement_trajectory(client, robot, process, m, step_sim=args.step_sim,
                        draw_polylines=True, line_color=pp.RED)

                process.set_movement_trajectory(m, smoothed_traj)
                altered_movements.append(m)

                if args.watch:
                    print('>'*20)
                    wait_if_gui('Trajectory AFTER smoothing. Press enter to start.')
                    visualize_movement_trajectory(client, robot, process, m, step_sim=args.step_sim,
                        draw_polylines=True, line_color=pp.GREEN)
                    wait_if_gui('simulation ends.')
                    pp.remove_all_debug()
            else:
                movements_need_fix.append(m)
                movements_failure_reasons.append(msg)
            pbar.update(1)

    if args.write:
        save_movements(args.design_dir, altered_movements, movement_subdir='smoothed_movements')

    LOGGER.debug('='*20)
    if len(movements_need_fix) == 0:
        LOGGER.info('Congrats, smoothing done!')
    else:
        LOGGER.warning('Movements that requires care and love:')
        for fm, reason in zip(movements_need_fix, movements_failure_reasons):
            global_movement_id = all_movements.index(fm)
            LOGGER.info('(MovementIndex={}) {}: {}'.format(global_movement_id, fm.short_summary, reason))
    LOGGER.debug('='*20)

    client.disconnect()

if __name__ == '__main__':
    main()

