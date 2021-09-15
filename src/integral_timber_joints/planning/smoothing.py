import os, time
import argparse
from termcolor import cprint, colored
from itertools import product, combinations
import numpy as np

import pybullet_planning as pp
from pybullet_planning import wait_if_gui, wait_for_user

from compas_fab_pychoreo.backend_features.pychoreo_trajectory_smoother import PyChoreoTrajectorySmoother

from compas_fab_pychoreo.utils import compare_configurations
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose

from integral_timber_joints.planning.parsing import parse_process, get_process_path, save_process_and_movements
from integral_timber_joints.planning.robot_setup import load_RFL_world, GANTRY_ARM_GROUP
from integral_timber_joints.planning.state import set_state
from integral_timber_joints.planning.utils import print_title, FRAME_TOL, color_from_success, beam_ids_from_argparse_seq_n
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
    # * update state
    start_state = process.get_movement_start_scene(movement)
    set_state(client, robot, process, start_state, options=options)

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
    process.set_initial_state_robot_config(process.robot_initial_config)
    set_state(client, robot, process, process.initial_state, initialize=True,
        options={'debug' : False, 'reinit_tool' : False})

    options = {
        'debug' : args.debug,
        'diagnosis' : args.diagnosis,
        'verbose' : args.verbose,
        'smooth_iterations' : 200,
        'max_smooth_time' : 60,
    }

    beam_ids = beam_ids_from_argparse_seq_n(process, args.seq_n, args.movement_id)
    altered_movements = []
    for beam_id in beam_ids:
        seq_i = process.assembly.sequence.index(beam_id)
        if not args.movement_id:
            print('='*20)
            cprint('(Seq#{}) Beam {}'.format(seq_i, beam_id), 'yellow')
        # if args.debug:
        #     process.get_movement_summary_by_beam_id(beam_id)

        all_movements = process.get_movements_by_beam_id(beam_id)
        for i, m in enumerate(all_movements):
            if args.movement_id and m.movement_id != args.movement_id:
                continue
            if not isinstance(m, RoboticFreeMovement):
                continue
            m_index = process.movements.index(m)
            print('-'*10)
            print_title('(MovementIndex={}) (Seq#{}-#{}) {}'.format(m_index, seq_i, i, m.short_summary))
            with pp.LockRenderer(): # not args.debug):
                success, smoothed_traj, msg = smooth_movement_trajectory(client, process, robot, m, options=options)
            cprint('Smooth success: {} | msg: {}'.format(success, msg), color_from_success(success))

            if success:
                if args.watch and args.debug:
                    print('='*20)
                    wait_if_gui('Trajectory before smoothing. Press enter to start.')
                    visualize_movement_trajectory(client, robot, process, m, step_sim=args.step_sim,
                        draw_polylines=True, line_color=pp.RED)
                m.trajectory = smoothed_traj
                altered_movements.append(m)
                if args.watch:
                    print('>'*20)
                    wait_if_gui('Trajectory AFTER smoothing. Press enter to start.')
                    visualize_movement_trajectory(client, robot, process, m, step_sim=args.step_sim,
                        draw_polylines=True, line_color=pp.GREEN)
                    wait_if_gui('simulation ends.')
                pp.remove_all_debug()
            else:
                wait_for_user()

    if args.write:
        save_process_and_movements(args.design_dir, args.problem, process, altered_movements, overwrite=False,
            include_traj_in_process=False, movement_subdir='smoothed_movements')

    client.disconnect()

if __name__ == '__main__':
    main()

