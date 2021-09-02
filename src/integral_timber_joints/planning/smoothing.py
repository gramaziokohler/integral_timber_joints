import os, time
import argparse
from termcolor import cprint, colored
from itertools import product, combinations
import numpy as np

from compas.robots import Joint

import pybullet_planning as pp
from pybullet_planning import wait_if_gui, wait_for_user
from pybullet_planning import  link_from_name

from compas_fab_pychoreo.backend_features.pychoreo_configuration_collision_checker import PyChoreoConfigurationCollisionChecker
from compas_fab_pychoreo.backend_features.pychoreo_trajectory_smoother import PyChoreoTrajectorySmoother

from compas_fab_pychoreo.utils import compare_configurations
from compas_fab_pychoreo.conversions import pose_from_frame, frame_from_pose

from integral_timber_joints.planning.parsing import parse_process, get_process_path, save_process_and_movements
from integral_timber_joints.planning.robot_setup import load_RFL_world, GANTRY_ARM_GROUP
from integral_timber_joints.planning.state import set_state
from integral_timber_joints.planning.utils import print_title, FRAME_TOL, color_from_success
from integral_timber_joints.planning.visualization import visualize_movement_trajectory

from integral_timber_joints.process import RoboticMovement, RoboticFreeMovement

#################################

def smooth_movement_trajectory(client, process, robot, movement, options=None):
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
    parser.add_argument('--seq_i', default=0, type=int, help='individual step to plan.')
    parser.add_argument('--batch_run', action='store_true', help='Batch run. Will turn `--seq_i` as run from.')
    #
    parser.add_argument('--id_only', default=None, type=str, help='Compute only for movement with a specific tag, e.g. `A54_M0`.')
    #
    parser.add_argument('--write', action='store_true', help='Write output json.')
    #
    parser.add_argument('--debug', action='store_true', help='Debug mode')
    parser.add_argument('--verbose', action='store_false', help='Print out verbose. Defaults to True.')
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

    # * Double check entire solution is valid
    for beam_id in process.assembly.sequence:
        if not process.dependency.beam_all_valid(beam_id):
            process.dependency.compute_all(beam_id)
            assert process.dependency.beam_all_valid(beam_id)

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
        'smooth_iterations' : 800,
        'max_smooth_time' : 120,
    }

    full_seq_len = len(process.assembly.sequence)
    assert args.seq_i < full_seq_len and args.seq_i >= 0
    if args.batch_run:
        # all beams
        beam_ids = [process.assembly.sequence[si] for si in range(args.seq_i, full_seq_len)]
    elif args.id_only:
        # only one movement
        beam_ids = [process.get_beam_id_from_movement_id(args.id_only)]
    else:
        # only one beam
        beam_ids = [process.assembly.sequence[args.seq_i]]

    altered_movements = []
    for beam_id in beam_ids:
        seq_i = process.assembly.sequence.index(beam_id)
        if not args.id_only:
            print('='*20)
            cprint('(Seq#{}) Beam {}'.format(seq_i, beam_id), 'yellow')
        # if args.debug:
        #     process.get_movement_summary_by_beam_id(beam_id)

        all_movements = process.get_movements_by_beam_id(beam_id)
        for i, m in enumerate(all_movements):
            if args.id_only and m.movement_id != args.id_only:
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
                    visualize_movement_trajectory(client, robot, process, m, step_sim=args.step_sim)
                m.trajectory = smoothed_traj
                altered_movements.append(m)
                if args.watch:
                    print('>'*20)
                    wait_if_gui('Trajectory AFTER smoothing. Press enter to start.')
                    visualize_movement_trajectory(client, robot, process, m, step_sim=args.step_sim)
                    wait_if_gui('simulation ends.')
            else:
                wait_for_user()

    if args.write:
        save_process_and_movements(args.design_dir, args.problem, process, altered_movements, overwrite=False,
            include_traj_in_process=False, save_temp=False)

    client.disconnect()

if __name__ == '__main__':
    main()

