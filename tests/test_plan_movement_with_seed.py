import os
import json
import logging
import argparse

from compas.data import DataDecoder, DataEncoder
from compas_fab_pychoreo.utils import is_configurations_close
from compas_fab_pychoreo.utils import LOGGER as PYCHOREO_LOGGER

from integral_timber_joints.planning.solve import compute_movement
from integral_timber_joints.planning.robot_setup import load_RFL_world, get_tolerances
from integral_timber_joints.planning.utils import beam_ids_from_argparse_seq_n, LOGGER
from integral_timber_joints.planning.parsing import parse_process, save_process, save_movements, get_process_path, \
    copy_robotic_movements, archive_robotic_movements
from integral_timber_joints.planning.state import set_state, set_initial_state

HERE = os.path.dirname(__file__)

def save_trajectory(traj, file_name='tmp_traj.json'):
    HERE = os.path.dirname(__file__)
    save_path = os.path.join(HERE, file_name)
    with open(save_path, 'w') as f:
        json.dump(traj, f, cls=DataEncoder, indent=None, sort_keys=True)

def parse_trajectory(file_name='tmp_traj.json'):
    save_path = os.path.join(HERE, file_name)
    with open(save_path, 'r') as f:
        traj = json.load(f, cls=DataDecoder)
    return traj

def compare_trajectories(traj0, traj1, options=None):
    options = options or {}
    assert len(traj0.points) == len(traj1.points)
    for conf0, conf1 in zip(traj0.points, traj1.points):
        assert is_configurations_close(conf0, conf1, options=options)

################################

def solve_for_movement(client, robot, process, args, options):
    use_stored_seed = options.get('use_stored_seed', False)
    if args.movement_id.startswith('A'):
        m = process.get_movement_by_movement_id(args.movement_id)
    else:
        m = process.movements[int(args.movement_id)]

    assert compute_movement(client, robot, process, m, options=options)

    if not use_stored_seed:
        # movements without trajectories but end conf set will be saved into the WIP process file
        save_process(args.design_dir, args.problem, process, save_dir=args.problem_subdir)
        save_movements(args.design_dir, [m], save_dir=args.problem_subdir, movement_subdir='movements')
        save_trajectory(m.trajectory, f'{m.movement_id}_trajectory.json')
    else:
        parsed_traj = parse_trajectory(f'{m.movement_id}_trajectory.json')
        compare_trajectories(m.trajectory, parsed_traj, options)

#################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--design_dir', default='210419_HyparHut',
                        help='problem json\'s containing folder\'s name.')
    parser.add_argument('--problem', default='Shelter_process.json', # twelve_pieces_process.json
                        help='The name of the problem to solve (json file\'s name, e.g. "nine_pieces_process.json")')
    parser.add_argument('--problem_subdir', default='seed_test',
                        help='subdir for saving movements, default to `seed_test`.')
    #
    parser.add_argument('--movement_id', type=str, help='Compute only for movement with a specific tag, e.g. `A54_M0`.')
    parser.add_argument('--use_stored_seed', action='store_true', help='Use existing seed.')
    parser.add_argument('--no_smooth', action='store_true', help='Not apply smoothing on free motions upon a plan is found. Defaults to False.')
    #
    parser.add_argument('--debug', action='store_true', help='Debug mode.')
    parser.add_argument('--diagnosis', action='store_true', help='Diagnosis mode, show collisions whenever encountered')
    parser.add_argument('--quiet', action='store_true', help='Disable print out verbose. Defaults to False.')
    args = parser.parse_args()

    logging_level = logging.DEBUG if args.debug else logging.INFO
    LOGGER.setLevel(logging_level)
    PYCHOREO_LOGGER.setLevel(logging_level)

    # * Connect to path planning backend and initialize robot parameters
    client, robot, _ = load_RFL_world(viewer=False)

    #########
    options = {
        'debug' : args.debug,
        'diagnosis' : args.diagnosis,
        'verbose' : not args.quiet,
        'gantry_attempts' : 100, # number of gantry sampling attempts when computing IK
        'solve_timeout': 1200,
        'rrt_iterations': 100,
        'check_sweeping_collision': True,
        'use_stored_seed' : args.use_stored_seed,
    }
    # ! frame, conf compare, joint flip tolerances are set here
    options.update(get_tolerances(robot, super_res=False))
    if not args.no_smooth:
        options.update({
            'smooth_iterations' : 150,
            'max_smooth_time' : 180,
             })

    #########
    # * Load process and recompute actions and states
    # ! parse the WIP process file
    process = parse_process(args.design_dir, args.problem, subdir=args.problem_subdir)
    beam_ids = beam_ids_from_argparse_seq_n(process, None, movement_id=args.movement_id)

    # * archive the target movements
    result_path = get_process_path(args.design_dir, args.problem, subdir=args.problem_subdir)
    ext_movement_path = os.path.dirname(result_path)
    # archive_movements(process, beam_ids, ext_movement_path, movement_id=args.movement_id)

    # ! parse the original, unplanned process file
    source_process = parse_process(args.design_dir, args.problem, subdir='.')

    # ! reset target movements from the original, unplanned process file
    copy_robotic_movements(source_process, process, beam_ids, movement_id=args.movement_id, options=options)

    set_initial_state(client, robot, process, reinit_tool=False, initialize=True)
    solve_for_movement(client, robot, process, args, options)

if __name__ == '__main__':
    main()
