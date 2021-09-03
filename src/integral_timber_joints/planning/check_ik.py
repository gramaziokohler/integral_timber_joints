import argparse
import os
import time

import pybullet_planning as pp
from termcolor import cprint

import integral_timber_joints
from integral_timber_joints.planning.rhino_interface import get_ik_solutions
from integral_timber_joints.process import RobotClampAssemblyProcess, RoboticMovement
from integral_timber_joints.planning.parsing import parse_process, get_process_path
from compas.rpc import Proxy


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--design_dir', default='210605_ScrewdriverTestProcess',
                        help='problem json\'s containing folder\'s name.')
    parser.add_argument('--problem', default='nine_pieces_process.json',  # pavilion_process.json
                        help='The name of the problem to solve')
    parser.add_argument('--problem_subdir', default='.',  # pavilion.json
                        help='subdir of the process file, default to `.`. Popular use: `YJ_tmp`, `<time stamp>`')
    #
    parser.add_argument('--seq_n', type=int, default=None, help='Compute only for movement with index (integer), default to None = compute all.')
    parser.add_argument('--id_only', default=None, type=str, help='Compute only for movement with a specific tag, e.g. `A54_M0` Specifying this will disable --seq_n.')
    parser.add_argument('--gantry_attempt', type=int, default=20, help='Number of gantry attempt. Default 20.')
    #
    parser.add_argument('-p', '--proxy', action='store_true', help='Use Proxy to call checker. Default False')
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning. Default False')
    # parser.add_argument('--reinit_tool', action='store_true', help='Regenerate tool URDFs.')
    parser.add_argument('--debug', action='store_true', help='debug mode.')
    args = parser.parse_args()
    print('Arguments:', args)

    ##########################

    path_to_json = os.path.realpath(os.path.join(os.path.dirname(integral_timber_joints.__file__), '..', '..', 'external', 'itj_design_study', '210605_ScrewdriverTestProcess', 'nine_pieces_process.json'))
    # process = load_process(path_to_json)  # type: RobotClampAssemblyProcess

    process = parse_process(args.design_dir, args.problem, subdir=args.problem_subdir)

    start_time = time.time()
    all_movements = process.movements
    failed_state_indices = []

    # Connecton to Proxy Server
    if args.proxy:
        rhino_interface = Proxy('integral_timber_joints.planning.rhino_interface', autoreload=False)

    if args.id_only:
        args.seq_n = None

    # * Construct options
    options = {
        'viewer': args.viewer,
        'debug': args.debug,
        'ik_gantry_attempts': args.gantry_attempt,
    }
    print('options: ' , options)

    for action in process.actions:
        for i, movement in enumerate(action.movements):
            # Skip non RoboticMovement
            if not isinstance(movement, RoboticMovement):
                continue
            # Skip movement if seq_n is specified
            if args.seq_n is not None and action.seq_n != args.seq_n:
                continue
            # Filter by movement id
            if args.id_only is not None and movement.movement_id != args.id_only:
                continue

            global_movement_id = all_movements.index(movement)
            print('='*10)
            print("Mov#%i/State#%i (%s) A(%s) M(%s) %s" % (global_movement_id, global_movement_id + 1, movement.movement_id, action.__class__.__name__, movement.__class__.__name__, movement.tag))

            # * Proxy call or normal call
            if args.proxy:
                result = rhino_interface.get_ik_solutions(process, global_movement_id, options)
            else:
                result = get_ik_solutions(process, global_movement_id, options)

            # * Inteprete Result
            success, conf, msg = result
            if success:
                cprint("IK Success: %s" % msg, 'green')
            else:
                cprint("- - - WARNING - - - IK Failed: %s" % msg, 'red')
                failed_state_indices.append(global_movement_id + 1)

    print("Failed State Indices: %s " % failed_state_indices)
    print('Total checking time: {:.3f}'.format(pp.elapsed_time(start_time)))
