import argparse
import os
import time

import pybullet_planning as pp
from termcolor import cprint

import integral_timber_joints
from integral_timber_joints.planning.robot_setup import load_RFL_world
from integral_timber_joints.planning.rhino_interface import get_ik_solutions
from integral_timber_joints.planning.state import set_state
from integral_timber_joints.planning.utils import beam_ids_from_argparse_seq_n
from integral_timber_joints.process import RobotClampAssemblyProcess, RoboticMovement
from integral_timber_joints.planning.parsing import parse_process, get_process_path
from compas.rpc import Proxy

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--design_dir', default='210916_SymbolicPlanning',
                        help='problem json\'s containing folder\'s name.')
    parser.add_argument('--problem', default='CantiBoxLeft_10pcs_process.json',  # pavilion_process.json
                        help='The name of the problem to solve')
    parser.add_argument('--problem_subdir', default='.',  # pavilion.json
                        help='subdir of the process file, default to `.`. Popular use: `YJ_tmp`, `<time stamp>`')
    #
    parser.add_argument('--seq_n', nargs='+', type=int, help='Zero-based index according to the Beam sequence in process.assembly.sequence. If only provide one number, `--seq_n 1`, we will only plan for one beam. If provide two numbers, `--seq_n start_id end_id`, we will plan from #start_id UNTIL #end_id. If more numbers are provided. By default, all the beams will be checked.')
    parser.add_argument('--movement_id', default=None, type=str, help='Compute only for movement with a specific tag or movement id, e.g. `A54_M0` or `53`. ! We support both str-like movement id or global list index.')
    #
    parser.add_argument('--gantry_attempt', type=int, default=100, help='Number of gantry attempt. Default 100.')
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

    # * use shared client instance
    # Connect to path planning backend and initialize robot parameters
    if not args.proxy:
        client, robot, _ = load_RFL_world(viewer=args.viewer)
        process.set_initial_state_robot_config(process.robot_initial_config)
        set_state(client, robot, process, process.initial_state, initialize=True,
            options={'include_env' : True, 'reinit_tool' : False})

    start_time = time.time()
    all_movements = process.movements
    failed_state_indices = []

    # Connecton to Proxy Server
    if args.proxy:
        rhino_interface = Proxy('integral_timber_joints.planning.rhino_interface', autoreload=False)

    # * Construct options
    options = {
        'viewer': args.viewer,
        'debug': args.debug,
        'diagnosis': args.debug,
        'ik_gantry_attempts': args.gantry_attempt,
        # ! shared client in options so we avoid recreating client every call
    }
    if not args.proxy:
        options.update(
            {'client' : client, 'robot' : robot,}
        )
    print('options: ' , options)

    beam_ids = beam_ids_from_argparse_seq_n(process, args.seq_n, args.movement_id)
    for beam_id in beam_ids:
        actions = process.assembly.get_beam_attribute(beam_id, 'actions')
        for action in actions:
            for i, movement in enumerate(action.movements):
                # Skip non RoboticMovement
                if not isinstance(movement, RoboticMovement):
                    continue

                global_movement_id = all_movements.index(movement)
                # Filter by movement id
                if args.movement_id is not None and \
                    (movement.movement_id != args.movement_id and args.movement_id != str(global_movement_id)):
                    continue

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
                    failed_state_indices.append((movement.movement_id, global_movement_id + 1, msg))

    if failed_state_indices:
        print('='*10)
        cprint("Failed Movement id | State Indices | Error msg", 'yellow')
        for fmsg in failed_state_indices:
            print(fmsg)
    print('Total checking time: {:.3f}'.format(pp.elapsed_time(start_time)))
