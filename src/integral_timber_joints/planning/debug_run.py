import argparse
from pybullet_planning.interfaces.env_manager.user_io import wait_if_gui
from termcolor import cprint
import pybullet_planning as pp

from integral_timber_joints.planning.robot_setup import load_RFL_world
from integral_timber_joints.planning.state import set_state, set_initial_state
from integral_timber_joints.planning.parsing import parse_process, save_process_and_movements, get_process_path, save_process
from integral_timber_joints.planning.rhino_interface import get_ik_solutions
from integral_timber_joints.process import movement

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--problem', default='nine_pieces_process.json', # twelve_pieces_process.json
                        help='The name of the problem to solve')
    parser.add_argument('--problem_subdir', default='.', # pavilion.json
                        help='subdir of the process file, default to `.`. Popular use: `results`')
    parser.add_argument('--movement_id', default='A0_M1', type=str, help='Compute only for movement with a specific tag, e.g. `A54_M0`.')

    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
    parser.add_argument('--debug', action='store_true', help='debug mode')
    args = parser.parse_args()
    print('Arguments:', args)
    print('='*10)

    process = parse_process(args.problem, subdir=args.problem_subdir)
    # result_path = get_process_path(args.problem, subdir='results')
    for beam_id in process.assembly.sequence:
        if not process.dependency.beam_all_valid(beam_id):
            process.dependency.compute_all(beam_id)
            assert process.dependency.beam_all_valid(beam_id)

    # * Connect to path planning backend and initialize robot parameters
    # viewer or diagnosis or view_states or watch or step_sim,
    client, robot, _ = load_RFL_world(viewer=args.viewer, verbose=False)
    set_initial_state(client, robot, process, disable_env=False, reinit_tool=False, debug=False)
    # pp.wait_if_gui('Initial state loaded.')

    m = process.get_movement_by_movement_id(args.movement_id)
    print(m.short_summary)

    start_scene = process.get_movement_start_scene(m)
    options = {'debug' : args.debug, 'verbose' : True}
    set_state(client, robot, process, start_scene, options=options)
    wait_if_gui('Start scene.')

    end_scene = process.get_movement_end_scene(m)
    set_state(client, robot, process, end_scene, options=options)
    wait_if_gui('End scene.')

    with pp.LockRenderer(not args.debug):
        result = get_ik_solutions(process, process.movements.index(m))
    # conf = get_ik_solutions(process, 236)
    if result[0]:
        client.set_robot_configuration(robot, result[1])
    wait_if_gui('IK solution found.')

    client.disconnect()

if __name__ == '__main__':
    main()
