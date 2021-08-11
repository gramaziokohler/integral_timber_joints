import argparse
from termcolor import cprint
import pybullet_planning as pp

from integral_timber_joints.planning.robot_setup import load_RFL_world
from integral_timber_joints.planning.run import set_initial_state
from integral_timber_joints.planning.parsing import parse_process, save_process_and_movements, get_process_path, save_process

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--problem', default='nine_pieces_process.json', # twelve_pieces_process.json
                        help='The name of the problem to solve')
    parser.add_argument('--problem_subdir', default='.', # pavilion.json
                        help='subdir of the process file, default to `.`. Popular use: `results`')
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
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
    set_initial_state(client, robot, process, disable_env=False, reinit_tool=False)

    pp.wait_if_gui('Initial state loaded.')

if __name__ == '__main__':
    main()
