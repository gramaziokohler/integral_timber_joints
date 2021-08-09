import json
import argparse
from termcolor import cprint
from compas.utilities import DataDecoder, DataEncoder
from integral_timber_joints.planning.parsing import get_process_path, parse_process, TEMP_SUBDIR, DESIGN_DIR
from integral_timber_joints.process import RobotClampAssemblyProcess

def recompute_action_states(process, verbose = False):
    #type: (RobotClampAssemblyProcess, bool) -> None
    print("deprecated Warning: recompute_action_states() should be retired. Use process.dependency.compute(beam_id, process.compute_all) if you want to make sure everything is computed.")

    for beam_id in process.assembly.sequence:
        process.create_actions_from_sequence(beam_id, verbose=verbose)
    process.assign_tools_to_actions(verbose=verbose)

    # I havent tried these optimization features so far but worth trying
    # process.optimize_actions_place_pick_gripper()
    # process.optimize_actions_place_pick_clamp()

    # * Create Movements and print log file
    for beam_id in process.assembly.sequence:
        process.create_movements_from_action(beam_id, verbose=verbose)

    # process.debug_print_process_actions_movements(log_file_path)
    # * Compute States
    # ! We dont do that anymore. The file you load should have everything computed already

    # * Save json to original location

###########################################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--problem', default='twelve_pieces_process.json', # pavilion.json
                        help='The name of the problem to solve')
    parser.add_argument('--problem_subdir', default='.', # pavilion.json
                        help='subdir of the process file, default to `.`. Popular use: `YJ_tmp`, `<time stamp>`')
    args = parser.parse_args()
    # * Load json
    process = parse_process(args.problem, subdir=args.problem_subdir)
    # * Compute Actions
    verbose = False
    recompute_action_states(process, verbose)

    print('---')
    path = get_process_path(args.problem, subdir=args.problem_subdir)
    with open(path, 'w') as f:
        json.dump(process, f, cls=DataEncoder, indent=None, sort_keys=True)
    cprint('Process saved to %s' % path, 'green')

########################################

if __name__ == '__main__':
    main()
