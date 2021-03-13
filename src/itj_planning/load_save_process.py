import json
import argparse
from termcolor import cprint
from compas.utilities import DataDecoder, DataEncoder
from .parsing import get_process_path, parse_process, TEMP_DESIGN_DIR, DESIGN_DIR

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--problem', default='twelve_pieces_process.json', # pavilion.json
                        help='The name of the problem to solve')
    parser.add_argument('--parse_temp', action='store_true',
                        help='Parse and save the temp process file.')
    args = parser.parse_args()
    # * Load json
    process = parse_process(args.problem, parse_temp=args.parse_temp)
    # * Compute Actions
    verbose = False
    process.create_actions_from_sequence(verbose=verbose)
    process.assign_tools_to_actions(verbose=verbose)
    # I havent tried these optimization features so far but worth trying
    # process.optimize_actions_place_pick_gripper()
    # process.optimize_actions_place_pick_clamp()
    # * Create Movements and print log file
    process.create_movements_from_actions(verbose=verbose)
    # process.debug_print_process_actions_movements(log_file_path)
    # * Compute States
    process.compute_initial_state()
    process.compute_intermediate_states(verbose=verbose)
    # Save json to original location (optional)
    print('---')
    path = get_process_path(args.problem, file_dir=TEMP_DESIGN_DIR if args.parse_temp else DESIGN_DIR)
    with open(path, 'w') as f:
        json.dump(process, f, cls=DataEncoder, indent=None, sort_keys=True)
    cprint('Process saved to %s' % path, 'green')

if __name__ == '__main__':
    main()
