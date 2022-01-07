import os, sys
import argparse
from termcolor import cprint

import integral_timber_joints.planning.pddlstream_planning.load_pddlstream
from integral_timber_joints.planning.pddlstream_planning.parse import get_pddlstream_problem
from integral_timber_joints.planning.pddlstream_planning.postprocessing import save_pddlstream_plan_to_itj_process

from integral_timber_joints.planning.parsing import parse_process
from integral_timber_joints.planning.state import set_state, set_initial_state

from pddlstream.algorithms.downward import set_cost_scale, parse_action, get_cost_scale
from pddlstream.algorithms.meta import solve
from pddlstream.utils import INF
from pddlstream.language.constants import print_plan, is_plan
from pddlstream.utils import flatten, Profiler, SEPARATOR, inf_generator, INF

##################################

def main():
    parser = argparse.ArgumentParser()
    # * Problem info
    parser.add_argument('--design_dir', default='210916_SymbolicPlanning', # 211010_CantiBox, 210128_RemodelFredPavilion
                        help='problem json\'s containing folder\'s name.')
    parser.add_argument('--problem', default='nine_pieces_process.json', # CantiBoxLeft_process.json, pavilion_process.json
                        help='The name of the problem to solve (json file\'s name, e.g. "nine_pieces_process.json")')
    parser.add_argument('--problem_subdir', default='.',
                        help='subdir of the process file, default to `.`. Popular use: `results`')
    # * PDDLStream configs
    parser.add_argument('--nofluents', action='store_true', help='Not use fluent facts in stream definitions.')
    parser.add_argument('--algorithm', default='incremental', help='PDDLSteam planning algorithm.')
    parser.add_argument('--pp_search', default='eager', help='pyplanner search configuration.')
    parser.add_argument('--pp_evaluator', default='greedy', help='pyplanner evaluator configuration.')
    parser.add_argument('--pp_h', default='ff', help='pyplanner heuristic configuration.')
    parser.add_argument('--fd_search', default='ff-eager', help='downward search configuration.')
    # * Planning for sub-assembly
    parser.add_argument('--seq_n', nargs='+', type=int, help='Zero-based index according to the Beam sequence in process.assembly.sequence. If only provide one number, `--seq_n 1`, we will only plan for one beam. If provide two numbers, `--seq_n start_id end_id`, we will plan from #start_id UNTIL #end_id. If more numbers are provided. By default, all the beams will be checked.')
    # * Debugs, verbose and visuals
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning, default False')
    parser.add_argument('--write', action='store_true', help='Write output json.')
    parser.add_argument('--save_dir', type=str, default='results',
        help='Subdir in the process design folder to save the process to, defaults to `results`.')
    parser.add_argument('--debug', action='store_true', help='Debug mode.')
    #
    args = parser.parse_args()
    print('Arguments:', args)

    #########
    # * Connect to path planning backend and initialize robot parameters
    # client, robot, _ = load_RFL_world(viewer=args.viewer or args.diagnosis or args.watch or args.step_sim)
    client = None
    robot = None

    #########
    # * Load process and recompute actions and states
    process = parse_process(args.design_dir, args.problem, subdir=args.problem_subdir)
    # set_initial_state(client, robot, process, reinit_tool=args.reinit_tool)

    #########
    # * PDDLStream problem conversion and planning
    cprint('Using {} backend.'.format('pyplanner' if not args.nofluents else 'downward'), 'cyan')
    pddlstream_problem = get_pddlstream_problem(client, process, robot,
        debug=True, reset_to_home=True, use_fluents=not args.nofluents, seq_n=args.seq_n)[0]

    additional_config = {}
    if not args.nofluents:
        additional_config['planner'] = {
            'search': args.pp_search, # eager | lazy | hill_climbing | a_star | random_walk | mcts
            'evaluator': args.pp_evaluator, # 'bfs' | 'uniform' | 'astar' | 'wastar2' | 'wastar3' | 'greedy'
            'heuristic': args.pp_h, # goal | add | ff | max | blind #'heuristic': ['ff', get_bias_fn(element_from_index)],
            'successors': 'all', # all | random | first_goals | first_operators # 'successors': order_fn,
        }
    else:
        # https://github.com/caelan/pddlstream/blob/4914667a13a80831cadaf115a70938e9f93b021e/pddlstream/algorithms/downward.py#L87
        additional_config['planner'] = args.fd_search
        # 'dijkstra' # 'max-astar' # 'lmcut-astar' # 'dijkstra' # 'ff-eager' # | 'add-random-lazy'

    effort_weight = 1. / get_cost_scale()
    with Profiler(num=25):
        solution = solve(pddlstream_problem, algorithm=args.algorithm,
                         max_time=INF,
                         unit_costs=True,
                         success_cost=INF,
                         unit_efforts=True,
                         effort_weight=effort_weight,
                         max_planner_time=INF,
                         debug=args.debug, verbose=1, **additional_config)

    plan, cost, evaluations = solution
    plan_success = is_plan(plan)

    #########
    # * PDDLStream problem conversion and planning
    print('-'*10)
    print_plan(plan)
    cprint('Planning {}'.format('succeeds' if plan_success else 'fails'), 'green' if plan_success else 'red')
    print('Plan length: ', len(plan))

    if plan_success and args.write:
        save_pddlstream_plan_to_itj_process(process, plan, args.design_dir, args.problem, verbose=0, save_subdir=args.save_dir)

if __name__ == '__main__':
    main()
