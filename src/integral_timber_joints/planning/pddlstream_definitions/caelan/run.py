import os, sys
import json
import argparse
from termcolor import cprint

HERE = os.path.abspath(os.path.dirname(__file__))
try:
    # prioritize local pddlstream first
    # add your PDDLStream path here: https://github.com/caelan/pddlstream
    sys.path.append(os.environ['PDDLSTREAM_PATH'])
except KeyError:
    cprint('No `PDDLSTREAM_PATH` found in the env variables, using pddlstream submodule', 'yellow')
    sys.path.extend([
       os.path.abspath(os.path.join(HERE, '..', '..', '..', 'external', 'pddlstream'))
    ])

try:
    sys.path.append(os.environ['PYPLANNERS_PATH'])
except KeyError:
    cprint('No `PYPLANNERS_PATH` found in the env variables, using pyplanner submodule', 'yellow')
    here = os.path.abspath(os.path.dirname(__file__))
    pyplanner_path = os.path.abspath(os.path.join(here, '..', '..', 'external', 'pyplanners'))
    # Inside PDDLStream, it will look for 'PYPLANNERS_PATH'
    os.environ['PYPLANNERS_PATH'] = pyplanner_path

import pddlstream
cprint("Using pddlstream from {}".format(pddlstream.__file__), 'yellow')

import strips # pyplanners
cprint("Using strips (pyplanners) from {}".format(strips.__file__), 'yellow')

################################

from pddlstream.utils import read
from pddlstream.language.stream import StreamInfo, PartialInputs, WildOutput, DEBUG
from pddlstream.language.constants import And, PDDLProblem, Equal, print_plan
from pddlstream.language.generator import from_gen_fn, from_fn, from_test

from pddlstream.algorithms.downward import set_cost_scale, parse_action
from pddlstream.algorithms.meta import solve
from pddlstream.utils import INF
from pddlstream.language.constants import print_plan, is_plan

class EmptyTrajectory(object):
    def __init__(self, tag=''):
        self.tag = tag
    def __repr__(self):
        return 'Traj-{}'.format(self.tag)

class EmptyConfiguration(object):
    def __init__(self, tag=''):
        self.tag = tag
    def __repr__(self):
        return 'Conf-{}'.format(self.tag)

######################################

def get_itj_pddl_problem_from_json(json_file_name, use_partial_order=True, debug=False, reset_to_home=False, consider_transition=False):
    json_file_path = os.path.join(HERE, json_file_name)
    with open(json_file_path, 'r') as f:
        process = json.load(f)
    cprint('Symbolic process json parsed from {}'.format(json_file_path), 'green')

    domain_pddl = read(os.path.join(HERE, 'debug_domain.pddl'))
    stream_pddl = read(os.path.join(HERE, 'debug_stream.pddl'))

    init = []

    home_conf = EmptyConfiguration('Home')
    constant_map = {}

    init.extend([
        ('RobotToolChangerEmpty',),
    ])
    if consider_transition:
        init.extend([
        ('ConsiderTransition',),
        ('RobotConf', home_conf),
        ('RobotAtConf', home_conf),
        ('CanFreeMove',),
        ])

    beam_seq = []
    for e_data in process['assembly']['sequence']:
        e = e_data['beam_id']
        beam_seq.append(e)
        init.extend([
            ('Element', e),
            ('AtRack', e),
            ('IsElement', e)
        ])
        if e_data['grounded']:
            init.extend([
                ('Grounded', e),
                # ('Joint', e, 'alum_foundation'),
                ])

    for j_data in process['assembly']['joints']:
        j = j_data['joint_id']
        init.extend([
            ('Joint', j[0], j[1]),
            ('Joint', j[1], j[0]),
            ('NoToolAtJoint', j[0], j[1]),
            ('NoToolAtJoint', j[1], j[0]),
        ])

    if use_partial_order:
        beam_seq_list = beam_seq
        for e1, e2 in zip(beam_seq_list[:-1], beam_seq_list[1:]):
            init.append(('Order', e1, e2))

    for c_name in process['clamps']:
        init.extend([
            ('Clamp', c_name),
            ('IsClamp', c_name),
            ('IsTool', c_name),
            ('AtRack', c_name),
            ('ToolNotOccupiedOnJoint', c_name),
        ])

    # * tool type
    for j_data in process['assembly']['joints']:
        j = j_data['joint_id']
        joint_clamp_type = j_data['tool_type']
        for c_name, c in process['clamps'].items():
            if c['type_name'] == joint_clamp_type:
                init.extend([
                    ('JointToolTypeMatch', j[0], j[1], c_name),
                    ('JointToolTypeMatch', j[1], j[0], c_name),
                ])

    for g_name in process['grippers']:
        init.extend([
            ('Gripper', g_name),
            ('IsGripper', g_name),
            ('IsTool', g_name),
            ('AtRack', g_name),
        ])
    # * gripper type
    for beam_data in process['assembly']['sequence']:
        beam_gripper_type = beam_data["beam_gripper_type"]
        for g_name, g_data in process['grippers'].items():
            if g_data['type_name'] == beam_gripper_type:
                init.append(('GripperToolTypeMatch', beam_data['beam_id'], g_name))

    if debug:
        stream_map = DEBUG
    else:
        stream_map = {
            'sample-move': from_fn(lambda conf1, conf2: (EmptyTrajectory(),)),
            'sample-pick-tool': from_fn(lambda obj: (EmptyConfiguration(), EmptyConfiguration(), EmptyTrajectory())),
            'sample-place-tool': from_fn(lambda obj: (EmptyConfiguration(), EmptyConfiguration(), EmptyTrajectory())),
            'sample-pick-element': from_fn(lambda obj: (EmptyConfiguration(), EmptyConfiguration(), EmptyTrajectory())),
            'sample-place-element': from_fn(lambda obj: (EmptyConfiguration(), EmptyConfiguration(), EmptyTrajectory())),
            # 'inverse-kinematics':  from_fn(lambda p: (p + GRASP,)),
            # 'test-cfree': from_test(lambda *args: not collision_test(*args)),
        }

    goal_literals = []
    goal_literals.extend(('Assembled', e) for e in beam_seq)
    if reset_to_home:
        goal_literals.extend(('AtRack', t.name) for t in list(process.clamps) + list(process.grippers))
        if consider_transition:
            goal_literals.append(('RobotAtConf', home_conf))
    goal = And(*goal_literals)

    pddlstream_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)
    return pddlstream_problem

###############################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--algorithm', default='incremental', help='PDDLSteam planning algorithm.')
    args = parser.parse_args()
    print('Arguments:', args)

    debug_problem_name = "nine_pieces_process_symbolic.json"
    debug_pddl_problem = get_itj_pddl_problem_from_json(debug_problem_name, use_partial_order=True, debug=True)

    print()
    print('Goal:', debug_pddl_problem.goal)
    print()

    set_cost_scale(1)
    solution = solve(debug_pddl_problem, algorithm=args.algorithm,
                     max_time=60,
                     unit_costs=True,
                     max_planner_time=300,
                     debug=0, verbose=0) #, planner=discrete_planner)

    plan, cost, evaluations = solution
    plan_success = is_plan(plan)

    print('-'*10)
    print_plan(plan)
    cprint('Planning {}'.format('succeeds' if plan_success else 'fails'), 'green' if plan_success else 'red')

if __name__ == '__main__':
    main()
