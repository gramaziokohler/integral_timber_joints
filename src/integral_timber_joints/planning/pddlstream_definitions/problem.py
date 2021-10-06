import os
import random
from integral_timber_joints.planning.pddlstream_definitions import ITJ_PDDLSTREAM_DEF_DIR
from integral_timber_joints.planning import load_pddlstream
from integral_timber_joints.assembly.beam_assembly_method import BeamAssemblyMethod

from pddlstream.utils import read
from pddlstream.language.stream import StreamInfo, PartialInputs, WildOutput, DEBUG
from pddlstream.language.constants import And, PDDLProblem, Equal, print_plan
from pddlstream.language.generator import from_gen_fn, from_fn, from_test

from compas.robots import Configuration
from compas_fab.robots import Trajectory

class EmptyTrajectory(object):
    def __init__(self, tag=''):
        self.tag = tag or random.random()
    def __repr__(self):
        return 'Traj-{:.2f}'.format(self.tag)

class EmptyConfiguration(object):
    def __init__(self, tag=''):
        self.tag = tag or random.random()
    def __repr__(self):
        return 'Conf-{:.2f}'.format(self.tag)

def get_pddlstream_problem(process, use_partial_order=True, debug=False, reset_to_home=True):
    domain_pddl = read(os.path.join(ITJ_PDDLSTREAM_DEF_DIR, 'symbolic_domain.pddl'))
    stream_pddl = read(os.path.join(ITJ_PDDLSTREAM_DEF_DIR, 'symbolic_stream.pddl'))

    init = []

    home_conf = EmptyConfiguration(0)
    constant_map = {}

    init.extend([
        ('RobotConf', home_conf),
        ('RobotAtConf', home_conf),
        ('CanMove'),
        ('RobotToolChangerEmpty'),
    ])

    beam_seq = process.assembly.sequence
    for e in beam_seq:
        init.extend([
            ('Element', e),
            ('AtRack', e),
            ('IsElement', e)
        ])
        if process.assembly.get_assembly_method(e) == BeamAssemblyMethod.GROUND_CONTACT:
            init.append(('Grounded', e))

    for j in process.assembly.joint_ids():
        init.extend([
            ('Joint', j[0], j[1]),
            ('NoToolAtJoint', j[0], j[1])
        ])

    if use_partial_order:
        for e1, e2 in zip(beam_seq[:-1], beam_seq[1:]):
            init.append(('Order', e1, e2))

    for c in process.clamps:
        init.extend([
            ('Clamp', c.name),
            ('IsClamp', c.name),
            ('IsTool', c.name),
            ('AtRack', c.name),
        ])
    for g in process.grippers:
        init.extend([
            ('Gripper', g.name),
            ('IsGripper', g.name),
            ('IsTool', g.name),
            ('AtRack', g.name),
        ])

    stream_map = {
        # 'sample-move': from_fn(lambda conf1, conf2: (EmptyTrajectory(),)),
        # 'sample-pick-tool': from_fn(lambda obj: (EmptyConfiguration(), EmptyConfiguration(), EmptyTrajectory())),
        # 'sample-place-tool': from_fn(lambda obj: (EmptyConfiguration(), EmptyConfiguration(), EmptyTrajectory())),
        # 'sample-pick-element': from_fn(lambda obj: (EmptyConfiguration(), EmptyConfiguration(), EmptyTrajectory())),
        # 'sample-place-element': from_fn(lambda obj: (EmptyConfiguration(), EmptyConfiguration(), EmptyTrajectory())),
    #     'inverse-kinematics':  from_fn(lambda p: (p + GRASP,)),
    #     'test-cfree': from_test(lambda *args: not collision_test(*args)),
    }

    if debug:
        stream_map = DEBUG

    goal_literals = []
    goal_literals.extend(('Assembled', e) for e in beam_seq)
    if reset_to_home:
        goal_literals.extend(('AtRack', t) for t in list(process.clamps) + list(process.grippers))
        goal_literals.append(('RobotAtConf', home_conf))
    goal = And(*goal_literals)

    pddlstream_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)
    return pddlstream_problem
