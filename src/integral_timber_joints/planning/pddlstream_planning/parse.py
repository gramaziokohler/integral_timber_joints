import os
import random
from termcolor import cprint
from collections import defaultdict

import integral_timber_joints.planning.pddlstream_planning.load_pddlstream
from integral_timber_joints.planning.pddlstream_planning import ITJ_PDDLSTREAM_DEF_DIR
from integral_timber_joints.planning.pddlstream_planning.stream import get_ik_fn, get_sample_pick_element_fn
from integral_timber_joints.planning.utils import beam_ids_from_argparse_seq_n

from integral_timber_joints.assembly.beam_assembly_method import BeamAssemblyMethod
from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotClampAssemblyProcess, Movement, RobotScrewdriverSyncLinearMovement

from compas_fab_pychoreo.client import PyChoreoClient

from pddlstream.utils import read
from pddlstream.language.stream import StreamInfo, PartialInputs, WildOutput, DEBUG
from pddlstream.language.constants import And, PDDLProblem, Equal, print_plan, TOTAL_COST
from pddlstream.language.generator import from_gen_fn, from_fn, from_test

#################################################

# : PyChoreoClient
def get_pddlstream_problem(client, process: RobotClampAssemblyProcess, robot,
        debug=False, reset_to_home=True, seq_n=None,
        use_fluents=True, symbolic_only=False, options=None):
    """Convert a Process instance into a PDDLStream formulation
    """
    if symbolic_only:
        raise DeprecationWarning()
        domain_pddl = read(os.path.join(ITJ_PDDLSTREAM_DEF_DIR, 'symbolic', 'domain.pddl'))
        stream_pddl = read(os.path.join(ITJ_PDDLSTREAM_DEF_DIR, 'symbolic', 'stream.pddl'))
    else:
        domain_pddl = read(os.path.join(ITJ_PDDLSTREAM_DEF_DIR, 'tamp', 'domain.pddl'))
        if not use_fluents:
            stream_pddl = read(os.path.join(ITJ_PDDLSTREAM_DEF_DIR, 'tamp', 'stream.pddl'))
        else:
            stream_pddl = read(os.path.join(ITJ_PDDLSTREAM_DEF_DIR, 'tamp', 'stream_fluents.pddl'))


    process_symdata = process.to_symbolic_problem_data()

    manipulate_cost = 0.0
    init = [
        Equal(('Cost',), manipulate_cost),
        Equal((TOTAL_COST,), 0)
    ]

    # home_conf = process.robot_initial_config
    constant_map = {}

    init.extend([
        ('RobotToolChangerEmpty',),
    ])

    # * Beams
    beam_seq = beam_ids_from_argparse_seq_n(process, seq_n)
    for i, e in enumerate(beam_seq):
        e_data = process_symdata['assembly']['sequence'][i]
        assert e_data['beam_id'] == e
        assert e_data['assembly_method'] != 'UNDEFINED'
        if e_data['assembly_method'] == 'ManualAssembly':
            init.append(('Scaffold', e))
        else:
            init.append(('Element', e))
            init.append((e_data['assembly_method']+'Element', e))
            for sf in e_data['associated_scaffolds']:
                init.append(('AssociatedScaffold', e, sf))

    joints_data = [j_data for j_data in process_symdata['assembly']['joints'] if j_data['joint_id'][0] in beam_seq and \
            j_data['joint_id'][1] in beam_seq]
    for j_data in joints_data:
        j = j_data['joint_id']
        init.extend([
            ('Joint', j[0], j[1]),
        ])

    # * assembly sequence
    cprint('Using beam sequence ordering: {}'.format(beam_seq), 'yellow')
    for e1, e2 in zip(beam_seq[:-1], beam_seq[1:]):
        init.append(('Order', e1, e2))

    # * Clamps
    for c_name in process_symdata['clamps']:
        init.extend([
            ('Clamp', c_name),
            ('Tool', c_name),
            ('AtRack', c_name),
            ('ToolNotOccupiedOnJoint', c_name),
        ])

    # * Screw Drivers
    if 'screwdrivers' in process_symdata:
        for sd_name in process_symdata['screwdrivers']:
            init.extend([
                ('ScrewDriver', sd_name),
                ('Tool', sd_name),
                ('AtRack', sd_name),
                ('ToolNotOccupiedOnJoint', sd_name),
            ])

    # * joint to clamp/scewdriver tool type assignment
    clamp_from_joint = defaultdict(set)
    screwdriver_from_joint = defaultdict(set)
    for j_data in joints_data:
        j = j_data['joint_id']
        joint_clamp_type = j_data['tool_type']
        for c_name, c in process_symdata['clamps'].items():
            if c['type_name'] == joint_clamp_type:
                init.extend([
                    ('JointToolTypeMatch', j[0], j[1], c_name),
                ])
                clamp_from_joint[j[0]+','+j[1]].add(c_name)
        if 'screwdrivers' in process_symdata:
            for sd_name, sd in process_symdata['screwdrivers'].items():
                if sd['type_name'] == joint_clamp_type:
                    init.extend([
                        ('JointToolTypeMatch', j[0], j[1], sd_name),
                    ])
                    screwdriver_from_joint[j[0]+','+j[1]].add(sd_name)

    # * Grippers
    for g_name in process_symdata['grippers']:
        init.extend([
            ('Gripper', g_name),
            ('Tool', g_name),
            ('AtRack', g_name),
        ])
    # * gripper type
    gripper_from_beam = defaultdict(set)
    for beam_data in process_symdata['assembly']['sequence']:
        if not beam_data['beam_id'] in beam_seq:
            continue
        beam_gripper_type = beam_data["beam_gripper_type"]
        for g_name, g_data in process_symdata['grippers'].items():
            if g_data['type_name'] == beam_gripper_type:
                beam_id = beam_data['beam_id']
                init.append(('GripperToolTypeMatch', beam_id, g_name))
                gripper_from_beam[beam_id].add(g_name)
                gripper_from_beam[beam_id].add(g_name)

    if debug:
        stream_map = DEBUG
    else:
        stream_map = {
            'inverse-kinematics':  from_fn(get_ik_fn(client, process, robot, options=options)),
        }

    goal_literals = []
    goal_literals.extend(('Assembled', e) for e in beam_seq)
    if reset_to_home:
        goal_literals.extend(('AtRack', t_name) for t_name in list(process_symdata['clamps']) + list(process_symdata['grippers']))
        if 'screwdrivers' in process_symdata:
            goal_literals.extend(('AtRack', t_name) for t_name in list(process_symdata['screwdrivers']))
    goal = And(*goal_literals)

    pddlstream_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)
    return pddlstream_problem,
