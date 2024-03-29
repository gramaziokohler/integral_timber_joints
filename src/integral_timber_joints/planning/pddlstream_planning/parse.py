import os
import random
from termcolor import cprint
from itertools import chain
from collections import defaultdict

import integral_timber_joints.planning.pddlstream_planning.load_pddlstream
from integral_timber_joints.planning.pddlstream_planning import ITJ_PDDLSTREAM_DEF_DIR
from .action_stream import get_action_ik_fn

from integral_timber_joints.planning.utils import beam_ids_from_argparse_seq_n, LOGGER

from integral_timber_joints.assembly.beam_assembly_method import BeamAssemblyMethod
from integral_timber_joints.process import RoboticFreeMovement, RoboticLinearMovement, RoboticClampSyncLinearMovement, RobotClampAssemblyProcess, Movement, RobotScrewdriverSyncLinearMovement

from compas.geometry import Transformation, Frame
from compas_fab_pychoreo.client import PyChoreoClient

from pddlstream.utils import read
from pddlstream.language.stream import StreamInfo, PartialInputs, WildOutput, DEBUG
from pddlstream.language.constants import And, PDDLProblem, Equal, print_plan, TOTAL_COST
from pddlstream.language.generator import from_gen_fn, from_fn, from_test

#################################################

def get_pddlstream_problem(client: PyChoreoClient, process: RobotClampAssemblyProcess, robot,
        enable_stream=True, reset_to_home=True, seq_n=None,
        use_fluents=True, symbolic_only=False, options=None):
    """Convert a Process instance into a PDDLStream formulation
    """
    options = options or {}

    domain_pddl = read(os.path.join(ITJ_PDDLSTREAM_DEF_DIR, 'tamp', 'domain.pddl'))
    if not use_fluents:
        stream_pddl = read(os.path.join(ITJ_PDDLSTREAM_DEF_DIR, 'tamp', 'stream.pddl'))
    else:
        stream_pddl = read(os.path.join(ITJ_PDDLSTREAM_DEF_DIR, 'tamp', 'stream_fluents.pddl'))

    process_symdata = process.to_symbolic_problem_data()

    manipulate_cost = 5.0
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
    toolchanger = process.robot_toolchanger
    flange_from_toolchanger_base = toolchanger.t_t0cf_from_tcf
    for i, e in enumerate(beam_seq):
        e_data = process_symdata['assembly']['sequence'][i]
        assert e_data['beam_id'] == e
        assert e_data['assembly_method'] != 'UNDEFINED'
        if symbolic_only:
            f_world_from_beam_final = None
        else:
            f_world_from_beam_final = process.assembly.get_beam_attribute(e, 'assembly_wcf_final')
        LOGGER.debug('{} : {}'.format(e, e_data['assembly_method']+'Element'))
        if e_data['assembly_method'] == 'ManualAssembly':
            init.extend([
                ('Scaffold', e),
                ('ElementGoalPose', e, f_world_from_beam_final),
                ])
        else:
            if symbolic_only:
                f_world_from_beam_pickup = None
                flange_from_beam = None
            else:
                f_world_from_beam_pickup = process.assembly.get_beam_attribute(e, 'assembly_wcf_pickup')
                # * get beam grasp
                # ? different gripper might have different grasp for a beam?
                t_gripper_tcf_from_beam = process.assembly.get_t_gripper_tcf_from_beam(e)
                beam_gripper_id = process.assembly.get_beam_attribute(e, "gripper_id")
                beam_gripper = process.tool(beam_gripper_id)
                flange_from_beam = flange_from_toolchanger_base * beam_gripper.t_t0cf_from_tcf * t_gripper_tcf_from_beam

            init.extend([
                ('Element', e),
                #
                ('RackPose', e, f_world_from_beam_pickup),
                ('ElementGoalPose', e, f_world_from_beam_final),
                ('Grasp', e, flange_from_beam),
                ])
            init.append((e_data['assembly_method']+'Element', e))
            for sf in e_data['associated_scaffolds']:
                init.append(('AssociatedScaffold', e, sf))

    init.append(('FirstElement', beam_seq[0]))

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
        c = process.clamp(c_name)
        if symbolic_only:
            tool_storage_frame = None
            clamp_grasp = None
        else:
            tool_storage_frame = c.tool_storage_frame
            clamp_grasp = toolchanger.t_t0cf_from_tcf
        init.extend([
            ('Clamp', c_name),
            ('Tool', c_name),
            ('AtRack', c_name),
            ('ToolNotOccupiedOnJoint', c_name),
            #
            ('RackPose', c_name, tool_storage_frame),
            ('AtPose', c_name, tool_storage_frame),
            ('Grasp', c_name, clamp_grasp),
        ])

    # * Screw Drivers
    if 'screwdrivers' in process_symdata:
        for sd_name in process_symdata['screwdrivers']:
            sd = process.screwdriver(sd_name)
            if symbolic_only:
                tool_storage_frame = None
                sd_grasp = None
            else:
                tool_storage_frame = sd.tool_storage_frame
                sd_grasp = toolchanger.t_t0cf_from_tcf
            init.extend([
                ('ScrewDriver', sd_name),
                ('Tool', sd_name),
                ('AtRack', sd_name),
                ('ToolNotOccupiedOnJoint', sd_name),
                #
                ('RackPose', sd_name, tool_storage_frame),
                ('AtPose', sd_name, tool_storage_frame),
                ('Grasp', sd_name, sd_grasp),
            ])

    # * joint to clamp/scewdriver tool type assignment
    clamp_from_joint = defaultdict(set)
    screwdriver_from_joint = defaultdict(set)
    for j_data in joints_data:
        j = j_data['joint_id']
        joint_clamp_type = j_data['tool_type']
        for c_name, c in process_symdata['clamps'].items():
            if c['type_name'] == joint_clamp_type:
                if symbolic_only:
                    clamp_wcf_final = None
                else:
                    clamp_wcf_final = process.assembly.get_joint_attribute(j, 'clamp_wcf_final')
                init.extend([
                    ('JointToolTypeMatch', j[0], j[1], c_name),
                    # goal pose of the clamp at the joint
                    ('ClampPose', c_name, j[0], j[1], clamp_wcf_final),
                ])
                clamp_from_joint[j[0]+','+j[1]].add(c_name)
        if 'screwdrivers' in process_symdata:
            for sd_name, sd in process_symdata['screwdrivers'].items():
                if sd['type_name'] == joint_clamp_type:
                    screwdriver_from_joint[j[0]+','+j[1]].add(sd_name)

    # * Grippers
    for g_name in process_symdata['grippers']:
        g = process.gripper(g_name)
        if symbolic_only:
            tool_storage_frame = None
            gripper_grasp = None
        else:
            tool_storage_frame = g.tool_storage_frame
            gripper_grasp = toolchanger.t_t0cf_from_tcf
        init.extend([
            ('Gripper', g_name),
            ('Tool', g_name),
            ('AtRack', g_name),
            #
            ('RackPose', g_name, tool_storage_frame),
            ('AtPose', g_name, tool_storage_frame),
            ('Grasp', g_name, gripper_grasp),
        ])
    # * gripper type
    gripper_from_beam = defaultdict(set)
    for beam_data in process_symdata['assembly']['sequence']:
        if not beam_data['beam_id'] in beam_seq:
            continue
        beam_gripper_type = beam_data["beam_gripper_type"]
        if 'screwdrivers' in process_symdata:
            sd_sym_data = process_symdata['screwdrivers'].items()
        else:
            sd_sym_data = []
        for g_name, g_data in chain(process_symdata['grippers'].items(), sd_sym_data):
            if g_data['type_name'] == beam_gripper_type:
                beam_id = beam_data['beam_id']
                init.append(('GripperToolTypeMatch', beam_id, g_name))
                gripper_from_beam[beam_id].add(g_name)
                gripper_from_beam[beam_id].add(g_name)

    if not enable_stream:
        stream_map = DEBUG
    else:
        stream_map = {
            'sample-place_clamp_to_structure':  from_fn(get_action_ik_fn(client, robot, process, 'place_clamp_to_structure', options=options)),
            'sample-pick_clamp_from_structure':  from_fn(get_action_ik_fn(client, robot, process, 'pick_clamp_from_structure', options=options)),
            'sample-beam_placement_without_clamp':  from_fn(get_action_ik_fn(client, robot, process, 'beam_placement_without_clamp', options=options)),
            'sample-beam_placement_with_clamps':  from_fn(get_action_ik_fn(client, robot, process, 'beam_placement_with_clamps', options=options)),
            'sample-assemble_beam_with_screwdrivers_with_gripper':  from_fn(get_action_ik_fn(client, robot, process, 'assemble_beam_with_screwdrivers', options=options)),
            'sample-assemble_beam_with_screwdrivers_without_gripper':  from_fn(get_action_ik_fn(client, robot, process, 'assemble_beam_with_screwdrivers', options=options)),
        }

    goal_literals = []
    goal_literals.extend(('Assembled', e) for e in beam_seq)
    if reset_to_home:
        goal_literals.extend(('AtRack', t_name) for t_name in list(process_symdata['clamps']) + list(process_symdata['grippers']))
        # * screwdriver returning is included in the assemble_beam_with_screwdrivers_* bundled actions
        # if 'screwdrivers' in process_symdata:
        #     goal_literals.extend(('AtRack', t_name) for t_name in list(process_symdata['screwdrivers']))
    goal = And(*goal_literals)

    pddlstream_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)
    return pddlstream_problem,
